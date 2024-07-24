#![no_std]
#![no_main]

use panic_halt as _;

use bsp::entry;
use bsp::hal;
use bsp::hal::fugit::RateExtU32;
use bsp::pac::gclk::clkctrl::GENSELECT_A;
use bsp::pac::gclk::genctrl::SRCSELECT_A;
use bsp::pac::{interrupt, Interrupt};

use cortex_m::peripheral::NVIC;
use cortex_m::prelude::*;
use cortex_m_rt::{exception, ExceptionFrame};

//use hal::prelude::*;
use hal::usb::usb_device::{bus::UsbBusAllocator, prelude::*};
use hal::usb::UsbBus;

mod bsp;
mod data_manager;
mod dutsp;
mod periph_managers;
mod prog_mngr;
mod startup;
mod strs_ctrl;
mod test_management;

use prog_mngr::{CtrlCmd, ProgState};

// Constants
// Message size buffer can't be less than 64 bytes, USB operation starts going awry
const USB_MSG_SIZE_LIM: usize = 64;

// Globals used for USB communication
static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_SER_CTRL: Option<
    usbd_serial::SerialPort<UsbBus, [u8; USB_MSG_SIZE_LIM], [u8; USB_MSG_SIZE_LIM]>,
> = None;
static mut USB_SER_DATA: Option<usbd_serial::SerialPort<UsbBus, [u8; 64], [u8; 528]>> = None;
static mut USB_DEV: Option<UsbDevice<UsbBus>> = None;
// The USB endpoints max packet size limits how much data can be sent in a single transaction
// Ensure that the data buffer sizes only exceed this where that behaviour is fine
static mut DATA_MNGR: data_manager::DataMngr = data_manager::DataMngr::new();
static mut USER_CMD: CtrlCmd = CtrlCmd::NoCmd;
static mut PROG_MSG: Option<[u8; USB_MSG_SIZE_LIM]> = None;
static mut ERR_MSG: Option<[u8; USB_MSG_SIZE_LIM]> = None;

static mut TEST_CHEST: test_management::TestChest = test_management::TestChest::new();

// Globals used for stress control
static mut PWM: Option<hal::pwm::Pwm0> = None;
static mut STRS_CTRLR: Option<strs_ctrl::StrsCtrlr> = None;

#[entry]
fn main() -> ! {
    prog_mngr::swd_print("\nBooting...");

    // Collect the key components of the microcontroller for setup and distribution
    let mut core = hal::pac::CorePeripherals::take().unwrap();
    let mut periphs = hal::pac::Peripherals::take().unwrap();
    let mut clocks = bsp::init_clocks(
        periphs.GCLK,
        &mut periphs.PM,
        &mut periphs.SYSCTRL,
        &mut periphs.NVMCTRL,
    );
    let pins = bsp::Pins::new(periphs.PORT);

    // Configure the RTC and EVSYS to provide an accurate 4.096kHz periodic event
    // Set RTC to use the internal 32kHz precision oscillator
    let tc_clk = clocks.gclk0();
    let rt_clk = clocks.gclk1();
    clocks.rtc(&rt_clk).unwrap();
    let rtc = startup::configure_rtc(periphs.RTC);

    // Configure the on-board SPI bus
    // Set GCLK3 to be a 12MHz clock derived from the 48MHz DFLL, no duty cycle improvement
    clocks.configure_gclk_divider_and_source(GENSELECT_A::GCLK3, 4, SRCSELECT_A::DFLL48M, false);
    clocks.sercom1_core(&tc_clk).unwrap();

    let mut spi = periph_managers::SpiMngr::new(
        &periphs.PM,
        periph_alias!(periphs.board_spi),
        (pins.mosi, pins.miso, pins.sclk),
        (pins.cs_flasmem, pins.cs_dac, pins.cs_pot),
    );

    // Configure the USB device drive, class, and physical bus interface
    unsafe {
        USB_ALLOCATOR = Some(startup::usb_allocator(
            periphs.USB,
            &mut clocks,
            &mut periphs.PM,
            pins.usb_dm,
            pins.usb_dp,
        ));
        USB_SER_CTRL = Some(usbd_serial::SerialPort::new_with_store(
            &USB_ALLOCATOR.as_ref().unwrap(),
            [0u8; USB_MSG_SIZE_LIM],
            [0u8; USB_MSG_SIZE_LIM],
        ));
        USB_SER_DATA = Some(usbd_serial::SerialPort::new_with_store(
            &USB_ALLOCATOR.as_ref().unwrap(),
            [0u8; 64],
            [0u8; 528],
        ));
        USB_DEV = Some(
            UsbDeviceBuilder::new(
                &USB_ALLOCATOR.as_ref().unwrap(),
                UsbVidPid(0x7076, bsp::get_board_id()),
            )
            //.manufacturer("Ivanov SoC Lab")
            //.product("Wearout Test Controller")
            .device_class(0xff)
            .self_powered(true)
            .build(),
        );
        // Enable the USB and periodic stress control interrupts in the processor
        core.NVIC.set_priority(Interrupt::USB, 1);
        NVIC::unmask(Interrupt::USB);
    }

    // Configure the PWM controller for heating the device under test
    pins.dsgp2.into_alternate::<hal::gpio::E>();
    unsafe {
        PWM = Some(hal::pwm::Pwm0::new(
            &clocks.tcc0_tcc1(&tc_clk).unwrap(),
            10.kHz(),
            periphs.TCC0,
            &mut periphs.PM,
        ));
    }

    // Configure the status indicator light manager
    let mut hot_ind = periph_managers::HotIndMngr::new(pins.dsgp3);
    // POSSIBLE TODO: on-board LED manager could be global to flash an LED pattern if hard failures occur
    let mut leds = periph_managers::LedMngr::new(pins.led0, pins.led1, pins.led2, pins.led3);
    // Configure the stress voltage enable pins
    let mut vs_en =
        periph_managers::VstrsEnMngr::new(pins.vsl0_en, pins.vsl1_en, pins.vsh0_en, pins.vsh1_en);

    // Configure the control signal manager
    let mut ctrlr = periph_managers::DutCtrlMngr::new(
        (pins.dsgp0, pins.dsgp1, pins.dsgp4),
        (
            pins.dvcc2lgp0,
            pins.dvcc2lgp1,
            pins.dvcc2lgp2,
            //pins.dvcc2lgp6,
            pins.dvcc2lgp7,
        ),
    );

    // Initialize the ADC manager
    let ana_ins = (
        pins.ain0,
        pins.ain1,
        pins.ain2,
        pins.ain3,
        pins.ain4,
        pins.ain5,
        pins.ain6,
        pins.ain7,
        pins.vsl0_sense,
        pins.vsh0_sense,
        pins.vsl1_sense,
        pins.vsh1_sense,
    );
    let adc = periph_managers::AdcMngr::new(periphs.ADC, &mut periphs.PM, &mut clocks, ana_ins);

    // Configure the frequency measurement counter unit
    let freq_counter = startup::configure_freq_counter(
        periphs.TC4,
        periphs.TC5,
        periphs.TC6,
        &mut periphs.PM,
        &mut periphs.EVSYS,
        clocks,
        (
            pins.mf0.into(),
            pins.mf1.into(),
            pins.mf2.into(),
            pins.mf3.into(),
        ),
    );

    // Setup the global measurement conductor
    let board_sel_pins = (pins.dsgp7.into(), pins.dsgp6.into(), pins.dsgp5.into());
    let chip_sel_pins = (
        pins.dvcc2lgp5.into(),
        pins.dvcc2lgp4.into(),
        pins.dvcc2lgp3.into(),
    );
    let mut meas = periph_managers::MeasMngr::new(
        adc,
        freq_counter,
        board_sel_pins,
        chip_sel_pins,
        pins.dvcc2lgp6.into(),
    );

    // Set up the stress controller
    unsafe { STRS_CTRLR = Some(strs_ctrl::StrsCtrlr::new([true; 4])) };

    // Create the test execution manager
    let mut test_mngr = test_management::TestMngr::new(unsafe { &TEST_CHEST });

    // Set up the program control flow variables
    let mut prog_state = prog_mngr::ProgManager::new();
    // With a u32 counting ticks we can run for just over 17 years before the counter wraps
    let mut curr_tick: u32;
    let mut last_second_tick: u32 = 0;
    let mut data_buffer: data_manager::DataAssembler = data_manager::DataAssembler::new();
    let mut flash_dump_curr_page: u16 = 0;

    prog_mngr::swd_print("Complete\n");

    // === Begin our infinite operational loop ===
    // From this point onwards the program must not panic
    loop {
        // Task 1: Program ticker increments once every 125ms (RTC increments 32 times per second)
        curr_tick = (rtc.mode0().count.read().count().bits() / 4) as u32;

        // Task 2: Measure stress conditions every 125ms to track and control them
        if curr_tick > prog_state.last_strs_meas_tick {
            if let Some(strs_ctrl) = unsafe { STRS_CTRLR.as_mut() } {
                // Update running temperature average
                let mut t_tot: f32 = 0.0;
                let mut t_i: u8 = 0;
                for t in dutsp::STRESS_SENSORS.tsensors {
                    t_tot += w_err_chk(meas.measure(&t), &mut ctrlr, &mut spi);
                    t_i += 1;
                }
                strs_ctrl.update_t_mv_avg(t_tot / t_i as f32, &mut hot_ind);
                // Update running voltage averages
                let mut v = [0.0; 4];
                for i in 0..4 {
                    v[i] = w_err_chk(
                        meas.measure(&dutsp::STRESS_SENSORS.vsensors[i]),
                        &mut ctrlr,
                        &mut spi,
                    );
                }
                for i in 0..4 {
                    strs_ctrl.update_v_mv_avg(v[i], i);
                }
            }
            // Indicate that we took measurements during this tick
            prog_state.last_strs_meas_tick = curr_tick;
        }

        // Task 3: Every half second we feed the current average measurement to the PID controllers
        // and make adjustments to the stress conditions
        if curr_tick > prog_state.last_strs_adjust_tick + 3 {
            if let (Some(strs_ctrl), Some(pwm)) = unsafe { (STRS_CTRLR.as_mut(), PWM.as_mut()) } {
                if strs_ctrl.run_ctrlr(pwm, &mut spi, &mut vs_en).is_err() {
                    build_msg("WARN: Stress control issue", false);
                }
                // Indicate that stress adjustments were made this tick
                prog_state.last_strs_adjust_tick = curr_tick;
            }
            // Also handle blinking LED behaviours here, 0.5 seconds is a good flash rate
            leds.handle_blinks();
        }

        // If in a test running state we track the amount of time we've been testing and check for
        // commands to stop or pause the test
        if prog_state.state != ProgState::Idle
            && prog_state.state != ProgState::FlashDump
            && prog_state.state != ProgState::LoadTest
        {
            // This is in seconds! Only increment duration once per second
            if curr_tick >= last_second_tick + 8 {
                last_second_tick = curr_tick;
            }

            if unsafe { matches!(USER_CMD, CtrlCmd::Stop) } {
                test_mngr.reset_test();
                prog_state.change_state(ProgState::Idle);
                unsafe { USER_CMD = CtrlCmd::NoCmd };
            } else if unsafe { matches!(USER_CMD, CtrlCmd::Pause) } {
                // TODO: Determine program pause behaviour and implement appropriately
                unsafe { USER_CMD = CtrlCmd::NoCmd };
            }
        }

        // Regardless of program state we need to check whether USB data needs to be sent and
        // whether we can send it if so
        unsafe { w_err_chk(DATA_MNGR.try_prep_usb_data(&mut spi), &mut ctrlr, &mut spi) };

        // TODO: Store the test specification in a specific range of the on-board flash chip, allowing
        // for automatic test recovery. Also track time markers to the nearest minute or so, so
        // that even if everything crashes the system can immediately resume the test on reset

        // Run state specific actions
        match prog_state.state {
            ProgState::Idle => {
                // Task 1: If entering the idle state, perform state entry tasks
                if prog_state.entry {
                    // Entry task 1: Ensure idle configuration and stress conditions minimized (room temp, 0V, not meas mode)
                    if ctrlr.set_dig_ctrl(dutsp::DIG_CTRL_DEFAULTS).is_err() {
                        build_msg("WARN: Dig ctrl issue", false);
                    }
                    if ctrlr
                        .set_ana_ctrl(&mut spi, dutsp::ANA_CTRL_DEFAULTS)
                        .is_err()
                    {
                        build_msg("WARN: Ana ctrl issue", false);
                    }
                    leds.set_states_from_byte(0);
                    if let Some(strs_ctrlr) = unsafe { STRS_CTRLR.as_mut() } {
                        strs_ctrlr.disable_all_stress();
                    }

                    // Entry task 2: Clear entry flag
                    prog_state.entry = false;
                }

                // Task 2: Check if user commanded to re-dump entire flash
                if unsafe { matches!(USER_CMD, CtrlCmd::Dump) } {
                    build_msg("INFO: Changing state to dump...", false);
                    prog_state.change_state(ProgState::FlashDump);
                    unsafe { USER_CMD = CtrlCmd::NoCmd };
                }

                // Task 3: Check if user wants to load a new test configuration
                if unsafe { matches!(USER_CMD, CtrlCmd::LoadTest) } {
                    build_msg("INFO: Changing state to load test...", false);
                    prog_state.change_state(ProgState::LoadTest);
                    unsafe { USER_CMD = CtrlCmd::NoCmd };
                }

                // Task 4: Check for control commands to start test, if so change program state
                if unsafe { matches!(USER_CMD, CtrlCmd::Start) } {
                    if unsafe { TEST_CHEST.test_available_to_run } {
                        // Reset data manager state
                        unsafe { DATA_MNGR.new_test_start() };
                        bsp::hard_delay(4_000_000);
                        // Set current time as the test start, reset test timer
                        build_msg("INFO: Starting test...", false);
                        prog_state.change_state(ProgState::CondShift);
                    } else {
                        build_msg("WARN: Can't start, no test...", false);
                    }
                    unsafe { USER_CMD = CtrlCmd::NoCmd };
                }
            }

            ProgState::LoadTest => {
                if prog_state.entry {
                    // Entry task 1: Reset the test loader, removing any existing test
                    unsafe { TEST_CHEST.clear() };
                    leds.set_state(bsp::BoardLed::YELLOW, periph_managers::LedState::BLINK);
                    // Entry task 2: Clear entry flag
                    prog_state.entry = false;
                }

                // Task 2: Check for exit condition to return to idle, occurs when the test has
                // been fully loaded
                if unsafe { TEST_CHEST.test_available_to_run } {
                    leds.set_state(bsp::BoardLed::YELLOW, periph_managers::LedState::OFF);
                    prog_state.change_state(ProgState::Idle);
                    build_msg("INFO: Test loaded successfully...", false);
                }

                // Task 3: If user aborts, return to idle
                if unsafe { matches!(USER_CMD, CtrlCmd::Stop) } {
                    unsafe { TEST_CHEST.clear() };
                    leds.set_state(bsp::BoardLed::YELLOW, periph_managers::LedState::OFF);
                    prog_state.change_state(ProgState::Idle);
                }
            }

            ProgState::FlashDump => {
                if prog_state.entry {
                    // Entry task 1: Reset flash dump page pointer
                    flash_dump_curr_page = 0;
                    // Entry task 2: Reset the USB data packet transfer status
                    unsafe { DATA_MNGR.new_test_start() };
                    leds.set_state(bsp::BoardLed::ORANGE, periph_managers::LedState::BLINK);
                    // Entry task 3: Clear entry flag
                    prog_state.entry = false;
                }

                if unsafe { !DATA_MNGR.data_valid_for_usb_send } {
                    if flash_dump_curr_page >= 8192 {
                        // Task 1: Return to idle if flash empty and all USB data sent
                        leds.set_state(bsp::BoardLed::ORANGE, periph_managers::LedState::OFF);
                        prog_state.change_state(ProgState::Idle);
                    } else {
                        // Task 2: Retrieve next chunk of flash data from the backup chip if DRAM space
                        w_err_chk(
                            unsafe { DATA_MNGR.flash_dump_prep(&mut spi, flash_dump_curr_page) },
                            &mut ctrlr,
                            &mut spi,
                        );
                        flash_dump_curr_page += 1;
                    }
                }

                // If user aborts, return to idle
                if unsafe { matches!(USER_CMD, CtrlCmd::Stop) } {
                    leds.set_state(bsp::BoardLed::ORANGE, periph_managers::LedState::OFF);
                    prog_state.change_state(ProgState::Idle);
                    // Reset the USB data packet transfer status
                    unsafe { DATA_MNGR.new_test_start() };
                    unsafe { USER_CMD = CtrlCmd::NoCmd };
                }
            }

            ProgState::CondShift => {
                // Task 1: Perform state entry items
                if prog_state.entry {
                    // Entry task 1: Get next test step by incrementing counter
                    test_mngr.next_step();
                    // Entry task 2: Set stress condition targets for next test step and reset
                    // stress stability if stress conditions need to change
                    if let Some(strs_ctrlr) = unsafe { STRS_CTRLR.as_mut() } {
                        strs_ctrlr.set_strs_trgts(test_mngr.curr_step);
                    }
                    // Entry task 3: Configure control signals for idle while conditions change
                    w_err_chk(
                        ctrlr.set_dig_ctrl(dutsp::DIG_CTRL_DEFAULTS),
                        &mut ctrlr,
                        &mut spi,
                    );
                    w_err_chk(
                        ctrlr.set_ana_ctrl(&mut spi, dutsp::ANA_CTRL_DEFAULTS),
                        &mut ctrlr,
                        &mut spi,
                    );
                    leds.set_states_from_byte(0);
                    // Entry task 4: Clear state entry flag
                    prog_state.entry = false;
                }

                // Task 2: Check if an automatic program state change should occur either because
                // the test is over or stress conditions have stabilized
                if let Some(next_step) = test_mngr.curr_step {
                    if let Some(strs_ctrl) = unsafe { STRS_CTRLR.as_ref() } {
                        if strs_ctrl.is_strs_stable() {
                            if matches!(next_step.step_type, test_management::StepType::STRS) {
                                prog_state.change_state(ProgState::Stress);
                            } else {
                                prog_state.change_state(ProgState::Measure);
                            }
                        }
                    }
                } else {
                    // If next step is None it means the test is over
                    prog_mngr::swd_print("Test finished!\n");
                    build_msg("INFO: Test Finished!", false);
                    prog_state.change_state(ProgState::Idle);
                }
            }

            ProgState::Stress => {
                // Task 1: Perform state entry items
                if prog_state.entry {
                    build_msg("INFO: Stressing...", false);
                    // Entry task 1: Note the current program tick to track the stress length and
                    // clear the data buffer to prepare for stress monitoring measurements
                    test_mngr.strs_step_start(curr_tick);
                    data_buffer.reset();
                    // Entry task 2: Set control voltages and signals for the stress phase
                    w_err_chk(
                        test_mngr.set_dig_ctrl(unsafe { &TEST_CHEST }, &mut ctrlr, &mut leds),
                        &mut ctrlr,
                        &mut spi,
                    );
                    w_err_chk(
                        test_mngr.set_ana_ctrl(unsafe { &TEST_CHEST }, &mut ctrlr, &mut spi),
                        &mut ctrlr,
                        &mut spi,
                    );
                    // Entry task 3: Set stress stable warning flag to true
                    prog_state.strs_unstable_warning = false;
                    // Entry task 4: Reset state entry flag
                    prog_state.entry = false;
                }

                // Task 2: Check if strs_stable counter has been reset, if so set strs_stable to
                // false and warn user (only warn once per stress step)
                if let Some(strs_ctrl) = unsafe { STRS_CTRLR.as_mut() } {
                    if !strs_ctrl.is_strs_stable() && !prog_state.strs_unstable_warning {
                        let unstable = strs_ctrl.which_cond_unstable();
                        prog_mngr::swd_print_val("Stress unstable, cond:", unstable);

                        let full_str = match unstable {
                            0 => "WARN: Unknown condition drifted outside stable range",
                            1 => "WARN: Temp stress drifted outside stable range",
                            2 => "WARN: V0 stress drifted outside stable range",
                            3 => "WARN: V1 stress drifted outside stable range",
                            4 => "WARN: V2 stress drifted outside stable range",
                            5 => "WARN: V3 stress drifted outside stable range",
                            _ => "CRIT: Impossible unstable condition",
                        };
                        build_msg(full_str, false);
                        prog_state.strs_unstable_warning = true;
                    }
                }

                // Task 3: If 1 minute has passed since the last stress report, record another
                if test_mngr.time_to_rprt_strs(curr_tick) {
                    if let Some(strs_ctrl) = unsafe { STRS_CTRLR.as_ref() } {
                        data_buffer.append_meas(strs_ctrl.get_t_mv_avg());
                        data_buffer.append_meas(strs_ctrl.get_v_mv_avg(0));
                        // TODO: Can only fit one voltage into reports if we want to keep below 528
                        // bytes per hour, evaluate alternative options
                        test_mngr.last_strs_rprt_tick = curr_tick;
                    }
                }

                // Task 4: If stress is ending or one hour has passed (1 measure per min fills a data
                // packet in 1 hour) then we need to submit the data packet to the data manager
                if test_mngr.strs_complete(curr_tick) || test_mngr.time_to_sbmt_strs(curr_tick) {
                    unsafe {
                        w_err_chk(
                            DATA_MNGR.submit_data(&mut spi, data_buffer.get_as_bytes()),
                            &mut ctrlr,
                            &mut spi,
                        );
                    };
                    data_buffer.reset();
                    test_mngr.last_strs_sbmt_tick = curr_tick;
                }

                // Task 5: Once stress period finishes, shift stress conditions for next step
                if test_mngr.strs_complete(curr_tick) {
                    prog_state.change_state(ProgState::CondShift);
                }
            }

            ProgState::Measure => {
                // Task 1: Perform state entry items
                if prog_state.entry {
                    prog_mngr::swd_print("Conducting measurements...");
                    build_msg("INFO: Measuring...", false);
                    // Entry task 1: Prep data buffer and test management trackers
                    test_mngr.meas_step_start();
                    data_buffer.reset();
                    // Entry task 2: Set controls for measure mode, wait for stability
                    w_err_chk(
                        test_mngr.set_dig_ctrl(unsafe { &TEST_CHEST }, &mut ctrlr, &mut leds),
                        &mut ctrlr,
                        &mut spi,
                    );
                    w_err_chk(
                        test_mngr.set_ana_ctrl(unsafe { &TEST_CHEST }, &mut ctrlr, &mut spi),
                        &mut ctrlr,
                        &mut spi,
                    );
                    // This delay should be about 50ms long total to let measure supplies stabilize
                    // In the middle we reset the ring oscillators to ensure they oscillate cleanly
                    bsp::hard_delay(1_200_000);
                    w_err_chk(ctrlr.refresh_ring_oscs(), &mut ctrlr, &mut spi);
                    bsp::hard_delay(1_200_000);
                    // Entry task 3: Clear state entry flag
                    prog_state.entry = false;
                }

                if test_mngr.more_to_measure() {
                    // Check if we've hit the max number of measurements that fit in a data packet
                    if data_buffer.curr_pos >= 528 {
                        unsafe {
                            w_err_chk(
                                DATA_MNGR.submit_data(&mut spi, data_buffer.get_as_bytes()),
                                &mut ctrlr,
                                &mut spi,
                            );
                        };
                        data_buffer.reset();
                    }
                    // Task 2: Collect next requested measurement for the step
                    // Only take one measurement per program loop to ensure that the stress conditions
                    // remain stable, need to keep tracking temperature for the stress controller
                    let measd =
                        w_err_chk(test_mngr.next_measurement(&mut meas), &mut ctrlr, &mut spi);
                    data_buffer.append_meas(measd);
                } else {
                    // Task 3: Once measurements taken, submit data and shift stress conditions for next step
                    unsafe {
                        w_err_chk(
                            DATA_MNGR.submit_data(&mut spi, data_buffer.get_as_bytes()),
                            &mut ctrlr,
                            &mut spi,
                        );
                    };
                    prog_state.change_state(ProgState::CondShift);
                }
            }
        }
    }
}

// **** Interrupt Handlers **** //

#[interrupt]
fn USB() {
    unsafe {
        if let (Some(usb_dev), Some(usb_ctrl), Some(usb_data)) = (
            USB_DEV.as_mut(),
            USB_SER_CTRL.as_mut(),
            USB_SER_DATA.as_mut(),
        ) {
            if usb_dev.poll(&mut [usb_ctrl, usb_data]) {
                let mut buf = [0u8; 64];
                // USB interface has four functional components:
                // 1. Control command receiver to allow simple user control from USB host
                // 2. Status message transmitter to allow basic reporting from device to host
                // 3. Data receiver to load test configurations over USB
                // 4. Data transmitter to send test data in a standard way to the USB host

                // 1. First check for control commands from the host computer
                match usb_ctrl.read(&mut buf[..]) {
                    Ok(_count) => {
                        match buf[0] {
                            // The zero command is sent frequently to poll this device for new data
                            0x00 => {}
                            0x01 => USER_CMD = CtrlCmd::Start,
                            0x02 => USER_CMD = CtrlCmd::Stop,
                            0x03 => USER_CMD = CtrlCmd::Pause,
                            0x04 => USER_CMD = CtrlCmd::Dump,
                            0x05 => USER_CMD = CtrlCmd::LoadTest,
                            _ => build_msg("INFO: Invalid command, ignoring...", false),
                        };
                    }
                    // WouldBlock error indicates no data is available to read
                    Err(UsbError::WouldBlock) => {}
                    Err(_) => build_msg("WARN: Failed to read user command over USB", false),
                };

                // 2. If in the load test state, check for the next packet of test config data
                if TEST_CHEST.awaiting_test_data {
                    match usb_data.read(&mut buf[..]) {
                        Ok(_count) => {
                            // Have the test chest add the test configuration item to its memory,
                            // optionally providing a warning if there are issues with the config
                            if let Some(w) = TEST_CHEST.input_next(&buf) {
                                build_msg(w, false)
                            };
                        }
                        Err(UsbError::WouldBlock) => {}
                        Err(_) => {
                            build_msg("WARN: Failed to read test config data over USB", false)
                        }
                    }
                };

                // 3. Have the test controller optionally send status updates to the host via USB
                if let Some(m) = ERR_MSG {
                    // Error messages get priority over standard updates or warnings
                    if usb_ctrl.write(&m).is_ok() {
                        ERR_MSG = None;
                    }
                } else if let Some(m) = PROG_MSG {
                    if usb_ctrl.write(&m).is_ok() {
                        PROG_MSG = None;
                    }
                }

                // 4. If the data manager is ready for a USB transfer, go for it
                if DATA_MNGR.data_valid_for_usb_send {
                    // Check that the software buffer is fully empty and ready for another packet
                    if usb_data.flush().is_ok() {
                        if usb_data.write(&DATA_MNGR.curr_usb_data).is_ok() {
                            // Clear main program to USB data transfer flag
                            DATA_MNGR.data_valid_for_usb_send = false;
                        }
                    }
                }
            }
        };
    }
}

// *** Error handling functions *** //

#[exception]
unsafe fn HardFault(_: &ExceptionFrame) -> ! {
    //if let Some(strs_ctrlr) = unsafe { STRS_CTRLR.as_mut() } {
    //disable_all_stress(&mut spi, &mut ctrlr, &mut strs_ctrlr);
    //strs_ctrlr.temp_ctrl_en = false;
    //strs_ctrlr.volt_ctrl_enabled = false;
    //}
    prog_mngr::swd_print("Hard fault occurred!\n");
    if let Some(pwm) = unsafe { PWM.as_mut() } {
        pwm.set_duty(hal::pwm::Channel::_0, 0);
    }
    loop {}
}

fn build_msg(msg: &str, err: bool) {
    // Create program message
    let mut msg_buf = [0u8; USB_MSG_SIZE_LIM];
    for (i, c) in msg.bytes().enumerate() {
        if i < USB_MSG_SIZE_LIM {
            msg_buf[i] = c;
        }
    }
    if err {
        unsafe { ERR_MSG = Some(msg_buf) };
    } else {
        unsafe { PROG_MSG = Some(msg_buf) };
    }
}

fn w_err_chk<T>(
    r: Result<T, &'static str>,
    ctrlr: &mut periph_managers::DutCtrlMngr,
    spi: &mut periph_managers::SpiMngr,
) -> T {
    if let Err(e) = r {
        prog_mngr::swd_print_val("CRIT:", e);
        if unsafe { ERR_MSG } == None {
            build_msg(e, true);
        }

        // Try to put DUT in idle state with no stress
        if let Some(strs_ctrlr) = unsafe { STRS_CTRLR.as_mut() } {
            ctrlr.set_dig_ctrl(dutsp::DIG_CTRL_DEFAULTS).ok();
            ctrlr.set_ana_ctrl(spi, dutsp::ANA_CTRL_DEFAULTS).ok();
            strs_ctrlr.disable_all_stress();
        }

        if let Some(pwm) = unsafe { PWM.as_mut() } {
            pwm.set_duty(hal::pwm::Channel::_0, 0);
        }

        // Go to the error state idle
        prog_mngr::swd_print("Critical error! Fail idling...");
        crit_idle();
    }
    r.unwrap()
}

fn crit_idle() {
    // This function never returns
    loop {
        // Safe failure task 1: Check if user wants to dump the flash memory
        if unsafe { matches!(USER_CMD, CtrlCmd::Dump) } {
            // TODO
            //for page in unsafe { DATA_MNGR.dump_flash() } {}
            unsafe { USER_CMD = CtrlCmd::NoCmd };
        }

        // Safe failure task 2: ???
    }
}
