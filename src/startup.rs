use atsamd_hal::clock::GenericClockController;
use atsamd_hal::pac::{EVSYS, PM, RTC, TC6, TC4, TC5, USB};
use atsamd_hal::usb::{usb_device::bus::UsbBusAllocator, UsbBus};
use bsp::pac::gclk::clkctrl::GENSELECT_A;
use bsp::pac::gclk::genctrl::SRCSELECT_A;

use crate::bsp;
use crate::periph_managers::FreqCounter;

pub fn configure_rtc(rtc: RTC) -> RTC {
    // Set RTC to mode 0 by resetting, set the prescaler to 1024 for a 4096 Hz tick rate
    rtc.mode0()
        .ctrl
        .write(|w| unsafe { w.prescaler().bits(0xA) });
    // Setup the event system to output an event every 8 input clock cycles
    rtc.mode0().evctrl.write(|w| w.pereo0().set_bit());
    // This must follow the previous enable-protected writes as it enables the RTC
    rtc.mode0().ctrl.modify(|_, w| w.enable().set_bit());
    // Finally, wait until the RTC is fully enabled before moving on, good for posterity
    let mut enabled = false;
    while !enabled {
        enabled = !rtc.mode0().status.read().syncbusy().bit();
    }
    rtc
}

/// Convenience function for setting up USB
pub fn usb_allocator(
    usb: USB,
    clocks: &mut GenericClockController,
    pm: &mut PM,
    dm: impl Into<bsp::UsbDm>,
    dp: impl Into<bsp::UsbDp>,
) -> UsbBusAllocator<UsbBus> {
    let gclk0 = clocks.gclk0();
    let clock = &clocks.usb(&gclk0).unwrap();
    let (dm, dp) = (dm.into(), dp.into());
    let bus = UsbBus::new(clock, pm, dm, dp, usb);
    UsbBusAllocator::new(bus)
}

/// Startup function to configure the frequency measurement counter units
pub fn configure_freq_counter(
    tc: TC4,
    tce: TC5,
    tcl: TC6,
    pm: &mut PM,
    mut evsys: &mut EVSYS,
    mut clocks: GenericClockController,
    mut freq_inputs: (bsp::MF0, bsp::MF1, bsp::MF2, bsp::MF3),
) -> FreqCounter {
    // Configure peripheral bus clock masking
    pm.apbcmask
        .modify(|_, w| w.evsys_().set_bit().tc4_().set_bit().tc5_().set_bit()
                .tc6_().set_bit());

    // Configure the GCLK inputs to accept the input frequencies
    freq_inputs.0 = freq_inputs.0.into_alternate::<atsamd_hal::gpio::H>();
    freq_inputs.1 = freq_inputs.1.into_alternate::<atsamd_hal::gpio::H>();
    freq_inputs.2 = freq_inputs.2.into_alternate::<atsamd_hal::gpio::H>();
    freq_inputs.3 = freq_inputs.3.into_alternate::<atsamd_hal::gpio::H>();
    // Configure GCLK4/5 to be generated from the input frequencies, no duty cycle improvement
    clocks.configure_gclk_divider_and_source(GENSELECT_A::GCLK4, 1, SRCSELECT_A::GCLKIN, false);
    clocks.configure_gclk_divider_and_source(GENSELECT_A::GCLK5, 1, SRCSELECT_A::GCLKIN, false);
    clocks.configure_gclk_divider_and_source(GENSELECT_A::GCLK6, 1, SRCSELECT_A::GCLKIN, false);
    clocks.configure_gclk_divider_and_source(GENSELECT_A::GCLK7, 1, SRCSELECT_A::GCLKIN, false);
    // Configure the MCU to allow for frequency measurement, takes ownership of all clocks
    let gclk0 = clocks.gclk0();
    //clocks.tcc2_tc3(&gclk0).unwrap();
    clocks.tc4_tc5(&gclk0).unwrap();
    clocks.tc6_tc7(&gclk0).unwrap();

    let gclk4 = clocks.get_gclk(GENSELECT_A::GCLK4).unwrap();
    let gclk5 = clocks.get_gclk(GENSELECT_A::GCLK5).unwrap();
    let gclk6 = clocks.get_gclk(GENSELECT_A::GCLK6).unwrap();
    let gclk7 = clocks.get_gclk(GENSELECT_A::GCLK7).unwrap();

    FreqCounter::new(&mut evsys, clocks, tc, tce, tcl, gclk4, gclk5, gclk6, gclk7, freq_inputs)
}
