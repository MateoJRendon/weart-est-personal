use core::ptr;

use crate::bsp;
use crate::bsp::ehal::digital::StatefulOutputPin;
use crate::bsp::hal::clock::{GClock, GenericClockController};
use crate::bsp::hal::pac::{ADC, EVSYS, PM, TC6, TC4, TC5};
use crate::bsp::hal::prelude::*;
use crate::bsp::hal::sercom;
use crate::dutsp::MeasConfigNew;

const CS_SET_ERROR_MSG: &str = "CRIT: Failed to set CS pin state";
const SPI_TRANSFER_ERROR_MSG: &str = "CRIT: SPI transaction failed";

const CALIB_ADDR: u32 = 0x806020;

// **** Helper Function to Read NVM Calibration Values **** //
fn get_cal_val(addr_offset: u32, bit_shift: u32, bit_mask: u32) -> u32 {
    let addr: *const u32 = (CALIB_ADDR + addr_offset) as *const _;
    let value = unsafe { ptr::read_unaligned(addr) };
    (value >> bit_shift) & bit_mask
}

// **** Custom ADC Interface Manager **** //
pub struct AdcMngr {
    adc: ADC,
    channels: (
        bsp::AnaIn0,
        bsp::AnaIn1,
        bsp::AnaIn2,
        bsp::AnaIn3,
        bsp::AnaIn4,
        bsp::AnaIn5,
        bsp::AnaIn6,
        bsp::AnaIn7,
        bsp::Vsl0Sense,
        bsp::Vsh0Sense,
        bsp::Vsl1Sense,
        bsp::Vsh1Sense,
    ),
    v_ref: f32,
    gain: f32,
}

impl AdcMngr {
    pub fn new(
        mut adc: ADC,
        pm: &mut PM,
        clocks: &mut GenericClockController,
        pins: (
            impl Into<bsp::AnaIn0>,
            impl Into<bsp::AnaIn1>,
            impl Into<bsp::AnaIn2>,
            impl Into<bsp::AnaIn3>,
            impl Into<bsp::AnaIn4>,
            impl Into<bsp::AnaIn5>,
            impl Into<bsp::AnaIn6>,
            impl Into<bsp::AnaIn7>,
            impl Into<bsp::Vsl0Sense>,
            impl Into<bsp::Vsh0Sense>,
            impl Into<bsp::Vsl1Sense>,
            impl Into<bsp::Vsh1Sense>,
        ),
    ) -> AdcMngr {
        let adc_pins = (
            pins.0.into(),
            pins.1.into(),
            pins.2.into(),
            pins.3.into(),
            pins.4.into(),
            pins.5.into(),
            pins.6.into(),
            pins.7.into(),
            pins.8.into(),
            pins.9.into(),
            pins.10.into(),
            pins.11.into(),
        );

        // Enable and set up the clock for the ADC peripheral
        pm.apbcmask.modify(|_, w| w.adc_().set_bit());
        // GCLK0 will be 48MHz, the ADC clock is set to be the input clock divided by 4,
        // thus the ADC is clocked at 12MHz maximum
        let gclk0 = clocks.gclk0();
        clocks.adc(&gclk0).unwrap();

        let mut mngr = AdcMngr {
            adc,
            channels: adc_pins,
            v_ref: 1.0,
            gain: 0.5,
        };
        // Configure the ADC peripheral according to the board use case
        // Most of the configuration matches the default. Reference select defaults to INT1V,
        // default 12bit operation, no averaging or sampling time changes needed

        // We will reduce the ADC clock significantly as we don't need measurements every 500ns
        mngr.adc.ctrlb.modify(|_, w| w.prescaler().bits(0x7));
        // Set the input gain to 1/2 to allow for up to 2V analog signals to be measured
        mngr.adc
            .inputctrl
            .modify(|_, w| unsafe { w.gain().bits(0xF) });
        // Set negative input channel to ground for proper single-ended operation
        mngr.adc.inputctrl.modify(|_, w| w.muxneg().gnd());
        // Not currently applying any gain or offset correction

        // Load the ADC calibration values from NVM
        let adc_linearity = get_cal_val(3, 3, 0x00FF) as u8;
        let adc_biascal = get_cal_val(3, 11, 0x0007) as u8;
        mngr.adc
            .calib
            .modify(|_, w| unsafe { w.linearity_cal().bits(adc_linearity) });
        mngr.adc
            .calib
            .modify(|_, w| unsafe { w.bias_cal().bits(adc_biascal) });

        // Finally, with the registers configured, power up the peripheral and wait for readiness
        mngr.adc.ctrla.modify(|_, w| w.enable().set_bit());
        while mngr.adc.status.read().syncbusy().bit_is_set() {}

        // Take an initial measurement, as the first measurement after the voltage reference
        // changes will be garbage, so get that out of the way now
        mngr.read(0x00).unwrap();

        mngr
    }

    fn read(&mut self, channel: u8) -> Result<u32, &'static str> {
        while self.adc.status.read().syncbusy().bit_is_set() {}

        self.adc
            .inputctrl
            .modify(|_, w| unsafe { w.muxpos().bits(channel) });
        let result = self.convert();

        Ok(result.into())
    }

    fn power_up(&mut self) {
        while self.adc.status.read().syncbusy().bit_is_set() {}
        self.adc.ctrla.modify(|_, w| w.enable().set_bit());
        while self.adc.status.read().syncbusy().bit_is_set() {}
    }

    fn power_down(&mut self) {
        while self.adc.status.read().syncbusy().bit_is_set() {}
        self.adc.ctrla.modify(|_, w| w.enable().clear_bit());
        while self.adc.status.read().syncbusy().bit_is_set() {}
    }

    fn convert(&mut self) -> u16 {
        // Clear the interrupt flag
        self.adc.intflag.modify(|_, w| w.resrdy().set_bit());
        // Trigger an ADC measurement
        self.adc.swtrig.modify(|_, w| w.start().set_bit());
        // Wait for the result to be ready and the register synchronized
        while self.adc.intflag.read().resrdy().bit_is_clear() {}
        while self.adc.status.read().syncbusy().bit_is_set() {}

        self.adc.result.read().result().bits()
    }

    pub fn meas_val(&mut self, channel: u8) -> Result<f32, &'static str> {
        // TODO: Make sure this works with all 12 ADC input channels
        let code = self.read(channel)?;
        Ok(self.code_to_volts(code as u16))
    }

    pub fn meas_slope(&mut self, channel: u8) -> Result<f32, &'static str> {
        // Set the correct channel
        self.adc
            .inputctrl
            .modify(|_, w| unsafe { w.muxpos().bits(channel) });

        // We will only measure slope for sub-1V signals, so increase the gain to x1
        self.adc
            .inputctrl
            .modify(|_, w| unsafe { w.gain().bits(0x0) });
        self.gain = 1.0;

        // Place the ADC into freerunning mode for continuous conversions
        self.adc.ctrlb.modify(|_, w| w.freerun().set_bit());

        // Clear the resrdy flag in case it wasn't cleared earlier
        self.adc.intflag.modify(|_, w| w.resrdy().set_bit());

        let mut curr_val: u16 = 0;                              // 12-bit ADC reading (0 to 65536 -> Expected 0 to 4095)
        let mut prev_val: u16;                                  // 12-bit ADC reading (0 to 65536 -> Expected 0 to 4095)
        let mut last_five_diffs: [i16; 5] = [0, 0, 0, 0, 0];    // Difference of 12-bit ADC readings (-32768 to 32768 -> Worst case +-4095)

        let mut i: u8 = 0;                  // Counting variable (0 to 255 -> Expected 0 to 50)
        let mut short_term_sum: i16 = 400;  // Sum of the last_five_diffs (-32768 to 32768 -> Expected 0 to 4095)
        let mut delayed_sum: i16 = 0;       // Sum of all last_five_diffs[0] (-32768 to 32768 -> w.c. 0 to 3377 0.8V)      

        let sampling_rate: u16 = 13393;     // Sampling rate in which the ADC is operating

        // Start repeatedly measuring the input
        self.adc.swtrig.modify(|_, w| w.start().set_bit());

        // Minimum slope of interest at time zero is 10.67 V/s
        // Maximum slope of interest at time zero is 836.36 V/s
        // In free-running mode, our time between ADC conversions for 1x gain, 512x clock prescaler,
        // 12 bit resolution, and a 48MHz Gclk0 will be ((12 bits / 2) + 1) / 93.75kHz = 74.667us
        // This comes from Free-running mode equation in p.784 of the datasheet.
        // 74.667us corresponds to 13.393k samples per second
        // Each ADC code with 12 bits in a 1V range is ~244uV
        // LIMITS:
        // Fastest slope: Consider a range of 400mV (due to roll-off), 5 differences, and 74.667us sampling
        // V_range/(n_array_length * t_sample) = 1071.43 [V/s]
        // Lowest slope: 20mV noise and 74.667us sampling 
        // V_noise/t_sample = 267.857 [V/s]

        // 328 corresponds to 80mV
        while (short_term_sum > 328) && (i < 50) {
            // Shift the history of signal differences
            for i in 0..4 {
                last_five_diffs[i] = last_five_diffs[i + 1];
            }
            // Obtain the next measured value
            prev_val = curr_val;
            while self.adc.intflag.read().resrdy().bit_is_clear() {}
            while self.adc.status.read().syncbusy().bit_is_set() {}
            curr_val = self.adc.result.read().result().bits();

            // Except for the first iteration, calculate the difference
            if i != 0 {
                last_five_diffs[4] = prev_val as i16 - curr_val as i16;
            }
            // Obtain the latest 4 values after 4 cycles
            if i >= 4 {
                short_term_sum = last_five_diffs[1].abs() + last_five_diffs[2].abs() + last_five_diffs[3].abs() + last_five_diffs[4].abs();
            }
            // We accumulate with delay to ensure we ignore the last few samples as the avg zeros
            delayed_sum += last_five_diffs[0];
            i += 1;
        }

        // Revert the freerun mode and gain changes
        self.adc.ctrlb.modify(|_, w| w.freerun().clear_bit());
        self.adc
            .inputctrl
            .modify(|_, w| unsafe { w.gain().bits(0xF) });

        // If we exited due to the voltage saturating we ignore the last few measurements
        let mut final_val: f32 = 0.0;

        if short_term_sum <= 328 {
            delayed_sum -= last_five_diffs[0];
            if i > 6 {
                final_val = delayed_sum as f32 / (i - 6) as f32;
            }
        // If we exited because we hit our iteration limit we can use all the measured values
        } else {
            delayed_sum += last_five_diffs[1] + last_five_diffs[2] + last_five_diffs[3] + last_five_diffs[4];
            final_val = delayed_sum as f32 / (i - 1) as f32;
        }
        let avg_diff_volts = self.code_to_volts_f32(final_val);
        let final_slope = avg_diff_volts * sampling_rate as f32;
        self.gain = 0.5;
        Ok(final_slope)
    }
    
    pub fn naive_meas_slope(&mut self, channel: u8) -> Result<f32, &'static str> {
        // Set the correct channel
        self.adc
            .inputctrl
            .modify(|_, w| unsafe { w.muxpos().bits(channel) });

        // We will only measure slope for sub-1V signals, so increase the gain to x1
        self.adc
            .inputctrl
            .modify(|_, w| unsafe { w.gain().bits(0x0) });
        self.gain = 1.0;

        // We will also set the measurements to be faster for this algorithm to increase the range. 
        // 5 corresponds to a division of x128
        self.adc.ctrlb.modify(|_, w| w.prescaler().bits(0x5));

        // Place the ADC into freerunning mode for continuous conversions
        self.adc.ctrlb.modify(|_, w| w.freerun().set_bit());

        // Clear the resrdy flag in case it wasn't cleared earlier
        self.adc.intflag.modify(|_, w| w.resrdy().set_bit());

        let mut curr_val: u16 = 10000;      // 12-bit ADC reading (0 to 65535 -> Expected 0 to 4095)
        let mut prev_val: u16;              // 12-bit ADC reading (0 to 65535 -> Expected 0 to 4095)
        let mut decrease: i16 = 0;          // Difference per iteration (-32768 to 32768 -> Expected +- 4095)

        let mut i: u8 = 0;                  // Counting variable (0 to 255 -> Expected 0 to 200)
        let mut delayed_sum: i16 = 0;       // Sum of all last_five_diffs [0] (-32768 to 32768 -> w.c. 0 to 3377 0.8V)

        let sampling_rate: u16 = 53571;     // Corresponds to 74.667us

        let curr_val_condition: u16 = 1638; // Corresponds to 0.4mV
        let n_samples: u8 = 200;            // Maximum samples until timeout

        // Start repeatedly measuring the input
        self.adc.swtrig.modify(|_, w| w.start().set_bit());

        // Minimum slope of interest at time zero is 10.67 V/s
        // Maximum slope of interest at time zero is 836.36 V/s
        // In free-running mode, our time between ADC conversions for 1x gain, 128x clock prescaler,
        // 12 bit resolution, and a 48MHz Gclk0 will be ((12 bits / 2) + 1) / 375kHz = 18.667us
        // This comes from Free-running mode equation in p.784 of the datasheet.
        // 18.667us corresponds to 53.570k samples per second
        // Each ADC code with 12 bits in a 1V range is ~244uV
        // LIMITS:
        // Fastest slope: Consider a range of 350mV (From 700mV to 350mV), and 18.667us sampling
        // V_range/(2 * t_sample) = 9374.8 [V/s]
        // Lowest slope: 20mV noise, 200 samples, and 18.667us sampling 
        // V_noise/n_samples * t_sample = 10.714 [V/s]

        while (curr_val > curr_val_condition) && (i < n_samples) {

            // Obtain the next measured value
            prev_val = curr_val;
            while self.adc.intflag.read().resrdy().bit_is_clear() {}
            while self.adc.status.read().syncbusy().bit_is_set() {}
            curr_val = self.adc.result.read().result().bits();

            // Except for the first iteration, calculate the difference
            if i != 0 {  
                decrease = prev_val as i16 - curr_val as i16;
                // We accumulate with delay to ensure we ignore the last few samples as the avg zeros
                delayed_sum += decrease;
            }
 
            i += 1;
        }

        // Revert the freerun mode and gain changes
        self.adc.ctrlb.modify(|_, w| w.freerun().clear_bit());
        self.adc
            .inputctrl
            .modify(|_, w| unsafe { w.gain().bits(0xF) });
        self.adc.ctrlb.modify(|_, w| w.prescaler().bits(0x7));    

        // If we exited due to the voltage saturating we ignore the last few measurements
        let mut final_val: f32 = 0.0;

        if (curr_val <= curr_val_condition) && (i > 2) {
            delayed_sum -= decrease;
            final_val = delayed_sum as f32 / (i - 2) as f32;
        // If we exited because we hit our iteration limit we can use all the measured values
        } else if i >= n_samples {
            final_val = delayed_sum as f32 / (i - 1) as f32;
        }

        let avg_diff_volts = self.code_to_volts_f32(final_val);
        let final_slope = avg_diff_volts * sampling_rate as f32;
        self.gain = 0.5;
        Ok(final_slope)
    }

    // Take an ADC code and convert it to the corresponding voltage
    fn code_to_volts(&self, code: u16) -> f32 {
        (code as f32 / 4096.0) * self.v_ref * (1.0 / self.gain)
    }

    // Code to volts without type conversion
    fn code_to_volts_f32(&self, code: f32) -> f32 {
        (code / 4096.0) * self.v_ref * (1.0 / self.gain)
    }
}

// **** Frequency Measurement Manager **** //
pub struct FreqCounter {
    clks: GenericClockController,
    tc: TC4,
    tcl: TC6,
    freq0: GClock,
    freq1: GClock,
    freq2: GClock,
    freq3: GClock,
    pins: (bsp::MF0, bsp::MF1, bsp::MF2, bsp::MF3),
}

impl FreqCounter {
    pub fn new(
        evsys: &mut EVSYS,
        clks: GenericClockController,
        tc: TC4,
        tce: TC5,
        tcl: TC6,
        freq_src_0: GClock,
        freq_src_1: GClock,
        freq_src_2: GClock,
        freq_src_3: GClock,
        mut pins: (bsp::MF0, bsp::MF1, bsp::MF2, bsp::MF3),
    ) -> FreqCounter {
        // Ensure INEN is enabled for all four pins, which will allow for the pin state to be read
        // even though they are configured as clock inputs. Refer to datasheet section 23.8.11 and
        // 32.8.13. By default they are enabled, but leaving this commented code here in case the
        // HAL changes in the future
        //let port_b_wrconfig: *mut u32 = 0x410044A8 as *mut _;
        //let cmd = 0b01000000_00000011_00111100_00000000;
        //unsafe { ptr::write(port_b_wrconfig, cmd) };

        // Now configure the event system to send the RTC periodic event to the TC/TCC units
        // Set channel 0 to accept the RTC periodic event asynchronously
        evsys
            .channel
            .write(|w| unsafe { w.channel().bits(0x0).path().bits(0x2).evgen().bits(0x04) });
        // Set users TC4 to accept the channel 0 event (note that the channel value is -1 from the
        // value written as a value 0x0 disables events to the user, yes it's very confusing)
        evsys
            .user
            .write(|w| unsafe { w.user().bits(0x13).channel().bits(0x1) });

        // Set channel 1 to propagate the TC5 overflow output event asynchronously
        evsys
            .channel
            .write(|w| unsafe { w.channel().bits(0x1).path().bits(0x02).evgen().bits(0x39) });
        // Set TC6 to accept the TC5 overflow event to trigger counter stop
        evsys
            .user
            .write(|w| unsafe { w.user().bits(0x15).channel().bits(0x2) });

        // Now set up the TC/TCC unit to consume the events and count pulses of the clocks
        // Set CC0 to capture the counter value on each event, see 30.6.2.7 in datasheet
        tc.count16().ctrlc.write(|w| w.cpten0().set_bit());
        tc.count16().intenset.write(|w| w.ovf().set_bit());
        // Set up the event system to accept asynchronous events, no specific event action needed
        tc.count16().evctrl.write(|w| w.tcei().set_bit());
        // Last step is to enable the unit, all other CTRLA fields match reset values
        tc.count16().ctrla.write(|w| w.enable().set_bit());
        //.write(|w| w.prescaler().bits(0x1).enable().set_bit());

        // Low frequency measurement setup
        // CC0 is the TOP value that resets the counter, we want to wrap every other cycle
        tce.count16().cc[0].write(|w| unsafe { w.cc().bits(0x0001) });
        // Generate an event every time the counter wraps
        tce.count16().evctrl.write(|w| w.ovfeo().set_bit());
        // Use wavegen MFRQ (match frequency) mode to have the counter wrap on a preset value
        tce.count16().ctrla.write(|w| w.wavegen().bits(0x1).enable().set_bit());

        // TC6 to count main clock cycles between events
        tcl.count32().ctrlc.write(|w| w.cpten0().set_bit());
        tcl.count32().evctrl.write(|w| w.tcei().set_bit());

        tcl.count32().ctrla.write(|w| unsafe { w.mode().bits(0x2).enable().set_bit() });

        // Finally, wait until the RTC and TC/TCC are fully enabled before moving on
        // This is unlikely to be necessary but is good for posterity
        let mut enabled = false;
        while !enabled {
            enabled = !tc.count16().status.read().syncbusy().bit()
                && !tce.count16().status.read().syncbusy().bit()
                && !tcl.count32().status.read().syncbusy().bit();
        }

        // Return the constructed frequency counter
        FreqCounter {
            clks,
            tc,
            tcl,
            freq0: freq_src_0,
            freq1: freq_src_1,
            freq2: freq_src_2,
            freq3: freq_src_3,
            pins,
        }
    }

    fn wait_for_sync(&self) -> bool {
        let mut i: u16 = 0;
        while i < 1000 {
            if !self.tc.count16().status.read().syncbusy().bit() {
                return true;
            }
            i += 1;
        }
        false
    }

    fn read_pin_as_dig_in(&self, chnl: bsp::FreqMeasChnl) -> bool {
        //let addr: *const u32 = (CALIB_ADDR + addr_offset) as *const _;
        let port_b_in_reg: *const u32 = 0x410044A0 as *const _;
        let pin_lvls = unsafe { ptr::read(port_b_in_reg) };
        let mask = match chnl {
            bsp::FreqMeasChnl::CH0 => 0b00000000_00000000_00010000_00000000, // PB12
            bsp::FreqMeasChnl::CH1 => 0b00000000_00000000_00100000_00000000, // PB13
            bsp::FreqMeasChnl::CH2 => 0b00000000_00000000_00001000_00000000, // PB11
            bsp::FreqMeasChnl::CH3 => 0b00000000_00000000_00000100_00000000, // PB10
        };
        (pin_lvls & mask) != 0
    }

    fn read_count(&self) -> Result<u16, u16> {
        // The read of the count register completely crashes/stalls the MCU if either the peripheral
        // clock or bus clock are halted or extremely slow, so we have to ensure the value is
        // synchronized before reading so this doesn't ever hang
        if self.wait_for_sync() {
            self.tc.count16().readreq.write(|w| w.rreq().set_bit());
        }
        // MATEO COMMENT: Unclear on how to add the tc1 in the return here.
        if self.wait_for_sync() {
            return Ok(self.tc.count16().cc[0].read().cc().bits());
        }
        // Frequency was too low to measure! Return 0 frequency to indicate this
        Err(0)
    }

    pub fn get_freq(&mut self, chnl: bsp::FreqMeasChnl) -> Result<f32, &'static str> {
        let source = match chnl {
            bsp::FreqMeasChnl::CH0 => &self.freq0,
            bsp::FreqMeasChnl::CH1 => &self.freq1,
            bsp::FreqMeasChnl::CH2 => &self.freq2,
            bsp::FreqMeasChnl::CH3 => &self.freq3,
        };
        self.clks
            .tc4_tc5(source)
            .ok_or("CRIT: Could not set TC source clock")?;

        // If frequency is too low to measure using this method, just call it 0
        let mut freq = self.get_count_diff().unwrap_or(0);
        if freq <= 80_000 {
            freq = self.get_low_freq();
        }
        Ok(freq as f32)
    }

    fn get_low_freq(&self) -> u32 {
        // The TC unit for low frequency uses the core clock so we don't have to worry
        // about the synchronization causing bus stall issue that makes reading high frequencies
        // challenging
        let flush = self.tcl.count32().cc[0].read().cc().bits();
        let mut count1 = flush;
        let mut i: u32 = 0;
        while count1 == flush {
            i += 1;
            if i > 5_000_000 {
                break;
            }
            count1 = self.tcl.count32().cc[0].read().cc().bits();

        }
        let mut count2 = count1;
        while count2 == count1 {
            i += 1;
            if i > 5_000_000 {
                break;
            }
            count2 = self.tcl.count32().cc[0].read().cc().bits();
        }

        // Small chance that the counter wrapped within the period of interest
        // Multiply by two as the events are only generated every other cycle of the frequency
        if count2 == count1 {
            0
        } else if count2 < count1 {
            let diff: u32 = (count1 - count2).into();
            // Subtract from 2^32 to get the frequency despite the wraparound (the +1 is needed
            // since 2^32 doesn't fit in a u32)
            (48_000_000 * 2) / ((4_294_967_295 - diff) + 1)
        } else {
            let diff: u32 = (count2 - count1).into();
            (48_000_000 * 2) / diff
        }
    }

    fn get_count_diff(&self) -> Result<u32, u32> {
        // The synchronization time of the CC0 register depends on the frequency of the TC gclk,
        // which means the slower the clock, the more time it takes to synchronize
        // If the clock isn't running, synchronization will never occur and attempting a read
        // will stall the entire peripheral bus
        // Read once first to see the previously requested synchronized value
        let count_flush = self.read_count()?;
        // Now request a new value and wait for RTC event to update CC0
        let mut count1 = self.read_count()?;
        let mut i: u16 = 0;
        while count1 == count_flush {
            count1 = self.read_count()?;
            // Add exit logic just in case a frequency is exactly synchronous to the events
            i += 1;
            if i > 1000 {
                break;
            }
        }
        // Now request the next value and wait for one more RTC event
        let mut count2 = self.read_count()?;
        i = 0;
        while count2 == count1 {
            // Add exit logic just in case a frequency is exactly synchronous to the events
            i += 1;
            if i > 1000 {
                break;
            }
            count2 = self.read_count()?;
        }
        // Possible that the counter has wrapped around, the following strategy works for
        // frequencies below (2^16)*(32768/<rtc_prescale>) Hz, currently expects rtc_prescale = 8
        if count2 < count1 {
            let diff: u32 = (count1 - count2).into();
            Ok(4096 * (65536 - diff))
        } else {
            let diff: u32 = (count2 - count1).into();
            Ok(4096 * diff)
        }
    }
}

// **** Measurement Collection Manager **** //
//
// BIGNOTE: When enabling the gated power supplies there will be a significant (~20ms) ramp up
// period to wait before conducting measurements
pub struct MeasMngr {
    adc: AdcMngr,
    counter: FreqCounter,
    board_sel: (bsp::Dsgp7, bsp::Dsgp6, bsp::Dsgp5),
    chip_sel: (bsp::Dlgp5, bsp::Dlgp4, bsp::Dlgp3),
    slope_trig: bsp::Dlgp6,
}

impl MeasMngr {
    pub fn new<'a>(
        adc: AdcMngr,
        counter: FreqCounter,
        board_sel: (bsp::Dsgp7, bsp::Dsgp6, bsp::Dsgp5),
        chip_sel: (bsp::Dlgp5, bsp::Dlgp4, bsp::Dlgp3),
        mut slope_trig: bsp::Dlgp6,
    ) -> MeasMngr {
        slope_trig.set_low().ok();
        MeasMngr {
            adc,
            counter,
            board_sel,
            chip_sel,
            slope_trig,
        }
    }

    pub fn measure(&mut self, param: &MeasConfigNew) -> Result<f32, &'static str> {
        // Extract the input channel and set of select lines based on what's being measured
        let (frq_ch, adc_ch, bs, cs): (
            Option<bsp::FreqMeasChnl>,
            Option<u8>,
            Option<(u8, u8, u8)>,
            Option<(u8, u8, u8)>,
        );
        match param {
            MeasConfigNew::TEMP(ch, sel) => {
                adc_ch = Some(*ch);
                frq_ch = None;
                let b = (sel & 0b00_100_000, sel & 0b00_010_000, sel & 0b00_001_000);
                bs = Some(b);
                cs = None;
            }
            MeasConfigNew::VSTRS(ch) => {
                adc_ch = Some(*ch);
                frq_ch = None;
                bs = None;
                cs = None;
            }
            MeasConfigNew::FREQ(ch, sel) => {
                adc_ch = None;
                let chnl = match ch {
                    0 => bsp::FreqMeasChnl::CH0,
                    1 => bsp::FreqMeasChnl::CH1,
                    2 => bsp::FreqMeasChnl::CH2,
                    _ => bsp::FreqMeasChnl::CH3,
                };
                frq_ch = Some(chnl);
                let b = (sel & 0b00_100_000, sel & 0b00_010_000, sel & 0b00_001_000);
                bs = Some(b);
                let c = (sel & 0b00_000_100, sel & 0b00_000_010, sel & 0b00_000_001);
                cs = Some(c);
            }
            MeasConfigNew::ANA(ch, sel) => {
                adc_ch = Some(*ch);
                frq_ch = None;
                let b = (sel & 0b00_100_000, sel & 0b00_010_000, sel & 0b00_001_000);
                bs = Some(b);
                let c = (sel & 0b00_000_100, sel & 0b00_000_010, sel & 0b00_000_001);
                cs = Some(c);
            }
            MeasConfigNew::DIG(ch, sel) => {
                if *ch < 4 {
                    let chnl = match ch {
                        0 => bsp::FreqMeasChnl::CH0,
                        1 => bsp::FreqMeasChnl::CH1,
                        2 => bsp::FreqMeasChnl::CH2,
                        _ => bsp::FreqMeasChnl::CH3,
                    };
                    frq_ch = Some(chnl);
                    adc_ch = None;
                } else {
                    adc_ch = Some(ch - 4);
                    frq_ch = None;
                }
                let b = (sel & 0b00_100_000, sel & 0b00_010_000, sel & 0b00_001_000);
                bs = Some(b);
                let c = (sel & 0b00_000_100, sel & 0b00_000_010, sel & 0b00_000_001);
                cs = Some(c);
            }
            MeasConfigNew::SLOPE(ch, sel) => {
                adc_ch = Some(*ch);
                frq_ch = None;
                let b = (sel & 0b00_100_000, sel & 0b00_010_000, sel & 0b00_001_000);
                bs = Some(b);
                let c = (sel & 0b00_000_100, sel & 0b00_000_010, sel & 0b00_000_001);
                cs = Some(c);
            }
            MeasConfigNew::SLOPENAIVE(ch, sel) => {
                adc_ch = Some(*ch);
                frq_ch = None;
                let b = (sel & 0b00_100_000, sel & 0b00_010_000, sel & 0b00_001_000);
                bs = Some(b);
                let c = (sel & 0b00_000_100, sel & 0b00_000_010, sel & 0b00_000_001);
                cs = Some(c);
            }
        }

        // Set the select pins to the correct values to expose the requested parameter
        if let Some((s2, s1, s0)) = bs {
            bsp::might_err(
                if s2 != 0 {
                    self.board_sel.0.set_high()
                } else {
                    self.board_sel.0.set_low()
                },
                "CRIT: Failed to set board select state",
            )?;
            bsp::might_err(
                if s1 != 0 {
                    self.board_sel.1.set_high()
                } else {
                    self.board_sel.1.set_low()
                },
                "CRIT: Failed to set board select state",
            )?;
            bsp::might_err(
                if s0 != 0 {
                    self.board_sel.2.set_high()
                } else {
                    self.board_sel.2.set_low()
                },
                "CRIT: Failed to set board select state",
            )?;
        }
        if let Some((s2, s1, s0)) = cs {
            bsp::might_err(
                if s2 != 0 {
                    self.chip_sel.0.set_high()
                } else {
                    self.chip_sel.0.set_low()
                },
                "CRIT: Failed to set chip select state",
            )?;
            bsp::might_err(
                if s1 != 0 {
                    self.chip_sel.1.set_high()
                } else {
                    self.chip_sel.1.set_low()
                },
                "CRIT: Failed to set chip select state",
            )?;
            bsp::might_err(
                if s0 != 0 {
                    self.chip_sel.2.set_high()
                } else {
                    self.chip_sel.2.set_low()
                },
                "CRIT: Failed to set chip select state",
            )?;
        }

        // Insert a delay after changing the select signals to allow analog voltages to stabilize,
        // current surges to pass, and clock signals to clean up, maximizing the chances of
        // obtaining accurate measurements. Delay is around 20ms.
        bsp::hard_delay(1_000_000);

        let measured: f32;
        if matches!(param, MeasConfigNew::TEMP(..)) {
            let actual_ch = bsp::pin_to_adc_chnl_map(adc_ch.unwrap());
            let adc_voltage = self.adc.meas_val(actual_ch)?.into();
            // Now convert the ADC voltage into temperature
            measured = MeasMngr::tmp235_conversion(adc_voltage);
        } else if matches!(param, MeasConfigNew::VSTRS(..)) {
            let actual_ch = bsp::vs_to_adc_chnl_map(adc_ch.unwrap());
            measured = self.adc.meas_val(actual_ch)?.into();
        } else if matches!(param, MeasConfigNew::ANA(..)) {
            let actual_ch = bsp::pin_to_adc_chnl_map(adc_ch.unwrap());
            measured = self.adc.meas_val(actual_ch)?.into();
        } else if matches!(param, MeasConfigNew::DIG(..)) {
            if let Some(ch) = adc_ch {
                let ana_val = self.adc.meas_val(adc_ch.unwrap())?.into();
                // Since we're trying to get a digital value, snap to 0 or 1 if close
                measured = if ana_val > 0.7 {
                    1.0
                } else if ana_val < 0.3 {
                    0.0
                } else {
                    ana_val
                };
            } else {
                let is_high = self.counter.read_pin_as_dig_in(frq_ch.unwrap());
                if is_high {
                    measured = 1.0;
                } else {
                    measured = 0.0;
                }
            }
        } else if matches!(param, MeasConfigNew::SLOPE(..)) {
            let actual_ch = bsp::pin_to_adc_chnl_map(adc_ch.unwrap());
            self.slope_trig.set_high().ok();
            measured = self.adc.meas_slope(actual_ch)?;
            self.slope_trig.set_low().ok();
        } else if matches!(param, MeasConfigNew::SLOPENAIVE(..)) {
            let actual_ch = bsp::pin_to_adc_chnl_map(adc_ch.unwrap());
            self.slope_trig.set_high().ok();
            measured = self.adc.naive_meas_slope(actual_ch)?;
            self.slope_trig.set_low().ok();
        } else {
            // Call frequency counter measurement function
            measured = self.counter.get_freq(frq_ch.unwrap())? as f32;
        }
        Ok(measured)
    }

    // Conversion function for calculating the temperature in Celsius given a TMP235 sensor output voltage
    fn tmp235_conversion(v: f32) -> f32 {
        let v_2 = v;
        // The sensor output response is approximated as a 3-segment piecewise function
        let (v_offset, tc, t_infl): (f32, f32, f32);
        if v_2 < 1.5 {
            (v_offset, tc, t_infl) = (0.5, 0.01, 0.0);
        } else if v_2 < 1.7525 {
            (v_offset, tc, t_infl) = (1.5, 0.0101, 100.0);
        } else {
            (v_offset, tc, t_infl) = (1.7525, 0.0106, 125.0);
        }
        ((v_2 - v_offset) / tc) + t_infl
    }
}

// **** SPI Bus Transaction Manager **** //
pub struct SpiMngr {
    pub spi: sercom::spi::Spi<
        sercom::spi::Config<sercom::spi::Pads<sercom::Sercom1, bsp::Miso, bsp::Mosi, bsp::Sclk>>,
        sercom::spi::Duplex,
    >,
    cs_pins: (bsp::CsFlash, bsp::CsDac, bsp::CsPot),
}

impl SpiMngr {
    pub fn new(
        pm: &bsp::pac::PM,
        spi: bsp::BoardSpi,
        spi_pins: (bsp::MosiUnset, bsp::MisoUnset, bsp::SclkUnset),
        cs_config: (
            impl Into<bsp::CsFlash>,
            impl Into<bsp::CsDac>,
            impl Into<bsp::CsPot>,
        ),
    ) -> SpiMngr {
        let pads = sercom::spi::Pads::<sercom::Sercom1>::default()
            .sclk(spi_pins.2)
            .data_in(spi_pins.1)
            .data_out(spi_pins.0);

        let spi = sercom::spi::Config::new(&pm, spi, pads, 12.MHz())
            .baud(2.MHz())
            .char_size::<sercom::spi::EightBit>()
            .spi_mode(sercom::spi::MODE_0)
            .enable();

        // Ensure no SPI device is selected to start
        let mut cs_pins = (cs_config.0.into(), cs_config.1.into(), cs_config.2.into());
        cs_pins.0.set_high().unwrap();
        cs_pins.1.set_high().unwrap();
        cs_pins.2.set_high().unwrap();

        let mut new_mngr = SpiMngr { spi, cs_pins };
        // Set the DAC voltage reference to be the internal bandgap reference for all channels
        new_mngr
            .set_dac_val(0x08, 0b01_01_01_01_01_01_01_01)
            .unwrap();
        new_mngr
    }

    fn transaction<'a, 'b>(
        &'a mut self,
        trgt: bsp::SpiDevs,
        bytes: &'b mut [u8],
    ) -> Result<&'b mut [u8], &'static str> {
        bsp::might_err(
            match trgt {
                bsp::SpiDevs::FLASH => self.cs_pins.0.set_low(),
                bsp::SpiDevs::DAC => self.cs_pins.1.set_low(),
                bsp::SpiDevs::POT => self.cs_pins.2.set_low(),
            },
            CS_SET_ERROR_MSG,
        )?;
        bsp::might_err(self.spi.transfer(bytes), SPI_TRANSFER_ERROR_MSG)?;
        bsp::might_err(
            match trgt {
                bsp::SpiDevs::FLASH => self.cs_pins.0.set_high(),
                bsp::SpiDevs::DAC => self.cs_pins.1.set_high(),
                bsp::SpiDevs::POT => self.cs_pins.2.set_high(),
            },
            CS_SET_ERROR_MSG,
        )?;
        // Wait about one SPI bus clock cycle to ensure any subsequent transactions don't
        // toggle CS pins too quickly
        bsp::hard_delay(10_000);
        Ok(bytes)
    }

    // *** MCP48FXBX8 Octal DAC Communication Traits ***
    pub fn set_ctrl_voltage(&mut self, pin: u8, voltage: f32) -> Result<(), &'static str> {
        let dac_code = SpiMngr::volts_to_dac_code(voltage);
        let chnl = bsp::pin_to_dac_chnl_map(pin);
        self.set_dac_val(chnl, dac_code)?;
        Ok(())
    }

    fn volts_to_dac_code(v: f32) -> u16 {
        // Assumes we're using the MCP48FXBX8 internal bandgap as reference
        let v_max: f32 = 2.44;
        // MCP48FEB28T is a 12-bit DAC
        let code_max: u16 = 0x0FFF;

        // Determine the correct output code, range of possible values is bounded
        if v >= v_max {
            code_max
        } else if v <= 0.0 {
            0
        } else {
            ((v / v_max) * (code_max as f32)) as u16
        }
    }

    pub fn read_dac_val(&mut self, channel: u8) -> Result<u16, &'static str> {
        // First 5 bits specify the address to read, next two specify the read operation, then a
        // reserved bit is ignored
        let cmd_byte = (channel << 3) + 0b00000_11_0;
        // Next two bytes will hold the DAC value (10 bits with current DAC, most significant 6
        // bits can be ignored)
        let mut command: [u8; 3] = [cmd_byte, 0x00, 0x00];

        // Set the chip select then execute the read transfer
        self.transaction(bsp::SpiDevs::DAC, &mut command)?;
        Ok((command[1] as u16 * 256) + command[2] as u16)
    }

    pub fn set_dac_val(&mut self, channel: u8, new_val: u16) -> Result<(), &'static str> {
        let cmd_byte = channel << 3; // 0bxxxxx_00_0
        let val_bytes = new_val.to_be_bytes();
        let mut command: [u8; 3] = [cmd_byte, val_bytes[0], val_bytes[1]];

        self.transaction(bsp::SpiDevs::DAC, &mut command)?;
        if command[1] != 0xff || command[2] != 0xff {
            Err(SPI_TRANSFER_ERROR_MSG)
        } else {
            Ok(())
        }
    }

    // *** Digital Potentiometer Communication Traits ***

    pub fn set_pot_code(&mut self, v_i: usize, code: u8) -> Result<(), &'static str> {
        let cmd_byte = match v_i {
            0 => 0b0000_00_00,
            1 => 0b0001_00_00,
            2 => 0b0110_00_00,
            _ => 0b0111_00_00,
        };
        let mut spi_bytes: [u8; 2] = [cmd_byte, code];
        self.transaction(bsp::SpiDevs::POT, &mut spi_bytes)?;
        Ok(())
    }

    pub fn write_mem_page(&mut self, page: u16, data: &[u8; 528]) -> Result<(), &'static str> {
        let mut cmd = [0; 4];
        // Opcode for program main flash memory through buffer with built-in erase
        cmd[0] = 0x82;
        // There are three address bytes for this opcode with a 528 page size:
        // 1 dummy bit, 13 page address bits, then 10 start byte address bits
        // We always start from byte zero so just need to shift the 16 bit page argument over by
        // two to format the address bytes correctly
        cmd[1..3].copy_from_slice(&(page << 2).to_be_bytes());
        // Wait for the flash to finish any previous operations before writing
        self.wait_until_ready()?;
        // We don't use the transaction method as we do two separate transfers here
        bsp::might_err(self.cs_pins.0.set_low(), CS_SET_ERROR_MSG)?;
        bsp::might_err(self.spi.write(&cmd), SPI_TRANSFER_ERROR_MSG)?;
        bsp::might_err(self.spi.write(data), SPI_TRANSFER_ERROR_MSG)?;
        bsp::might_err(self.cs_pins.0.set_high(), CS_SET_ERROR_MSG)?;
        bsp::hard_delay(10_000);
        Ok(())
    }

    pub fn read_mem_page(&mut self, page: u16, buf: &mut [u8; 528]) -> Result<(), &'static str> {
        let mut cmd: [u8; 6] = [0; 6];
        // Opcode for main flash memory continuous read at high frequency
        cmd[0] = 0x1b;
        // There are three address bytes for this opcode with a 528 page size:
        // 1 dummy bit, 13 page address bits, then 10 start byte address bits
        cmd[1..3].copy_from_slice(&(page << 2).to_be_bytes());

        self.wait_until_ready()?;
        // We don't use the transaction method as we do two separate transfers here
        bsp::might_err(self.cs_pins.0.set_low(), CS_SET_ERROR_MSG)?;
        bsp::might_err(self.spi.write(&cmd), SPI_TRANSFER_ERROR_MSG)?;
        bsp::might_err(self.spi.transfer(buf), SPI_TRANSFER_ERROR_MSG)?;
        bsp::might_err(self.cs_pins.0.set_high(), CS_SET_ERROR_MSG)?;
        bsp::hard_delay(10_000);
        Ok(())
    }

    pub fn wait_until_ready(&mut self) -> Result<(), &'static str> {
        // Clock in the command to read the status register, the flash memory will keep clocking
        // the register value out on a loop until we release CS
        let cmd = [0xd7];
        let mut status = [0b0000_0000];
        // Need to avoid toggling the CS pin too quickly between consecutive commands
        bsp::might_err(self.cs_pins.0.set_low(), CS_SET_ERROR_MSG)?;
        bsp::might_err(self.spi.write(&cmd), SPI_TRANSFER_ERROR_MSG)?;
        while status[0] & 0b1000_0000 == 0 {
            bsp::might_err(self.spi.transfer(&mut status), SPI_TRANSFER_ERROR_MSG)?;
        }
        bsp::might_err(self.cs_pins.0.set_high(), CS_SET_ERROR_MSG)?;
        bsp::hard_delay(10_000);
        Ok(())
    }
}

// **** Managers for Indicator Outputs **** //
const HOT_IND_ON_TEMP: f32 = 50.0;
const HOT_IND_OFF_TEMP: f32 = 45.0;
const HOT_IND_WARN_MSG: &str = "WARN: Failed to turn on hot indicator";

pub struct HotIndMngr {
    hot_ind_pin: bsp::Dsgp3,
}

impl HotIndMngr {
    pub fn new(hot_ind_pin: impl Into<bsp::Dsgp3>) -> HotIndMngr {
        HotIndMngr {
            hot_ind_pin: hot_ind_pin.into(),
        }
    }

    pub fn indicate_if_hot(&mut self, temp: f32) -> Result<(), &'static str> {
        let is_on = self.hot_ind_pin.is_set_high().unwrap();
        if temp > HOT_IND_ON_TEMP && !is_on {
            bsp::might_err(self.hot_ind_pin.set_high(), HOT_IND_WARN_MSG)?;
        } else if temp < HOT_IND_OFF_TEMP && is_on {
            bsp::might_err(self.hot_ind_pin.set_low(), HOT_IND_WARN_MSG)?;
        }
        Ok(())
    }
}

const LED_WARN_MSG: &str = "WARN: LED ctrl issue";

#[repr(u8)]
#[derive(Copy, Clone, PartialEq)]
pub enum LedState {
    ON,
    OFF,
    BLINK,
}

pub struct LedMngr {
    led0: bsp::Led0,
    led1: bsp::Led1,
    led2: bsp::Led2,
    led3: bsp::Led3,
    led_states: [LedState; 4],
}

impl LedMngr {
    pub fn new(
        color_led_0: impl Into<bsp::Led0>,
        color_led_1: impl Into<bsp::Led1>,
        color_led_2: impl Into<bsp::Led2>,
        color_led_3: impl Into<bsp::Led3>,
    ) -> LedMngr {
        LedMngr {
            led0: color_led_0.into(),
            led1: color_led_1.into(),
            led2: color_led_2.into(),
            led3: color_led_3.into(),
            led_states: [LedState::OFF; 4],
        }
    }

    pub fn handle_blinks(&mut self) {
        for i in 0..4 {
            if self.led_states[i] == LedState::BLINK {
                let trgt = match i {
                    0 => bsp::BoardLed::GREEN,
                    1 => bsp::BoardLed::YELLOW,
                    2 => bsp::BoardLed::ORANGE,
                    _ => bsp::BoardLed::RED,
                };
                self.toggle(trgt).ok();
            }
        }
    }

    pub fn set_state(&mut self, trgt: bsp::BoardLed, state: LedState) {
        let i = match trgt {
            bsp::BoardLed::GREEN => 0,
            bsp::BoardLed::YELLOW => 1,
            bsp::BoardLed::ORANGE => 2,
            bsp::BoardLed::RED => 3,
        };
        self.led_states[i] = state;
        // Blinking state handled separately
        if state == LedState::OFF {
            self.turn_off(trgt).ok();
        } else if state == LedState::ON {
            self.turn_on(trgt).ok();
        }
    }

    pub fn set_states_from_byte(&mut self, byte: u8) {
        for i in 0..4 {
            let trgt = bsp::led_color_map(i);
            // Solid on state gets priority over blink state as it feels intuitive
            if byte & (0b0000_0001 << i) != 0 {
                self.set_state(trgt, LedState::ON);
            } else if byte & (0b0001_0000 << i) != 0 {
                self.set_state(trgt, LedState::BLINK);
            } else {
                self.set_state(trgt, LedState::OFF);
            }
        }
    }

    fn led_map(&mut self, led: bsp::BoardLed) -> bsp::Leds {
        match led {
            bsp::BoardLed::RED => bsp::Leds::LED0(&mut self.led0),
            bsp::BoardLed::GREEN => bsp::Leds::LED1(&mut self.led1),
            bsp::BoardLed::YELLOW => bsp::Leds::LED2(&mut self.led2),
            bsp::BoardLed::ORANGE => bsp::Leds::LED3(&mut self.led3),
        }
    }

    fn toggle(&mut self, led: bsp::BoardLed) -> Result<(), &'static str> {
        let mut led = self.led_map(led);
        if bsp::might_err(led.is_set_high(), LED_WARN_MSG)? {
            bsp::might_err(led.set_low(), LED_WARN_MSG)?;
        } else {
            bsp::might_err(led.set_high(), LED_WARN_MSG)?;
        }
        Ok(())
    }

    fn turn_on(&mut self, led: bsp::BoardLed) -> Result<(), &'static str> {
        let led = &mut self.led_map(led);
        bsp::might_err(led.set_high(), LED_WARN_MSG)?;
        Ok(())
    }

    fn turn_off(&mut self, led: bsp::BoardLed) -> Result<(), &'static str> {
        let led = &mut self.led_map(led);
        bsp::might_err(led.set_low(), LED_WARN_MSG)?;
        Ok(())
    }
}

// **** Wrapper to abstract the stress voltage enable pins **** //
pub struct VstrsEnMngr {
    vsl0_en: bsp::Vsl0En,
    vsl1_en: bsp::Vsl1En,
    vsh0_en: bsp::Vsh0En,
    vsh1_en: bsp::Vsh1En,
}

impl VstrsEnMngr {
    pub fn new(
        en_0: impl Into<bsp::Vsl0En>,
        en_1: impl Into<bsp::Vsl1En>,
        en_2: impl Into<bsp::Vsh0En>,
        en_3: impl Into<bsp::Vsh1En>,
    ) -> VstrsEnMngr {
        VstrsEnMngr {
            vsl0_en: en_0.into(),
            vsl1_en: en_1.into(),
            vsh0_en: en_2.into(),
            vsh1_en: en_3.into(),
        }
    }

    fn en_map(&mut self, ind: usize) -> bsp::VsEn {
        match ind {
            0 => bsp::VsEn::VL0(&mut self.vsl0_en),
            1 => bsp::VsEn::VH0(&mut self.vsh0_en),
            2 => bsp::VsEn::VL1(&mut self.vsl1_en),
            _ => bsp::VsEn::VH1(&mut self.vsh1_en),
        }
    }

    pub fn disable(&mut self, ind: usize) -> Result<(), &'static str> {
        let mut pin = self.en_map(ind);
        bsp::might_err(pin.set_low(), "WARN: Vstrs en ctrl issue")?;
        Ok(())
    }

    pub fn enable(&mut self, ind: usize) -> Result<(), &'static str> {
        let mut pin = self.en_map(ind);
        bsp::might_err(pin.set_high(), "WARN: Vstrs en ctrl issue")?;
        Ok(())
    }

    pub fn is_disabled(&mut self, ind: usize) -> Result<bool, &'static str> {
        let mut pin = self.en_map(ind);
        bsp::might_err(pin.is_set_low(), "WARN: Vstrs en ctrl issue")
    }

    pub fn is_enabled(&mut self, ind: usize) -> Result<bool, &'static str> {
        let mut pin = self.en_map(ind);
        bsp::might_err(pin.is_set_high(), "WARN: Vstrs en ctrl issue")
    }
}

// **** Digital Control for DUT Configuration Manager **** //

const GPIO_FAIL_MSG: &str = "CRIT: Failed GPIO state set";

pub struct DutCtrlMngr {
    std_gpio: (bsp::Dsgp0, bsp::Dsgp1, bsp::Dsgp4),
    //lv_gpio: (bsp::Dlgp0, bsp::Dlgp1, bsp::Dlgp2, bsp::Dlgp6, bsp::Dlgp7),
    lv_gpio: (bsp::Dlgp0, bsp::Dlgp1, bsp::Dlgp2, bsp::Dlgp7),
}

impl DutCtrlMngr {
    pub fn new(
        std_pins: (
            impl Into<bsp::Dsgp0>,
            impl Into<bsp::Dsgp1>,
            impl Into<bsp::Dsgp4>,
        ),
        lv_pins: (
            impl Into<bsp::Dlgp0>,
            impl Into<bsp::Dlgp1>,
            impl Into<bsp::Dlgp2>,
            //impl Into<bsp::Dlgp6>,
            impl Into<bsp::Dlgp7>,
        ),
    ) -> DutCtrlMngr {
        DutCtrlMngr {
            std_gpio: (std_pins.0.into(), std_pins.1.into(), std_pins.2.into()),
            lv_gpio: (
                lv_pins.0.into(),
                lv_pins.1.into(),
                lv_pins.2.into(),
                lv_pins.3.into(),
                //lv_pins.4.into(),
            ),
        }
    }

    pub fn set_dig_ctrl(&mut self, signals: bsp::DigCtrlSetting) -> Result<(), &'static str> {
        let mask = signals.gpio;
        bsp::might_err(
            if (mask & 0b000_0_00_01) != 0 {
                self.std_gpio.0.set_high()
            } else {
                self.std_gpio.0.set_low()
            },
            GPIO_FAIL_MSG,
        )?;
        bsp::might_err(
            if (mask & 0b000_0_00_10) != 0 {
                self.std_gpio.1.set_high()
            } else {
                self.std_gpio.1.set_low()
            },
            GPIO_FAIL_MSG,
        )?;
        bsp::might_err(
            if (mask & 0b000_1_00_00) != 0 {
                self.std_gpio.2.set_high()
            } else {
                self.std_gpio.2.set_low()
            },
            GPIO_FAIL_MSG,
        )?;

        let mask = signals.lv_gpio;
        bsp::might_err(
            if (mask & 0b00_000_001) != 0 {
                self.lv_gpio.0.set_high()
            } else {
                self.lv_gpio.0.set_low()
            },
            GPIO_FAIL_MSG,
        )?;
        bsp::might_err(
            if (mask & 0b00_000_010) != 0 {
                self.lv_gpio.1.set_high()
            } else {
                self.lv_gpio.1.set_low()
            },
            GPIO_FAIL_MSG,
        )?;
        bsp::might_err(
            if (mask & 0b00_000_100) != 0 {
                self.lv_gpio.2.set_high()
            } else {
                self.lv_gpio.2.set_low()
            },
            GPIO_FAIL_MSG,
        )?;
        //bsp::might_err(
        //    if (mask & 0b01_000_000) != 0 {
        //        self.lv_gpio.3.set_high()
        //    } else {
        //        self.lv_gpio.3.set_low()
        //    },
        //    GPIO_FAIL_MSG,
        //)?;
        bsp::might_err(
            if (mask & 0b10_000_000) != 0 {
                self.lv_gpio.3.set_high()
            } else {
                self.lv_gpio.3.set_low()
            },
            GPIO_FAIL_MSG,
        )?;

        Ok(())
    }

    pub fn set_ana_ctrl(
        &self,
        spi: &mut SpiMngr,
        voltages: bsp::AnaCtrlSetting,
    ) -> Result<(), &'static str> {
        for (i, v) in voltages.ana_vals.iter().enumerate() {
            spi.set_ctrl_voltage(i as u8, *v)?;
        }
        Ok(())
    }

    pub fn refresh_ring_oscs(&mut self) -> Result<(), &'static str> {
        // Briefly stop ring oscillators from running then smooth restart to ensure clean oscillations
        bsp::might_err(self.lv_gpio.1.set_low(), GPIO_FAIL_MSG)?;
        bsp::hard_delay(4_000);
        bsp::might_err(self.lv_gpio.1.set_high(), GPIO_FAIL_MSG)?;
        Ok(())
    }
}