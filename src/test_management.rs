use crate::bsp::{AnaCtrlSetting, DigCtrlSetting};
use crate::dutsp::{MeasConfigNew, ANA_CTRL_DEFAULTS, DIG_CTRL_DEFAULTS};
use crate::periph_managers::{DutCtrlMngr, LedMngr, MeasMngr, SpiMngr};

#[derive(Copy, Clone)]
pub enum StepType {
    STRS,
    MEAS,
}

#[derive(Copy, Clone)]
pub struct TestStepNew {
    pub step_type: StepType,
    pub temp: f32,
    pub vs: [f32; 4],
    pub strs_en_mask: u8,
    pub meas_list: Option<u8>,
    pub duration: Option<u32>,
    pub ana_ctrl_vals: u8,
    pub dig_ctrl_vals: u8,
}

impl TestStepNew {
    pub const fn new() -> TestStepNew {
        TestStepNew {
            step_type: StepType::STRS,
            temp: 25.0,
            vs: [0.0; 4],
            strs_en_mask: 0b000_0_0000,
            meas_list: None,
            duration: None,
            ana_ctrl_vals: 0,
            dig_ctrl_vals: 0,
        }
    }

    pub fn strs_from_bytes(&bytes: &[u8; 64]) -> TestStepNew {
        let v0 = f32::from_be_bytes(bytes[14..18].try_into().unwrap());
        let v1 = f32::from_be_bytes(bytes[18..22].try_into().unwrap());
        let v2 = f32::from_be_bytes(bytes[22..26].try_into().unwrap());
        let v3 = f32::from_be_bytes(bytes[26..30].try_into().unwrap());

        TestStepNew {
            step_type: StepType::STRS,
            meas_list: None,
            dig_ctrl_vals: bytes[3],
            ana_ctrl_vals: bytes[4],
            duration: Some(u32::from_be_bytes(bytes[5..9].try_into().unwrap())),
            strs_en_mask: bytes[9],
            temp: f32::from_be_bytes(bytes[10..14].try_into().unwrap()),
            vs: [v0, v1, v2, v3],
        }
    }

    pub fn meas_from_bytes(&bytes: &[u8; 64]) -> TestStepNew {
        let v0 = f32::from_be_bytes(bytes[14..18].try_into().unwrap());
        let v1 = f32::from_be_bytes(bytes[18..22].try_into().unwrap());
        let v2 = f32::from_be_bytes(bytes[22..26].try_into().unwrap());
        let v3 = f32::from_be_bytes(bytes[26..30].try_into().unwrap());

        TestStepNew {
            step_type: StepType::MEAS,
            duration: None,
            dig_ctrl_vals: bytes[3],
            ana_ctrl_vals: bytes[4],
            meas_list: Some(bytes[8]),
            strs_en_mask: bytes[9],
            temp: f32::from_be_bytes(bytes[10..14].try_into().unwrap()),
            vs: [v0, v1, v2, v3],
        }
    }
}

pub struct TestMngr<'a> {
    tc: &'a TestChest,
    pub curr_step: Option<&'a TestStepNew>,
    pub step_requires_stress_change: bool,
    next_step_ind: usize,
    pub last_strs_rprt_tick: u32,
    pub last_strs_sbmt_tick: u32,
    curr_strs_start_tick: u32,
    what_to_meas: usize,
    next_meas_ind: usize,
}

impl<'a> TestMngr<'a> {
    pub fn new(tc: &TestChest) -> TestMngr {
        TestMngr {
            tc,
            curr_step: None,
            step_requires_stress_change: true,
            next_step_ind: 0,
            last_strs_rprt_tick: 0,
            last_strs_sbmt_tick: 0,
            curr_strs_start_tick: 0,
            what_to_meas: 0,
            next_meas_ind: 0,
        }
    }

    pub fn set_ana_ctrl(
        &self,
        test_chest: &TestChest,
        ctrlr: &mut DutCtrlMngr,
        spi: &mut SpiMngr,
    ) -> Result<(), &'static str> {
        if let Some(step) = self.curr_step {
            // The test step can specify the ctrl signals to be applied or let defaults be used
            if step.ana_ctrl_vals >= 10 {
                ctrlr.set_ana_ctrl(spi, ANA_CTRL_DEFAULTS)?;
            } else {
                ctrlr.set_ana_ctrl(spi, test_chest.ana_out_configs[step.ana_ctrl_vals as usize])?;
            }
        } else {
            ctrlr.set_ana_ctrl(spi, ANA_CTRL_DEFAULTS)?;
        }
        Ok(())
    }

    pub fn set_dig_ctrl(
        &self,
        test_chest: &TestChest,
        dig_ctrlr: &mut DutCtrlMngr,
        led_ctrlr: &mut LedMngr,
    ) -> Result<(), &'static str> {
        if let Some(step) = self.curr_step {
            if step.dig_ctrl_vals >= 10 {
                dig_ctrlr.set_dig_ctrl(DIG_CTRL_DEFAULTS)?;
                led_ctrlr.set_states_from_byte(0);
            } else {
                dig_ctrlr.set_dig_ctrl(test_chest.dig_out_configs[step.dig_ctrl_vals as usize])?;
                led_ctrlr.set_states_from_byte(
                    test_chest.dig_out_configs[step.dig_ctrl_vals as usize].led_enable,
                );
            }
        } else {
            dig_ctrlr.set_dig_ctrl(DIG_CTRL_DEFAULTS)?;
            led_ctrlr.set_states_from_byte(0);
        }
        Ok(())
    }

    pub fn next_step(&mut self) {
        if self.next_step_ind >= self.tc.test_len {
            // End of the test is reached
            self.reset_test();
        } else {
            // The test ordering gives us the index of the next test step definition to execute
            let step_id = self.tc.test_order[self.next_step_ind] as usize;
            self.curr_step = Some(&self.tc.test_steps[step_id]);
            self.next_step_ind += 1;
        }
    }

    pub fn reset_test(&mut self) {
        self.curr_step = None;
        self.next_step_ind = 0;
    }

    /// Tracking for adding a stress condition measurement set every 60 seconds
    pub fn time_to_rprt_strs(&self, tick: u32) -> bool {
        tick >= self.last_strs_rprt_tick + (8 * 60)
    }

    /// Tracking for sending stress condition measurements to the data manager
    pub fn time_to_sbmt_strs(&self, tick: u32) -> bool {
        tick >= self.last_strs_sbmt_tick + (8 * 60 * 60)
    }

    pub fn strs_step_start(&mut self, tick: u32) {
        self.curr_strs_start_tick = tick;
        // Report stress at the midpoint of each minute instead of the end, so subtract 30 seconds
        self.last_strs_rprt_tick = tick - (8 * 30);
        self.last_strs_sbmt_tick = tick - (8 * 30);
    }

    pub fn strs_complete(&self, tick: u32) -> bool {
        if let Some(step) = self.curr_step.as_ref() {
            // If the elapsed real-time timer has not yet surpassed the sum of previous step
            // durations + the length of stress for the current step then stress not complete
            if let Some(duration) = step.duration {
                // Durations are in seconds, ticks occur 8 times per second
                if tick < self.curr_strs_start_tick + (duration * 8) {
                    return false;
                }
            }
        }
        // If no current step or measure step there's no stress to complete so return true
        true
    }

    pub fn meas_step_start(&mut self) {
        if let Some(step) = self.curr_step.as_ref() {
            if let Some(meas_list) = step.meas_list {
                self.what_to_meas = meas_list as usize;
                self.next_meas_ind = 0;
            } else {
                self.next_meas_ind = 100;
            }
        } else {
            self.next_meas_ind = 100;
        }
    }

    pub fn more_to_measure(&self) -> bool {
        self.next_meas_ind < self.tc.meas_lens[self.what_to_meas]
    }

    pub fn next_measurement(&mut self, measurer: &mut MeasMngr) -> Result<f32, &'static str> {
        let next_to_meas = self.tc.meas_orders[self.what_to_meas][self.next_meas_ind];
        self.next_meas_ind += 1;
        measurer.measure(&self.tc.meas_configs[next_to_meas as usize])
    }
}

// The Test Chest is essentially a box for holding the test configuration data and ensuring it is
// accessed and loaded in a clean and reliable fashion
// In total, the SAMD21J18A has 32kB of RAM, this struct uses a significant portion of this space
// to store the test configurations
pub struct TestChest {
    // With up to 255 unique measurements, this requires (1 type byte, 1 channel byte, 2 sel bytes) 1kB
    meas_configs: [MeasConfigNew; 255],
    // With up to 5 lists of measurements, this requires (250 bytes max list length) 1.25kB
    meas_orders: [[u8; 250]; 5],
    meas_lens: [usize; 5],
    // With up to 30 unique steps, this requires (5 conds x 4 byte floats, 1 byte step type, 4 byte
    // duration / meas_list ID, 1 bytes dig ctrl setting, 1 byte ana ctrl setting) ~1kB
    test_steps: [TestStepNew; 30],
    // With up to 4000 steps, this requires 4kB
    test_order: [u8; 4000],
    test_len: usize,
    // Up to 10 unique analog output settings, requires (4 byte float x 8 outputs) 0.3kB
    ana_out_configs: [AnaCtrlSetting; 10],
    // Up to 10 unique digital output settings, requires (4 bytes) ~0kB
    dig_out_configs: [DigCtrlSetting; 10],
    // Test availability information
    pub awaiting_test_data: bool,
    pub test_available_to_run: bool,
}

impl TestChest {
    pub const fn new() -> TestChest {
        let blank_step = TestStepNew::new();
        let blank_config = MeasConfigNew::VSTRS(0);
        let nostrs_anaconfig = ANA_CTRL_DEFAULTS;
        let nostrs_digconfig = DIG_CTRL_DEFAULTS;

        TestChest {
            ana_out_configs: [nostrs_anaconfig; 10],
            dig_out_configs: [nostrs_digconfig; 10],

            meas_configs: [blank_config; 255],
            meas_orders: [[0; 250]; 5],
            meas_lens: [0; 5],

            test_steps: [blank_step; 30],
            test_order: [0; 4000],
            test_len: 0,

            awaiting_test_data: false,
            // TODO: Potentially leave a small test as a default so you don't always have to load a
            // test via USB to do something with the board
            test_available_to_run: false,
        }
    }

    pub fn clear(&mut self) {
        self.test_available_to_run = false;
        self.test_len = 0;
        self.meas_lens = [0; 5];
        self.awaiting_test_data = true;
    }

    pub fn input_next(&mut self, &bytes: &[u8; 64]) -> Option<&'static str> {
        match bytes[0] {
            0x10 => self.add_meas_config(&bytes),
            0x11 => self.add_meas_order(&bytes),

            0x20 => self.add_ana_out_setting(&bytes),
            0x30 => self.add_dig_out_setting(&bytes),

            0x40 => self.add_test_step(&bytes),
            0x41 => self.add_to_test_order(&bytes),

            0xff => self.end_load_phase(),

            // Default behaviour is just to ignore, not error out
            _ => Some("WARNING: Invalid input code, ignoring..."),
        }
    }

    fn add_ana_out_setting(&mut self, &bytes: &[u8; 64]) -> Option<&'static str> {
        let id = bytes[1] as usize;
        if id >= 10 {
            return Some("WARNING: Ana ctrl ID too large, ignoring...");
        }
        let mut ana_vals: [f32; 8] = [0.0; 8];
        for i in 0..8 {
            let i1 = 2 + (i * 4);
            let i2 = i1 + 4;
            ana_vals[i] = f32::from_be_bytes(bytes[i1..i2].try_into().unwrap());
        }
        self.ana_out_configs[id] = AnaCtrlSetting { ana_vals };
        None
    }

    fn add_dig_out_setting(&mut self, &bytes: &[u8; 64]) -> Option<&'static str> {
        let id = bytes[1] as usize;
        if id >= 10 {
            return Some("WARNING: Dig ctrl ID too large, ignoring...");
        }
        self.dig_out_configs[id] = DigCtrlSetting {
            gpio: bytes[2],
            lv_gpio: bytes[3],
            led_enable: bytes[4],
        };
        None
    }

    fn add_meas_config(&mut self, &bytes: &[u8; 64]) -> Option<&'static str> {
        let id = bytes[1] as usize;
        if id >= 255 {
            return Some("WARNING: Meas config ID too large, ignoring...");
        }
        self.meas_configs[id] = match bytes[2] {
            0x00 => MeasConfigNew::VSTRS(bytes[3]),
            0x01 => MeasConfigNew::TEMP(bytes[3], bytes[4]),
            0x02 => MeasConfigNew::DIG(bytes[3], bytes[4]),
            0x03 => MeasConfigNew::FREQ(bytes[3], bytes[4]),
            0x04 => MeasConfigNew::ANA(bytes[3], bytes[4]),
            0x05 => MeasConfigNew::SLOPE(bytes[3], bytes[4]),
            0x06 => MeasConfigNew::SLOPENAIVE(bytes[3], bytes[4]),
            _ => MeasConfigNew::VSTRS(bytes[3]),
        };
        None
    }

    fn add_meas_order(&mut self, &bytes: &[u8; 64]) -> Option<&'static str> {
        let id = bytes[1] as usize;
        let mut i: usize = 2;
        while i <= 63 && bytes[i] != 0xff {
            if self.meas_lens[id] < 250 {
                self.meas_orders[id][self.meas_lens[id]] = bytes[i];
                self.meas_lens[id] += 1;
            } else {
                return Some("WARNING: Max measure order length reached, ignoring...");
            }
            i += 1;
        }
        None
    }

    fn add_to_test_order(&mut self, &bytes: &[u8; 64]) -> Option<&'static str> {
        let mut i: usize = 1;
        while i <= 63 && bytes[i] != 0xff {
            if self.test_len < 4000 {
                self.test_order[self.test_len] = bytes[i];
                self.test_len += 1;
            } else {
                return Some("WARNING: Max test length reached, ignoring...");
            }
            i += 1;
        }
        None
    }

    fn add_test_step(&mut self, &bytes: &[u8; 64]) -> Option<&'static str> {
        let id = bytes[1] as usize;
        if id >= 30 {
            return Some("WARNING: Test step ID too large, ignoring...");
        }
        if bytes[2] == 0 {
            self.test_steps[id] = TestStepNew::strs_from_bytes(&bytes);
        } else {
            self.test_steps[id] = TestStepNew::meas_from_bytes(&bytes);
        }
        None
    }

    fn end_load_phase(&mut self) -> Option<&'static str> {
        self.awaiting_test_data = false;
        if self.test_len > 0 {
            self.test_available_to_run = true;
        }
        None
    }
}
