#[cfg(feature = "semihost")]
use {core::fmt::Write, cortex_m_semihosting::hio};

/// Wrapper for semihost debug message printing over SWD, if semihost feature is not active
/// this gets optimized away to nothing so imposes no runtime or space cost
pub fn swd_print_val(msg: &'static str, val: impl core::fmt::Debug) {
    #[cfg(feature = "semihost")]
    if let Ok(mut hstdout) = hio::hstdout() {
        writeln!(hstdout, "{} {:?}", msg, val).ok();
    }
}

pub fn swd_print(msg: &'static str) {
    #[cfg(feature = "semihost")]
    if let Ok(mut hstdout) = hio::hstdout() {
        writeln!(hstdout, "{}", msg).ok();
    }
}

#[derive(PartialEq)]
#[repr(u8)]
pub enum CtrlCmd {
    Start,
    Stop,
    Pause,
    Dump,
    LoadTest,
    NoCmd,
}

#[derive(PartialEq, Debug)]
#[repr(u8)]
pub enum ProgState {
    Idle,
    LoadTest,
    FlashDump,
    Stress,
    CondShift,
    Measure,
}

pub struct ProgManager {
    pub state: ProgState,
    pub entry: bool,
    pub strs_unstable_warning: bool,
    pub last_strs_meas_tick: u32,
    pub last_strs_adjust_tick: u32,
    pub last_debug_out_tick: u32,
}

impl ProgManager {
    pub fn new() -> ProgManager {
        ProgManager {
            state: ProgState::Idle,
            entry: true,
            strs_unstable_warning: false,
            last_strs_meas_tick: 0,
            last_strs_adjust_tick: 0,
            last_debug_out_tick: 0,
        }
    }

    pub fn change_state(&mut self, new_state: ProgState) {
        let mut is_invalid = true;
        match self.state {
            ProgState::Idle => {
                if matches!(
                    new_state,
                    ProgState::FlashDump | ProgState::CondShift | ProgState::LoadTest
                ) {
                    is_invalid = false;
                }
            }
            ProgState::LoadTest => {
                if matches!(new_state, ProgState::Idle) {
                    is_invalid = false;
                }
            }
            ProgState::FlashDump => {
                if matches!(new_state, ProgState::Idle) {
                    is_invalid = false;
                }
            }
            ProgState::Stress => {
                if matches!(new_state, ProgState::Idle | ProgState::CondShift) {
                    is_invalid = false;
                }
            }
            ProgState::CondShift => {
                if matches!(
                    new_state,
                    ProgState::Idle | ProgState::Stress | ProgState::Measure
                ) {
                    is_invalid = false;
                }
            }
            ProgState::Measure => {
                if matches!(new_state, ProgState::Idle | ProgState::CondShift) {
                    is_invalid = false;
                }
            }
        };
        if is_invalid {
            // FIXME: Can panic in the main program loop
            panic!("Invalid state transition attempted!")
        }
        swd_print_val("New state:", &new_state);
        self.state = new_state;
        self.entry = true;
    }
}
