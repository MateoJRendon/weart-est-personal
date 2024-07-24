use crate::bsp::{AnaCtrlSetting, DigCtrlSetting};

#[derive(Copy, Clone)]
pub enum MeasConfigNew {
    VSTRS(u8),
    TEMP(u8, u8),
    DIG(u8, u8),
    FREQ(u8, u8),
    ANA(u8, u8),
    SLOPE(u8, u8),
    SLOPENAIVE(u8, u8),
}

// #[repr(u8)]
// // Represents the board (S2, S1, S0) select signals
// // TODO: Was not able to figure out how to put the repr(u8) as a tuple so set it as u8
// pub enum SxBrd {
//     Y0Brd = 0,
//     Y1Brd = 1,
//     Y2Brd = 2,
//     Y3Brd = 3,
//     Y4Brd = 4,
//     Y5Brd = 5,
//     Y6Brd = 6,
//     Y7Brd = 7,
// }
// #[repr(u8)]
// // Represents the chip (s2, s1, s0) select signals
// // TODO: Was not able to figure out how to put the repr(u8) as a tuple so set it as u8
// pub enum SxChp {
//     Y0Chp = 0,
//     Y1Chp = 1,
//     Y2Chp = 2,
//     Y3Chp = 3,
//     Y4Chp = 4,
//     Y5Chp = 5,
//     Y6Chp = 6,
//     Y7Chp = 7,
// }

pub struct StressMeasConfigs {
    pub tsensors: [MeasConfigNew; 2],
    pub vsensors: [MeasConfigNew; 4],
}

pub const STRESS_SENSORS: StressMeasConfigs = StressMeasConfigs {
    tsensors: [
        MeasConfigNew::TEMP(6, 0b00_111_000),
        MeasConfigNew::TEMP(6, 0b00_011_000),
    ],
    vsensors: [
        MeasConfigNew::VSTRS(0),
        MeasConfigNew::VSTRS(1),
        MeasConfigNew::VSTRS(12),
        MeasConfigNew::VSTRS(13),
    ],
};

pub const ANA_CTRL_IDFBCAMP: AnaCtrlSetting = AnaCtrlSetting {
    //         Iref,  Prt,   Pgv,   Nrt,   Ngv
    ana_vals: [0.350, 0.380, 0.400, 0.420, 0.400, 0.0, 0.0, 0.0],
};

pub const ANA_CTRL_DEFAULTS: AnaCtrlSetting = AnaCtrlSetting { ana_vals: [0.0; 8] };
pub const DIG_CTRL_DEFAULTS: DigCtrlSetting = DigCtrlSetting {
    gpio: 0b000_000_00,
    lv_gpio: 0b0000_0000,
    led_enable: 0b0000_0000,
};
