pub use cortex_m_rt::entry;
use libm::sqrtf;

pub use atsamd_hal as hal;
use atsamd_hal::ehal::digital::OutputPin;
pub use hal::ehal;
pub use hal::pac;

hal::bsp_peripherals!(
    SERCOM0 { OptionalDlgp2345Bus }
    SERCOM1 { BoardSpi }
    SERCOM2 { OptionalDsgp0145Bus }
    SERCOM3 { OptionalDlgpXX07Bus }
    SERCOM4 { OptionalDlgpXX16Bus }
    TCC0 { PWM }
    // TCC1 { Pwm1 }
);

hal::bsp_pins!(
    // PA00 XIN32 - PA01 XOUT32.
    PA02 {
        // ADC voltage stress low 0 sensing, nominally up to 1.1V. Could be reconfigured as DAC output but unlikely. AIN[0] or VOUT.
        name: vsl0_sense
        aliases: {
            AlternateB: Vsl0Sense
            AlternateB: DacVOut
        }
    }
    PA03 {
        // ADC voltage stress high 0 sensing, divided by 2, nominally up to 3/2=1.5V. Could be reconfigured as DAC/ADC ref but unlikely. AIN[1] or VREFA.
        name: vsh0_sense
        aliases: {
            AlternateB: Vsh0Sense
            AlternateB: AnaVRef
        }
    }
    PA04 {
        // Sensitive analog input for ADC measurement. AIN[4].
        name: ain7
        aliases: {
            AlternateB: AnaIn7
        }
    }
    PA05 {
        // Sensitive analog input for ADC measurement. AIN[5]. Connected to AO3 of the host board.
        name: ain6
        aliases: {
            AlternateB: AnaIn6
        }
    }
    PA06 {
        // Sensitive analog input for ADC measurement. AIN[6]. Connected to AO2 of the host board.
        name: ain5
        aliases: {
            AlternateB: AnaIn5
        }
    }
    PA07 {
        // Sensitive analog input for ADC measurement. AIN[7]. Connected to AO1 of the host board.
        name: ain4
        aliases: {
            AlternateB: AnaIn4
        }
    }
    PA08 {
        // D3 in level-shifted 3.3V to 1.8V GPIO. Optional serial communications. GPIO or SERCOM0/PAD[0]. Connected to S0 of the host board.
        name: dvcc2lgp3
        aliases: {
            PushPullOutput: Dlgp3
            AlternateC: SerComDlgp3
        }
    }
    PA09 {
        // D4 in level-shifted 3.3V to 1.8V GPIO. Optional serial communications. GPIO or SERCOM0/PAD[1]. Connected to S1 of the host board.
        name: dvcc2lgp4
        aliases: {
            PushPullOutput: Dlgp4
            AlternateC: SerComDlgp4
        }
    }
    PA10 {
        // D2 in level-shifted 3.3V to 1.8V GPIO. Optional serial communications. GPIO or SERCOM0/PAD[2]. Connected to Meas of the host board.
        name: dvcc2lgp2
        aliases: {
            PushPullOutput: Dlgp2
            AlternateC: SerComDlgp2
        }
    }
    PA11 {
        // D5 in level-shifted 3.3V to 1.8V GPIO. Optional serial communications. GPIO or SERCOM0/PAD[3]. Connected to S2 of the host board.
        name: dvcc2lgp5
        aliases: {
            PushPullOutput: Dlgp5
            AlternateC: SerComDlgp5
        }
    }
    PA12 {
        // D5 in standard 3.3V GPIO. Optional serial communications. GPIO or SERCOM2/PAD[0]. Connected to S0VCC of the host board.
        name: dsgp5
        aliases: {
            PushPullOutput: Dsgp5
            AlternateC: SerComDsgp5
        }
    }
    PA13 {
        // D4 in standard 3.3V GPIO. Optional serial communications. GPIO or SERCOM2/PAD[1].
        name: dsgp4
        aliases: {
            PushPullOutput: Dsgp4
            AlternateC: SerComDsgp4
        }
    }
    PA14 {
        // D0 in standard 3.3V GPIO. Optional serial communications. GPIO or SERCOM2/PAD[2].
        name: dsgp0
        aliases: {
            PushPullOutput: Dsgp0
            AlternateC: SerComDsgp0
        }
    }
    PA15 {
        // D1 in standard 3.3V GPIO. Optional serial communications. GPIO or SERCOM2/PAD[3].
        name: dsgp1
        aliases: {
            PushPullOutput: Dsgp1
            AlternateC: SerComDsgp1
        }
    }
    PA16 {
        // Board control SPI bus for flash memory, octal DAC, and digital potentiometer. SERCOM1/PAD[0].
        name: mosi
        aliases: {
            AlternateC: Mosi
            FloatingDisabled: MosiUnset
        }
    }
    PA17 {
        // Board control SPI bus for flash memory, octal DAC, and digital potentiometer. SERCOM1/PAD[1]
        name: sclk
        aliases: {
            AlternateC: Sclk
            FloatingDisabled: SclkUnset
        }
    }
    PA18 {
        // Board control SPI bus for flash memory, octal DAC, and digital potentiometer. SERCOM1/PAD[2]
        name: miso
        aliases: {
            AlternateC: Miso
            FloatingDisabled: MisoUnset
        }
    }
    PA19 {
        // Board control SPI bus for flash memory, octal DAC, and digital potentiometer. SERCOM1/PAD[3]
        name: cs_flasmem
        aliases: {
            PushPullOutput: CsFlash
        }
    }
    PA20 {
        //  D1 in level-shifted 3.3V to 1.8V GPIO. Optional input clocks and serial communications. GPIO, SERCOM5/PAD[2] or GCLK_IO[4].
        // Connected to OscRST of the host board.
        name: dvcc2lgp1
        aliases: {
            PushPullOutput: Dlgp1
            AlternateC: SerComDlgp1
            AlternateH: GClkDlgp1
        }
    }
    PA21 {
        // D6 in level-shifted 3.3V to 1.8V GPIO. Optional input clocks and serial communications. GPIO, SERCOM5/PAD[3] or GCLK_IO[5].
        // Connected to Freq1En of the host board.
        name: dvcc2lgp6
        aliases: {
            PushPullOutput: Dlgp6
            AlternateC: SerComDlgp6
            AlternateH: GClkDlgp6
        }
    }
    PA22 {
        // D0 in level-shifted 3.3V to 1.8V GPIO. Optional input clocks and serial communications. GPIO, SERCOM3/PAD[0] or GCLK_IO[6].
        // Connected to SlReset of the host board.
        name: dvcc2lgp0
        aliases: {
            PushPullOutput: Dlgp0
            AlternateC: SerComDlgp0
            AlternateH: GClkDlgp0
        }
    }
    PA23 {
        // D7 in level-shifted 3.3V to 1.8V GPIO. Optional input clocks and serial communications. GPIO, SERCOM3/PAD[1] or GCLK_IO[7].
        // Connected to OscEN of the host board.
        name: dvcc2lgp7
        aliases: {
            PushPullOutput: Dlgp7
            AlternateC: SerComDlgp7
            AlternateH: GClkDlgp7
        }
    }
    PA24 {
        // USB D- line for communication with host computer.
        name: usb_dm
        aliases: {
            AlternateG: UsbDm
        }
    }
    PA25 {
        // USB D+ line for communication with host computer.
        name: usb_dp
        aliases: {
            AlternateG: UsbDp
        }
    }
    PA27 {
        // LED 3 driver. GPIO.
        name: led3
        aliases: {
            PushPullOutput: Led3
        }
    }
    PA28 {
        // LED 2 driver. GPIO.
        name: led2
        aliases: {
            PushPullOutput: Led2
        }
    }

    // PA30 SWCLK - PA31 SWDIO

    PB00 {
        // Enable for voltage stress high 1. GPIO.
        name: vsh1_en
        aliases: {
            PushPullOutput: Vsh1En
        }
    }
    PB01 {
        // Enable for voltage stress low 1. GPIO.
        name: vsl1_en
        aliases: {
            PushPullOutput: Vsl1En
        }
    }
    PB02 {
        // Enable for voltage stress high 0. GPIO.
        name: vsh0_en
        aliases: {
            PushPullOutput: Vsh0En
        }
    }
    PB03 {
        // Enable for voltage stress low 0. GPIO.
        name: vsl0_en
        aliases: {
            PushPullOutput: Vsl0En
        }
    }
    PB04 {
        //  ADC voltage stress low 1 sensing, nominally up to 1.1V. AIN[12].
        name: vsl1_sense
        aliases: {
            AlternateB: Vsl1Sense
        }
    }
    PB05 {
        //  ADC voltage stress high 1 sensing, divided by 2, nominally up to 3/2=1.5V. AIN[13].
        name: vsh1_sense
        aliases: {
            AlternateB: Vsh1Sense
        }
    }
    PB06 {
        // Analog input with 100pF capacitance for ADC measurement. AIN[14].
        name: ain0
        aliases: {
            AlternateB: AnaIn0
        }
    }
    PB07 {
        // Analog input with 100pF capacitance for ADC measurement. AIN[15].
        name: ain1
        aliases: {
            AlternateB: AnaIn1
        }
    }
    PB08 {
        // Analog input with 100pF capacitance for ADC measurement. AIN[2].
        name: ain2
        aliases: {
            AlternateB: AnaIn2
        }
    }
    PB09 {
        // Analog input with 100pF capacitance for ADC measurement. AIN[3].
        name: ain3
        aliases: {
            AlternateB: AnaIn3
        }
    }
    PB10 {
        // Input 'clock' for frequency measurement functionality. GCLK_IO[4]. Connected to MF4-HB of the host board.
        name: mf3
        aliases: {
            FloatingInput: MD3
            AlternateH: MF3
        }
    }
    PB11 {
        // Input 'clock' for frequency measurement functionality.  GCLK_IO[5]. Connected to MF3-HB of the host board.
        name: mf2
        aliases: {
            FloatingInput: MD2
            AlternateH: MF2
        }
    }
    PB12 {
        // Input 'clock' for frequency measurement functionality.  GCLK_IO[6]. Connected to MF1-HB of the host board.
        name: mf0
        aliases: {
            FloatingInput: MD0
            AlternateH: MF0
        }
    }
    PB13 {
        // Input 'clock' for frequency measurement functionality.  GCLK_IO[7]. Connected to MF2-HB of the host board.
        name: mf1
        aliases: {
            FloatingInput: MD1
            AlternateH: MF1
        }
    }
    PB14 {
        // D7 in standard 3.3V GPIO. Can output TC5/WO[0] frequencies or GCLK_IO[0]. GPIO. Connected to S2VCC of the host board.
        name: dsgp7
        aliases: {
            PushPullOutput: Dsgp7
            AlternateE: TcDsgp7
            AlternateH: GClkDsgp7
        }
    }
    PB15 {
        // D6 in standard 3.3V GPIO. Can output TC5/WO[1] frequencies or GCLK_IO[1]. GPIO. Connected to S1VCC of the host board.
        name: dsgp6
        aliases: {
            PushPullOutput: Dsgp6
            AlternateE: TcDsgp6
            AlternateH: GClkDsgp6
        }
    }
    PB16 {
        // Board control SPI bus for flash memory, octal DAC, and digital potentiometer. GPIO.
        name: cs_pot
        aliases: {
            PushPullOutput: CsPot
        }
    }
    PB17 {
        // Board control SPI bus for flash memory, octal DAC, and digital potentiometer. GPIO.
        name: cs_dac
        aliases: {
            PushPullOutput: CsDac
        }
    }
    PB22 {
        //  LED 0 driver. GPIO. Also connected to an output block.
        name: led0
        aliases: {
            PushPullOutput: Led0
        }
    }
    PB23 {
        //  LED 1 driver. GPIO. Also connected to an output block.
        name: led1
        aliases: {
            PushPullOutput: Led1
        }
    }
    PB30 {
        //  D2 in standard 3.3V GPIO. Intended for PWM signals. TCC0/WO[0].  Connected to HeatCtrl of the host board.
        name: dsgp2
        aliases: {
            PushPullOutput: Dsgp2
            AlternateE: TccDsgp2
        }
    }
    PB31 {
        //  D3 in standard 3.3V GPIO. Intended for PWM signals. TCC1/WO[3] or GPIO.  Connected to WarnLight of the host board.
        name: dsgp3
        aliases: {
            PushPullOutput: Dsgp3
            AlternateF: TccDsgp3
        }
    }
);

#[derive(Copy, Clone)]
pub enum FreqMeasChnl {
    CH0,
    CH1,
    CH2,
    CH3,
}

pub enum BoardLed {
    GREEN,
    YELLOW,
    ORANGE,
    RED,
}

pub fn led_color_map(i: usize) -> BoardLed {
    match i {
        0 => BoardLed::GREEN,
        1 => BoardLed::YELLOW,
        2 => BoardLed::ORANGE,
        _ => BoardLed::RED,
    }
}

pub enum Leds<'l> {
    LED0(&'l mut hal::gpio::pin::Pin<hal::gpio::pin::PB22, hal::gpio::pin::PushPullOutput>),
    LED1(&'l mut hal::gpio::pin::Pin<hal::gpio::pin::PB23, hal::gpio::pin::PushPullOutput>),
    LED2(&'l mut hal::gpio::pin::Pin<hal::gpio::pin::PA28, hal::gpio::pin::PushPullOutput>),
    LED3(&'l mut hal::gpio::pin::Pin<hal::gpio::pin::PA27, hal::gpio::pin::PushPullOutput>),
}

impl ehal::digital::StatefulOutputPin for Leds<'_> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        match self {
            Leds::LED0(l) => l.is_set_high(),
            Leds::LED1(l) => l.is_set_high(),
            Leds::LED2(l) => l.is_set_high(),
            Leds::LED3(l) => l.is_set_high(),
        }
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        match self {
            Leds::LED0(l) => l.is_set_low(),
            Leds::LED1(l) => l.is_set_low(),
            Leds::LED2(l) => l.is_set_low(),
            Leds::LED3(l) => l.is_set_low(),
        }
    }
}

impl atsamd_hal::prelude::_atsamd_hal_embedded_hal_digital_v2_OutputPin for Leds<'_> {
    type Error = core::convert::Infallible;
    fn set_low(&mut self) -> Result<(), Self::Error> {
        match self {
            Leds::LED0(l) => l.set_low(),
            Leds::LED1(l) => l.set_low(),
            Leds::LED2(l) => l.set_low(),
            Leds::LED3(l) => l.set_low(),
        }
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        match self {
            Leds::LED0(l) => l.set_high(),
            Leds::LED1(l) => l.set_high(),
            Leds::LED2(l) => l.set_high(),
            Leds::LED3(l) => l.set_high(),
        }
    }
}

impl ehal::digital::OutputPin for Leds<'_> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        match self {
            Leds::LED0(l) => l.set_low(),
            Leds::LED1(l) => l.set_low(),
            Leds::LED2(l) => l.set_low(),
            Leds::LED3(l) => l.set_low(),
        }
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        match self {
            Leds::LED0(l) => l.set_high(),
            Leds::LED1(l) => l.set_high(),
            Leds::LED2(l) => l.set_high(),
            Leds::LED3(l) => l.set_high(),
        }
    }
}

impl ehal::digital::ErrorType for Leds<'_> {
    type Error = core::convert::Infallible;
}

// *** Enum to abstract which pins are the stress voltage enables *** //
pub enum VsEn<'l> {
    VL0(&'l mut hal::gpio::pin::Pin<hal::gpio::pin::PB03, hal::gpio::pin::PushPullOutput>),
    VL1(&'l mut hal::gpio::pin::Pin<hal::gpio::pin::PB01, hal::gpio::pin::PushPullOutput>),
    VH0(&'l mut hal::gpio::pin::Pin<hal::gpio::pin::PB02, hal::gpio::pin::PushPullOutput>),
    VH1(&'l mut hal::gpio::pin::Pin<hal::gpio::pin::PB00, hal::gpio::pin::PushPullOutput>),
}

impl ehal::digital::StatefulOutputPin for VsEn<'_> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        match self {
            VsEn::VL0(l) => l.is_set_high(),
            VsEn::VL1(l) => l.is_set_high(),
            VsEn::VH0(l) => l.is_set_high(),
            VsEn::VH1(l) => l.is_set_high(),
        }
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        match self {
            VsEn::VL0(l) => l.is_set_low(),
            VsEn::VL1(l) => l.is_set_low(),
            VsEn::VH0(l) => l.is_set_low(),
            VsEn::VH1(l) => l.is_set_low(),
        }
    }
}

impl atsamd_hal::prelude::_atsamd_hal_embedded_hal_digital_v2_OutputPin for VsEn<'_> {
    type Error = core::convert::Infallible;
    fn set_low(&mut self) -> Result<(), Self::Error> {
        match self {
            VsEn::VL0(l) => l.set_low(),
            VsEn::VL1(l) => l.set_low(),
            VsEn::VH0(l) => l.set_low(),
            VsEn::VH1(l) => l.set_low(),
        }
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        match self {
            VsEn::VL0(l) => l.set_high(),
            VsEn::VL1(l) => l.set_high(),
            VsEn::VH0(l) => l.set_high(),
            VsEn::VH1(l) => l.set_high(),
        }
    }
}

impl ehal::digital::OutputPin for VsEn<'_> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        match self {
            VsEn::VL0(l) => l.set_low(),
            VsEn::VL1(l) => l.set_low(),
            VsEn::VH0(l) => l.set_low(),
            VsEn::VH1(l) => l.set_low(),
        }
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        match self {
            VsEn::VL0(l) => l.set_high(),
            VsEn::VL1(l) => l.set_high(),
            VsEn::VH0(l) => l.set_high(),
            VsEn::VH1(l) => l.set_high(),
        }
    }
}

impl ehal::digital::ErrorType for VsEn<'_> {
    type Error = core::convert::Infallible;
}

// #[derive(Copy, Clone)]
// pub enum AnaMeasChnl {
//     CH0,
//     CH1,
//     CH2,
// }
// #[derive(Copy, Clone)]
// pub enum VStressChnl {
//     VSTRESSL0,
//     VSTRESSH0,
//     VSTRESSL1,
//     VSTRESSH1,
// }

pub enum SpiDevs {
    FLASH,
    DAC,
    POT,
}

#[repr(u8)]
// Potentiometer channel select, pot 0 = 0b000, pot 1 = 0b001, pot 2 = 0b010, pot 3 = 0b011, all = 0b100.
pub enum PotSel {
    VSTRSL0 = 0,
    VSTRSH0 = 1,
    VSTRSL1 = 2,
    VSTRSH1 = 3,
    ALL = 4,
}

#[repr(u8)]
pub enum StrsConds {
    TEMP,
    VSTRSL0,
    VSTRSL1,
    VSTRSH0,
    VSTRSH1,
}

// Structure representing how a 3 byte memory block is mapped to the digital outputs of the board
#[derive(Copy, Clone)]
pub struct DigCtrlSetting {
    pub gpio: u8,
    pub lv_gpio: u8,
    pub led_enable: u8,
}

// Structure representing the analog outputs of the board
#[derive(Copy, Clone)]
pub struct AnaCtrlSetting {
    pub ana_vals: [f32; 8],
}

pub fn get_board_id() -> u16 {
    let id = 0x0005;
    id
}

// Mapping function to go from DAC channel to analog output board pin
pub fn pin_to_dac_chnl_map(pin: u8) -> u8 {
    // The DAC channels correspond to the silkscreen labels of the output pins.
    match pin {
        1 => 1,
        3 => 3,
        _ => pin,
    }
}

pub fn pin_to_adc_chnl_map(pin: u8) -> u8 {
    match pin {
        0 => 14,
        1 => 15,
        2 => 2,
        3 => 3,
        4 => 7,
        5 => 6,
        6 => 5,
        7 => 4,
        _ => pin,
    }
}

pub fn vs_to_adc_chnl_map(vs: u8) -> u8 {
    match vs {
        0 => 0,
        1 => 1,
        2 => 12,
        3 => 13,
        _ => vs,
    }
}

pub fn trgt_t_to_pwm_duty(trgt_t: f32) -> f32 {
    // Determine an approximate guideline for the duty cycle required to maintain a certain
    // temperature. This is used to make the PID control more symmetric, without this the PID
    // controller is only optimized for a certain temperature set point
    if trgt_t <= 25.0 {
        0.0
    // NOTE: This range depends on the voltage of the heater power source, here target 12V
    // This curve was based on tests in open air on an insulating bed in a 23C room
    // The approximate equation based on 5 test points gives temp = -100d^2 + 240d + 25
    } else {
        1.2 - sqrtf((-0.01 * trgt_t) + 1.69)
    }
}

// TODO: Fix to use all four stress voltages, correct resistors and such
pub fn trgt_v_to_pot_code(v: f32, i: usize) -> u8 {
    // On the board, R2 is 20kOhm for low voltage rails, 82.5kOhm for high voltage rails
    let r2_k = match i {
        0 => 82.5,
        1 => 20.0,
        2 => 82.5,
        _ => 20.0,
    };
    // Equation for RT4098 topology is Vout = 0.5(R1 + R2) / R2
    // Thus: r1 in kOhm = r2(2v - 1)
    let r1_k = r2_k * ((2.0 * v) - 1.0);
    // Range of potentiometer is 0 to 100kOhm, u8 return ensures code is within range
    ((r1_k / 100.0) * 255.0) as u8
}

/// The source for the 32kHz clock can depend on the board so this function is custom
pub fn init_clocks(
    clk_unit: hal::pac::GCLK,
    mut pm: &mut hal::pac::PM,
    mut sysctrl: &mut hal::pac::SYSCTRL,
    mut nvmctrl: &mut hal::pac::NVMCTRL,
) -> hal::clock::GenericClockController {
    // GCLK0 will be 48MHz generated from the DFLL, GCLK1 is the external 32kHz oscillator
    hal::clock::GenericClockController::with_external_32kosc(
        clk_unit,
        &mut pm,
        &mut sysctrl,
        &mut nvmctrl,
    )
}

// Wrapper for customized error raising on failed code execution
// Essentially just transforms results to contain application-specific error messages
pub fn might_err<T, E>(r: Result<T, E>, cust_msg: &str) -> Result<T, &str> {
    if let Some(val) = r.ok() {
        Ok(val)
    } else {
        Err(cust_msg)
    }
}

// Manual delay function that doesn't get optimized away.
pub fn hard_delay(ticks: u32) {
    for _ in 0..ticks {
        unsafe {
            core::arch::asm!("nop");
        }
    }
}
