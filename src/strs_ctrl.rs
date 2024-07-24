use cortex_m::prelude::_embedded_hal_Pwm;
use pid::Pid;

use crate::bsp;
use crate::hal::pwm;
use crate::periph_managers::{HotIndMngr, SpiMngr, VstrsEnMngr};
use crate::test_management::TestStepNew;

// Temperature PID controller tuned parameters
const TEMP_P: f32 = 0.15;
const TEMP_I: f32 = 0.03;
const TEMP_D: f32 = 0.7;

// Action threshold for making an adjustment to a voltage supply based on PID output value
const VOLT_ADJUST_THRESHOLD: f32 = 0.5;

// Each step of the pot for VSTRSL is ~1.5mV, so only want to cross 0.5 PID action threshold if the
// voltage strays more than 2mV away from the desired point
// 0.5 action threshold / 150 = 3.33mV offset before action
const VSTRSL_P: f32 = 130.0;
const VSTRSL_D: f32 = 60.0;

// Each step of the pot for VSTRSH is ~9mV, so only want to cross 0.5 PID action threshold if the
// voltage strays more than 9mV away from the desired point
// ( 0.5 action threshold / 75 ) * 2 voltage div factor = 13.33mV offset before action
const VSTRSH_P: f32 = 60.0;
const VSTRSH_D: f32 = 60.0;

// Conditions need to be within bounds for this number of executions of the 'adjust' function
// continuously to be considered stable
const CYCLES_UNTIL_STABLE: u32 = 32;

// Definitions for the bounds within which a stress parameter must stay to be considered stable,
// measured average value must be between TARGET + ERROR_BOUND and TARGET - ERROR_BOUND
const TEMP_STABLE_ERROR_BOUND: f32 = 1.5;
const VOLT_STABLE_ERROR_BOUND: f32 = 0.02;

pub struct StrsCtrlr {
    t_pid: Pid<f32>,
    t_ctrl_en: bool,
    t_mv_avg: f32,
    t_sensing_faulty: bool,
    t_stable_counter: u32,
    t_base: f32,
    t_duty: f32,

    v_pid: [Pid<f32>; 4],
    v_ctrl_en: [bool; 4],
    // TODO: This fb avail flag will allow compatibility with both Rev4 and Rev5 boards
    v_fb_avail: [bool; 4],
    v_mv_avg: [f32; 4],
    v_stable_counter: [u32; 4],
    v_base: [u8; 4],
    v_nudge: [i8; 4],
    v_code: [u8; 4],
}

impl StrsCtrlr {
    pub fn new(vstrs_fb_avail: [bool; 4]) -> StrsCtrlr {
        let mut t_pid = Pid::new(0.0, 1.0);
        t_pid.p(TEMP_P, 1.0);
        t_pid.i(TEMP_I, 0.2);
        t_pid.d(TEMP_D, 0.2);

        let mut v0_pid = Pid::new(0.0, 1.0);
        v0_pid.p(VSTRSL_P, 1.0);
        v0_pid.d(VSTRSL_D, 1.0);
        let mut v1_pid = Pid::new(0.0, 1.0);
        v1_pid.p(VSTRSH_P, 1.0);
        v0_pid.d(VSTRSH_D, 1.0);
        let mut v2_pid = Pid::new(0.0, 1.0);
        v2_pid.p(VSTRSL_P, 1.0);
        v0_pid.d(VSTRSL_D, 1.0);
        let mut v3_pid = Pid::new(0.0, 1.0);
        v3_pid.p(VSTRSH_P, 1.0);
        v0_pid.d(VSTRSH_D, 1.0);

        StrsCtrlr {
            t_pid,
            t_ctrl_en: false,
            t_mv_avg: 0.0,
            t_sensing_faulty: false,
            t_stable_counter: 0,
            t_base: 0.0,
            t_duty: 0.0,

            v_pid: [v0_pid, v1_pid, v2_pid, v3_pid],
            v_ctrl_en: [false, false, false, false],
            v_fb_avail: vstrs_fb_avail,
            v_mv_avg: [0.0; 4],
            v_stable_counter: [0; 4],
            v_base: [0; 4],
            v_nudge: [0; 4],
            v_code: [0; 4],
        }
    }

    pub fn update_t_mv_avg(&mut self, t_measd: f32, hot_ind: &mut HotIndMngr) {
        self.t_mv_avg = (self.t_mv_avg * 0.6) + (t_measd * 0.4);
        // If the temperature sensors are disconnected or break we will read very low temps
        // We identify this behaviour and disable temperature control to avoid runaway behaviour
        self.t_sensing_faulty = self.t_mv_avg < -35.0;
        // Track how long the temperature is stable within the tolerance range of the target temp
        if self.t_mv_avg < self.t_pid.setpoint + TEMP_STABLE_ERROR_BOUND
            && self.t_mv_avg > self.t_pid.setpoint - TEMP_STABLE_ERROR_BOUND
        {
            self.t_stable_counter += 1;
        } else {
            self.t_stable_counter = 0;
        }
        // Turn on and off the hot warning indicator as needed, ignore errors as low priority
        hot_ind.indicate_if_hot(self.t_mv_avg).ok();
    }

    pub fn get_t_mv_avg(&self) -> f32 {
        self.t_mv_avg
    }

    pub fn update_v_mv_avg(&mut self, v_measd: f32, v_i: usize) {
        // Stress voltage sensing uses a resistor divider for the higher voltage rails, correct
        let v_actual = if v_i == 1 || v_i == 3 {
            v_measd * 2.0
        } else {
            v_measd
        };
        self.v_mv_avg[v_i] = (self.v_mv_avg[v_i] * 0.6) + (v_actual * 0.4);
        // Track how long the temperature is stable within the tolerance range of the target temp
        if self.v_mv_avg[v_i] < self.v_pid[v_i].setpoint + VOLT_STABLE_ERROR_BOUND
            && self.v_mv_avg[v_i] > self.v_pid[v_i].setpoint - VOLT_STABLE_ERROR_BOUND
        {
            self.v_stable_counter[v_i] += 1;
        } else {
            self.v_stable_counter[v_i] = 0;
        }
    }

    pub fn get_v_mv_avg(&self, v_i: usize) -> f32 {
        self.v_mv_avg[v_i]
    }

    pub fn run_ctrlr(
        &mut self,
        pwm: &mut pwm::Pwm0,
        spi: &mut SpiMngr,
        en_pins: &mut VstrsEnMngr,
    ) -> Result<(), &'static str> {
        // Temperature first
        let t_adjust = self.t_pid.next_control_output(self.t_mv_avg).output;
        let t_duty_new = self.t_base + t_adjust;
        // Set the temperature PWM
        if (self.t_sensing_faulty || !self.t_ctrl_en) && (pwm.get_duty(pwm::Channel::_0) != 0) {
            pwm.set_duty(pwm::Channel::_0, 0);
            self.t_duty = 0.0;
        } else if t_duty_new != self.t_duty {
            pwm.set_duty(
                pwm::Channel::_0,
                (t_duty_new * pwm.get_max_duty() as f32) as u32,
            );
            self.t_duty = t_duty_new;
        }

        // Now for stress voltage rails
        for i in 0..4 {
            let v_adjust = self.v_pid[i].next_control_output(self.v_mv_avg[i]).output;
            if v_adjust > VOLT_ADJUST_THRESHOLD {
                self.v_nudge[i] += 1;
            } else if v_adjust < -VOLT_ADJUST_THRESHOLD {
                self.v_nudge[i] -= 1;
            }
            let v_code_new = (self.v_base[i] as i8 + self.v_nudge[i]) as u8;
            // Toggle stress voltage enable pins if needed
            if !self.v_ctrl_en[i] && en_pins.is_enabled(i)? {
                en_pins.disable(i)?;
            } else if self.v_ctrl_en[i] && en_pins.is_disabled(i)? {
                en_pins.enable(i)?;
            }
            // Send SPI messages to set the potentiometers appropriately as needed
            if !self.v_ctrl_en[i] && (self.v_code[i] != 0) {
                spi.set_pot_code(i, 0)?;
                self.v_code[i] = 0;
            } else if v_code_new != self.v_code[i] {
                spi.set_pot_code(i, v_code_new)?;
                self.v_code[i] = v_code_new;
            }
        }
        Ok(())
    }

    pub fn set_strs_trgts(&mut self, step_spec: Option<&TestStepNew>) {
        // Extract the enabled states and target values from the step if it exists, if not
        // currently in a test we disable all stress by default
        let (en_mask, t_trgt, v_trgts) = if let Some(&step) = step_spec.as_ref() {
            (step.strs_en_mask, step.temp, step.vs)
        } else {
            (0, 0.0, [0.0; 4])
        };

        // For each stress condition, set enabled state and target value
        let mut is_en = (en_mask & 0b000_1_0000) != 0;
        // While setting targets, track whether any conditions are different from the previous state, indicating
        // that time is required to shift conditions and stabilize before the controller takes further action
        if is_en && (!self.t_ctrl_en || (t_trgt != self.t_pid.setpoint)) {
            self.t_stable_counter = 0;
        }
        self.t_ctrl_en = is_en;
        self.t_pid.setpoint = t_trgt;
        self.t_base = StrsCtrlr::calc_base_duty(t_trgt);
        // Now for the voltage stress conditions
        for i in 0..4 {
            is_en = (en_mask & (0b000_0_1000 >> i)) != 0;
            let v_trgt = v_trgts[i];
            if is_en && (!self.v_ctrl_en[i] || (v_trgt != self.v_pid[i].setpoint)) {
                self.v_stable_counter[i] = 0;
            }
            self.v_ctrl_en[i] = is_en;
            self.v_pid[i].setpoint = v_trgt;
            self.v_base[i] = StrsCtrlr::calc_base_code(v_trgt, i);
        }
    }

    pub fn disable_all_stress(&mut self) {
        self.set_strs_trgts(None);
    }

    pub fn calc_base_duty(trgt_temp: f32) -> f32 {
        bsp::trgt_t_to_pwm_duty(trgt_temp)
    }

    fn calc_base_code(trgt_v: f32, v_ind: usize) -> u8 {
        crate::bsp::trgt_v_to_pot_code(trgt_v, v_ind)
    }

    pub fn is_strs_stable(&self) -> bool {
        let mut stable = true;
        if self.t_ctrl_en && self.t_stable_counter < CYCLES_UNTIL_STABLE {
            stable = false;
        }
        for i in 0..4 {
            if self.v_ctrl_en[i] && self.v_stable_counter[i] < CYCLES_UNTIL_STABLE {
                stable = false;
            }
        }
        stable
    }

    pub fn which_cond_unstable(&self) -> u8 {
        for i in 0..4 {
            if self.v_ctrl_en[i] && self.v_stable_counter[i] < CYCLES_UNTIL_STABLE {
                return i as u8;
            }
        }
        if self.t_ctrl_en && self.t_stable_counter < CYCLES_UNTIL_STABLE {
            return 4;
        }
        5
    }
}
