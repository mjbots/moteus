// Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! A simulated motor: the same model the firmware's own simulation
//! tests use, through the C interface in motor_sim.cc.

use std::ffi::c_void;

/// What can be observed of the motor.
#[repr(C)]
#[derive(Debug, Default, Clone, Copy)]
pub struct MotorState {
    pub current_a: f32,
    pub current_b: f32,
    pub current_c: f32,
    pub position_rev: f32,
    pub velocity_rev_s: f32,
    pub torque_nm: f32,
}

/// What the firmware must be told about the motor to control it.
#[repr(C)]
#[derive(Debug, Default, Clone, Copy)]
pub struct MotorParams {
    pub pole_pairs: i32,
    pub resistance_ohm: f32,
    pub inductance_h: f32,
    pub kv: f32,
}

extern "C" {
    fn moteus_motor_new() -> *mut c_void;
    fn moteus_motor_free(motor: *mut c_void);
    fn moteus_motor_params(out: *mut MotorParams);
    fn moteus_motor_step(motor: *mut c_void, dt_s: f32, v_a: f32, v_b: f32, v_c: f32, load: f32);
    fn moteus_motor_step_hiz(motor: *mut c_void, dt_s: f32, load: f32);
    fn moteus_motor_state(motor: *mut c_void, out: *mut MotorState);
}

pub struct Motor {
    handle: *mut c_void,
    /// An external torque applied to the rotor.
    pub load_nm: f32,
}

impl Motor {
    pub fn new() -> Self {
        Self {
            // SAFETY: Returns a new simulator which this owns until
            // it is dropped.
            handle: unsafe { moteus_motor_new() },
            load_nm: 0.0,
        }
    }

    pub fn params() -> MotorParams {
        let mut result = MotorParams::default();
        // SAFETY: The pointer is to a valid structure of the layout
        // the C side declares.
        unsafe { moteus_motor_params(&mut result) };
        result
    }

    /// Advance with the given phase to neutral voltages applied.
    pub fn step(&mut self, dt_s: f32, volts: [f32; 3]) {
        // SAFETY: The handle is valid for the life of self.
        unsafe {
            moteus_motor_step(
                self.handle,
                dt_s,
                volts[0],
                volts[1],
                volts[2],
                self.load_nm,
            )
        };
    }

    /// Advance with the phases disconnected.
    pub fn step_hiz(&mut self, dt_s: f32) {
        // SAFETY: The handle is valid for the life of self.
        unsafe { moteus_motor_step_hiz(self.handle, dt_s, self.load_nm) };
    }

    pub fn state(&self) -> MotorState {
        let mut result = MotorState::default();
        // SAFETY: The handle is valid for the life of self, and the
        // pointer is to a valid structure of the layout the C side
        // declares.
        unsafe { moteus_motor_state(self.handle, &mut result) };
        result
    }
}

impl Default for Motor {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for Motor {
    fn drop(&mut self) {
        // SAFETY: The handle came from moteus_motor_new and is not
        // used again.
        unsafe { moteus_motor_free(self.handle) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spins_up_under_voltage() {
        let mut motor = Motor::new();
        assert_eq!(Motor::params().pole_pairs, 7);
        // A fixed voltage vector pulls the rotor into alignment, which
        // means current must flow and the rotor must move.
        for _ in 0..100 {
            motor.step(1.0 / 30000.0, [0.0, 0.5, -0.5]);
        }
        let state = motor.state();
        assert!(state.current_b.abs() > 0.1, "{state:?}");
        assert!(state.position_rev != 0.0, "{state:?}");

        // With the phases open, no current flows.
        motor.step_hiz(1.0 / 30000.0);
        assert_eq!(motor.state().current_b, 0.0);
    }
}
