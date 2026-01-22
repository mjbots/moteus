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

//! Register definitions for moteus controllers.
//!
//! The full list can be found at:
//! <https://mjbots.github.io/moteus/protocol/registers/>

use num_enum::{IntoPrimitive, TryFromPrimitive};

/// Registers exposed for reading or writing from a moteus controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, IntoPrimitive, TryFromPrimitive)]
#[repr(u16)]
pub enum Register {
    // === Telemetry Registers (0x000 - 0x00f) ===
    /// Current mode of operation
    Mode = 0x000,
    /// Output shaft position in revolutions
    Position = 0x001,
    /// Output shaft velocity in revolutions/second
    Velocity = 0x002,
    /// Motor torque in Nm
    Torque = 0x003,
    /// Q-axis current in A
    QCurrent = 0x004,
    /// D-axis current in A
    DCurrent = 0x005,
    /// Absolute encoder position (if configured)
    AbsPosition = 0x006,
    /// Electrical power in W
    Power = 0x007,

    /// Motor temperature in C
    MotorTemperature = 0x00a,
    /// 1 if trajectory is complete
    TrajectoryComplete = 0x00b,
    /// Home/rezero state
    HomeState = 0x00c,
    /// Input voltage in V
    Voltage = 0x00d,
    /// Board temperature in C
    Temperature = 0x00e,
    /// Fault code (0 = no fault)
    Fault = 0x00f,

    // === PWM Registers (0x010 - 0x012) ===
    /// PWM duty cycle for phase A
    PwmPhaseA = 0x010,
    /// PWM duty cycle for phase B
    PwmPhaseB = 0x011,
    /// PWM duty cycle for phase C
    PwmPhaseC = 0x012,

    // === Phase Voltage Registers (0x014 - 0x016) ===
    /// Phase A voltage in V
    VoltagePhaseA = 0x014,
    /// Phase B voltage in V
    VoltagePhaseB = 0x015,
    /// Phase C voltage in V
    VoltagePhaseC = 0x016,

    // === VFOC Registers (0x018 - 0x01e) ===
    /// Voltage FOC theta angle
    VFocTheta = 0x018,
    /// Voltage FOC voltage
    VFocVoltage = 0x019,
    /// Voltage DQ D component
    VoltageDqD = 0x01a,
    /// Voltage DQ Q component
    VoltageDqQ = 0x01b,
    /// Command Q current for current mode
    CommandQCurrent = 0x01c,
    /// Command D current for current mode
    CommandDCurrent = 0x01d,
    /// Voltage FOC theta rate
    VFocThetaRate = 0x01e,

    // === Position Mode Command Registers (0x020 - 0x02d) ===
    /// Target position in revolutions
    CommandPosition = 0x020,
    /// Target velocity in revolutions/second
    CommandVelocity = 0x021,
    /// Feedforward torque in Nm
    CommandFeedforwardTorque = 0x022,
    /// Kp scale (0-1)
    CommandKpScale = 0x023,
    /// Kd scale (0-1)
    CommandKdScale = 0x024,
    /// Maximum torque in Nm
    CommandPositionMaxTorque = 0x025,
    /// Stop position (for trajectories)
    CommandStopPosition = 0x026,
    /// Watchdog timeout in seconds
    CommandTimeout = 0x027,
    /// Velocity limit in revolutions/second
    CommandVelocityLimit = 0x028,
    /// Acceleration limit in revolutions/second^2
    CommandAccelLimit = 0x029,
    /// Fixed voltage override
    CommandFixedVoltageOverride = 0x02a,
    /// Current limit scale
    CommandIlimitScale = 0x02b,
    /// Fixed current override
    CommandFixedCurrentOverride = 0x02c,
    /// Ignore position bounds flag
    CommandIgnorePositionBounds = 0x02d,

    // === Position Torque Reporting Registers (0x030 - 0x034) ===
    /// Position Kp torque
    PositionKp = 0x030,
    /// Position Ki torque
    PositionKi = 0x031,
    /// Position Kd torque
    PositionKd = 0x032,
    /// Position feedforward torque
    PositionFeedforward = 0x033,
    /// Position total torque
    PositionCommand = 0x034,

    // === Control Registers (0x038 - 0x03d) ===
    /// Control position target
    ControlPosition = 0x038,
    /// Control velocity target
    ControlVelocity = 0x039,
    /// Control torque target
    ControlTorque = 0x03a,
    /// Position error
    ControlPositionError = 0x03b,
    /// Velocity error
    ControlVelocityError = 0x03c,
    /// Torque error
    ControlTorqueError = 0x03d,

    // === Stay-Within Mode Command Registers (0x040 - 0x048) ===
    /// Lower position bound
    CommandStayWithinLowerBound = 0x040,
    /// Upper position bound
    CommandStayWithinUpperBound = 0x041,
    /// Stay-within feedforward torque
    CommandStayWithinFeedforwardTorque = 0x042,
    /// Stay-within Kp scale
    CommandStayWithinKpScale = 0x043,
    /// Stay-within Kd scale
    CommandStayWithinKdScale = 0x044,
    /// Stay-within maximum torque
    CommandStayWithinPositionMaxTorque = 0x045,
    /// Stay-within timeout
    CommandStayWithinTimeout = 0x046,
    /// Stay-within current limit scale
    CommandStayWithinIlimitScale = 0x047,
    /// Stay-within ignore position bounds
    CommandStayWithinIgnorePositionBounds = 0x048,

    // === Encoder Registers (0x050 - 0x058) ===
    /// Encoder 0 position
    Encoder0Position = 0x050,
    /// Encoder 0 velocity
    Encoder0Velocity = 0x051,
    /// Encoder 1 position
    Encoder1Position = 0x052,
    /// Encoder 1 velocity
    Encoder1Velocity = 0x053,
    /// Encoder 2 position
    Encoder2Position = 0x054,
    /// Encoder 2 velocity
    Encoder2Velocity = 0x055,
    /// Encoder validity flags
    EncoderValidity = 0x058,

    // === GPIO Registers (0x05c - 0x05f) ===
    /// Aux1 GPIO command
    Aux1GpioCommand = 0x05c,
    /// Aux2 GPIO command
    Aux2GpioCommand = 0x05d,
    /// Aux1 GPIO status
    Aux1GpioStatus = 0x05e,
    /// Aux2 GPIO status
    Aux2GpioStatus = 0x05f,

    // === Analog Input Registers (0x060 - 0x06c) ===
    /// Aux1 analog input 1
    Aux1AnalogIn1 = 0x060,
    /// Aux1 analog input 2
    Aux1AnalogIn2 = 0x061,
    /// Aux1 analog input 3
    Aux1AnalogIn3 = 0x062,
    /// Aux1 analog input 4
    Aux1AnalogIn4 = 0x063,
    /// Aux1 analog input 5
    Aux1AnalogIn5 = 0x064,
    /// Aux2 analog input 1
    Aux2AnalogIn1 = 0x068,
    /// Aux2 analog input 2
    Aux2AnalogIn2 = 0x069,
    /// Aux2 analog input 3
    Aux2AnalogIn3 = 0x06a,
    /// Aux2 analog input 4
    Aux2AnalogIn4 = 0x06b,
    /// Aux2 analog input 5
    Aux2AnalogIn5 = 0x06c,

    // === Timing Registers (0x070 - 0x071) ===
    /// Millisecond counter
    MillisecondCounter = 0x070,
    /// Clock trim value
    ClockTrim = 0x071,

    // === PWM Input Registers (0x072 - 0x075) ===
    /// Aux1 PWM input period in microseconds
    Aux1PwmInputPeriod = 0x072,
    /// Aux1 PWM input duty cycle (0-1)
    Aux1PwmInputDutyCycle = 0x073,
    /// Aux2 PWM input period in microseconds
    Aux2PwmInputPeriod = 0x074,
    /// Aux2 PWM input duty cycle (0-1)
    Aux2PwmInputDutyCycle = 0x075,

    // === PWM Output Registers (0x076 - 0x07f) ===
    /// Aux1 PWM output 1
    Aux1Pwm1 = 0x076,
    /// Aux1 PWM output 2
    Aux1Pwm2 = 0x077,
    /// Aux1 PWM output 3
    Aux1Pwm3 = 0x078,
    /// Aux1 PWM output 4
    Aux1Pwm4 = 0x079,
    /// Aux1 PWM output 5
    Aux1Pwm5 = 0x07a,
    /// Aux2 PWM output 1
    Aux2Pwm1 = 0x07b,
    /// Aux2 PWM output 2
    Aux2Pwm2 = 0x07c,
    /// Aux2 PWM output 3
    Aux2Pwm3 = 0x07d,
    /// Aux2 PWM output 4
    Aux2Pwm4 = 0x07e,
    /// Aux2 PWM output 5
    Aux2Pwm5 = 0x07f,

    // === Device Info Registers (0x100 - 0x122) ===
    /// Model number
    ModelNumber = 0x100,
    /// Firmware version
    FirmwareVersion = 0x101,
    /// Register map version
    RegisterMapVersion = 0x102,
    /// Multiplex CAN ID
    MultiplexId = 0x110,
    /// Serial number part 1
    SerialNumber1 = 0x120,
    /// Serial number part 2
    SerialNumber2 = 0x121,
    /// Serial number part 3
    SerialNumber3 = 0x122,

    // === Output Control Registers (0x130 - 0x133) ===
    /// Set output position to nearest (alias: Rezero)
    SetOutputNearest = 0x130,
    /// Set output position to exact value
    SetOutputExact = 0x131,
    /// Require reindex before motion
    RequireReindex = 0x132,
    /// Recapture position and velocity
    RecapturePositionVelocity = 0x133,

    // === Driver Fault Registers (0x140 - 0x141) ===
    /// Driver fault register 1
    DriverFault1 = 0x140,
    /// Driver fault register 2
    DriverFault2 = 0x141,

    // === UUID Registers (0x150 - 0x158) ===
    /// UUID part 1
    Uuid1 = 0x150,
    /// UUID part 2
    Uuid2 = 0x151,
    /// UUID part 3
    Uuid3 = 0x152,
    /// UUID part 4
    Uuid4 = 0x153,
    /// UUID mask part 1
    UuidMask1 = 0x154,
    /// UUID mask part 2
    UuidMask2 = 0x155,
    /// UUID mask part 3
    UuidMask3 = 0x156,
    /// UUID mask part 4
    UuidMask4 = 0x157,
    /// UUID mask capability flag
    UuidMaskCapable = 0x158,
}

impl Register {
    /// Returns the register address as a u16.
    #[inline]
    pub const fn address(self) -> u16 {
        self as u16
    }

    /// Creates a Register from a raw address.
    ///
    /// Returns `None` if the address doesn't correspond to a known register.
    pub fn from_address(addr: u16) -> Option<Register> {
        Self::try_from(addr).ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_register_addresses() {
        assert_eq!(Register::Mode.address(), 0x000);
        assert_eq!(Register::Position.address(), 0x001);
        assert_eq!(Register::Fault.address(), 0x00f);
        assert_eq!(Register::CommandPosition.address(), 0x020);
    }

    #[test]
    fn test_register_from_address() {
        assert_eq!(Register::from_address(0x000), Some(Register::Mode));
        assert_eq!(Register::from_address(0x001), Some(Register::Position));
        assert_eq!(Register::from_address(0xfff), None);
    }
}
