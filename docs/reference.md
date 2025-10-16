# Reference Documentation

This page has been reorganized. Please use the navigation menu or the links below to find the content you're looking for.

## Quick Links

### Theory and Operation

<a name="theory-of-operation"></a>
**[Theory of Operation](https://mjbots.github.io/moteus/reference/theory/)** - Controller architecture and control laws

<a name="usage-modes"></a>
**[Usage Modes / Control Modes](https://mjbots.github.io/moteus/guides/control-modes/)** - Position, velocity, and torque control modes

### Configuration

<a name="initial-parameters"></a>
**[Initial Parameters / Configuration](https://mjbots.github.io/moteus/guides/configuration/)** - Getting started with configuration

<a name="encoder-configuration"></a>
**[Encoder Configuration](https://mjbots.github.io/moteus/guides/encoder-overview/)** - Encoder setup and configuration

<a name="auxiliary-port"></a>
**[Auxiliary Port](https://mjbots.github.io/moteus/reference/encoders/#auxiliary-port)** - Auxiliary connector configuration and pinouts

<a name="aux1--enc"></a>
**[AUX1/ENC Pins](https://mjbots.github.io/moteus/reference/encoders/#r411-pins-aux1enc)** - r4.11 AUX1/ENC connector pinout

<a name="aux2--abs"></a>
**[AUX2/ABS Pins](https://mjbots.github.io/moteus/reference/encoders/#r411-pins-aux2abs)** - r4.11 AUX2/ABS connector pinout

<a name="aux1-out---set-gpio-output-values"></a>
**[AUX GPIO Output Control](https://mjbots.github.io/moteus/protocol/diagnostic/#aux1-out-set-gpio-output-values)** - Setting GPIO outputs via diagnostic commands

<a name="aux12uartmode"></a>
**[AUX UART Mode](https://mjbots.github.io/moteus/reference/encoders/#uart)** - Configuring UART on auxiliary ports

<a name="pin-options"></a>
**[Pin Capabilities](https://mjbots.github.io/moteus/reference/encoders/#io-pin-capabilities)** - Available pin modes and configurations

### Configuration Parameters

<a name="idid"></a>
**[id.id](https://mjbots.github.io/moteus/reference/configuration/#idid)** - Device ID configuration

<a name="servodefault_timeout_s"></a>
**[servo.default_timeout_s](https://mjbots.github.io/moteus/reference/configuration/#servodefault_timeout_s)** - Command timeout configuration

<a name="servoflux_brake_margin_voltage"></a>
**[servo.flux_brake_margin_voltage](https://mjbots.github.io/moteus/reference/configuration/#servoflux_brake_margin_voltage)** - Flux braking voltage threshold

<a name="servomax_power_w"></a>
**[servo.max_power_W](https://mjbots.github.io/moteus/reference/configuration/#servomax_power_w)** - Maximum power limit

<a name="servopid_position"></a>
**[servo.pid_position](https://mjbots.github.io/moteus/reference/configuration/#servopid_position)** - Position PID controller parameters

<a name="servoposposition_min"></a>
**[servopos.position_min](https://mjbots.github.io/moteus/reference/configuration/#servoposposition_min)** - Minimum position limit

<a name="motor_positionoutputoffsetsign"></a>
**[motor_position.output.offset/sign](https://mjbots.github.io/moteus/reference/configuration/#motor_positionoutputoffsetsign)** - Output encoder offset and sign

<a name="motor_positionrotor_to_output_ratio"></a>
**[motor_position.rotor_to_output_ratio](https://mjbots.github.io/moteus/reference/configuration/#motor_positionrotor_to_output_ratio)** - Gear reduction ratio

<a name="conf-write"></a>
**[Configuration Commands](https://mjbots.github.io/moteus/protocol/diagnostic/#conf-configuration)** - Reading and writing configuration via console

### Register Reference

<a name="0x000---mode"></a>
**[0x000 - Mode](https://mjbots.github.io/moteus/protocol/registers/#0x000-mode)** - Operational mode register

<a name="0x00f---fault-code"></a>
**[0x00f - Fault Code](https://mjbots.github.io/moteus/protocol/registers/#0x00f-fault-code)** - Fault status register

<a name="0x014--0x15--0x16---voltage-phase-a--b--c"></a>
**[0x014/0x15/0x16 - Voltage Phase A/B/C](https://mjbots.github.io/moteus/protocol/registers/#0x014-0x15-0x16-voltage-phase-a-b-c)** - Phase voltage control registers

<a name="0x020---position-command"></a>
**[0x020 - Position Command](https://mjbots.github.io/moteus/protocol/registers/#0x020-position-command)** - Position setpoint register

<a name="0x021---velocity-command"></a>
**[0x021 - Velocity Command](https://mjbots.github.io/moteus/protocol/registers/#0x021-velocity-command)** - Velocity setpoint register

<a name="0x023---kp-scale"></a>
**[0x023 - Kp Scale](https://mjbots.github.io/moteus/protocol/registers/#0x023-kp-scale)** - Proportional gain scaling

<a name="0x025---maximum-torque"></a>
**[0x025 - Maximum Torque](https://mjbots.github.io/moteus/protocol/registers/#0x025-maximum-torque)** - Torque limit register

<a name="0x028---velocity-limit"></a>
**[0x028 - Velocity Limit](https://mjbots.github.io/moteus/protocol/registers/#0x028-velocity-limit)** - Trajectory velocity limit

<a name="0x058---encoder-validity"></a>
**[0x058 - Encoder Validity](https://mjbots.github.io/moteus/protocol/registers/#0x058-encoder-validity)** - Encoder status bitfield

<a name="0x05c---aux1-gpio-command"></a>
**[0x05c - Aux1 GPIO Command](https://mjbots.github.io/moteus/protocol/registers/#0x05c-aux1-gpio-command)** - GPIO output control register

<a name="0x070---millisecond-counter"></a>
**[0x070 - Millisecond Counter](https://mjbots.github.io/moteus/protocol/registers/#0x070-millisecond-counter)** - System timestamp register

<a name="0x131---set-output-exact"></a>
**[0x131 - Set Output Exact](https://mjbots.github.io/moteus/protocol/registers/#0x131-set-output-exact)** - Force absolute position register

<a name="0x150---0x153---uuid"></a>
**[0x150-0x153 - UUID](https://mjbots.github.io/moteus/protocol/registers/#0x150-0x153-uuid)** - Device UUID registers

<a name="0x154---0x157---uuid-mask"></a>
**[0x154-0x157 - UUID Mask](https://mjbots.github.io/moteus/protocol/registers/#0x154-0x157-uuid-mask)** - UUID filtering registers

<a name="a2a-mappings"></a>
**[Register Mappings](https://mjbots.github.io/moteus/protocol/registers/#mappings)** - Data type encodings and scaling

<a name="a2b-registers"></a>
**[Register List](https://mjbots.github.io/moteus/protocol/registers/#registers)** - Complete register documentation

### CAN Protocol

<a name="f-can-fd-communication"></a>
<a name="a1-can-format"></a>
**[CAN Format](https://mjbots.github.io/moteus/protocol/can/#can-format)** - CAN-FD frame structure

<a name="a1a-write-registers"></a>
**[Write Registers](https://mjbots.github.io/moteus/protocol/can/#write-registers)** - Register write subframe format

<a name="a3-example"></a>
**[CAN Protocol Example](https://mjbots.github.io/moteus/protocol/can/#example)** - Sample CAN frame breakdown

<a name="sending-multiple-commands-at-once"></a>
**[Sending Multiple Commands](https://mjbots.github.io/moteus/reference/client-tools/#sending-multiple-commands-at-once)** - Batching commands in tview

<a name="communicating-with-a-specific-device"></a>
**[Communicating with Specific Device](https://mjbots.github.io/moteus/reference/client-tools/#communicating-with-a-specific-device)** - Targeting devices by ID

<a name="bit-timings"></a>
**[CAN Bit Timings](https://mjbots.github.io/moteus/platforms/socketcan/#bit-timings)** - socketcan bitrate configuration

<a name="40mhz-clock-systems"></a>
<a name="80-mhz-clock-systems"></a>
**[Clock-Specific Bit Timings](https://mjbots.github.io/moteus/platforms/socketcan/#bit-timings)** - 40MHz and 80MHz configurations

### Diagnostic Commands

<a name="b-diagnostic-command-set"></a>
**[Diagnostic Protocol](https://mjbots.github.io/moteus/protocol/diagnostic/)** - Console command reference

<a name="d-pwm"></a>
**[d pwm](https://mjbots.github.io/moteus/protocol/diagnostic/#d-pwm)** - Raw PWM control command

<a name="d-nearest"></a>
**[d nearest](https://mjbots.github.io/moteus/protocol/diagnostic/#d-nearest)** - Set output nearest command

<a name="d-cfg-set-output"></a>
**[d cfg-set-output](https://mjbots.github.io/moteus/protocol/diagnostic/#d-cfg-set-output)** - Configure output encoder

### Hardware Reference

<a name="electrical--pinout"></a>
<a name="pinout"></a>
**[Pinouts](https://mjbots.github.io/moteus/reference/pinouts/)** - Connector pinouts for all board variants

<a name="jst-zh-6-swd"></a>
**[Debug Port (JST ZH-6)](https://mjbots.github.io/moteus/reference/pinouts/)** - SWD debug connector

<a name="moteus-n1c1---aux2---jst-gh-7"></a>
**[n1/c1 AUX2 (JST GH-7)](https://mjbots.github.io/moteus/reference/pinouts/)** - n1 and c1 auxiliary port connector

<a name="moteus-r4---abs---jst-zh-4"></a>
**[r4 ABS (JST ZH-4)](https://mjbots.github.io/moteus/reference/pinouts/)** - r4 absolute encoder connector

<a name="moteus-r4---pico-spox-6-enc"></a>
**[r4 ENC (Pico-SPOX-6)](https://mjbots.github.io/moteus/reference/pinouts/)** - r4 incremental encoder connector

**[Hardware Specifications](https://mjbots.github.io/moteus/reference/hardware/)** - Electrical ratings and mechanical information

**[Application Limits](https://mjbots.github.io/moteus/reference/limits/)** - Position, velocity, and performance constraints

### Calibration and Tuning

<a name="calibration"></a>
**[Calibration](https://mjbots.github.io/moteus/guides/calibration/)** - Motor and encoder calibration procedures

<a name="pid-tuning"></a>
**[PID Tuning](https://mjbots.github.io/moteus/guides/pid-tuning/)** - Controller tuning methodology

### Firmware and Tools

<a name="building-firmware"></a>
<a name="flashing-over-can"></a>
<a name="from-the-debug-port"></a>
**[Flashing and Building Firmware](https://mjbots.github.io/moteus/reference/firmware/)** - Firmware compilation and installation

<a name="tview-usage"></a>
**[tview Usage](https://mjbots.github.io/moteus/reference/client-tools/#tview-usage)** - Command-line tool documentation

### Deployment

<a name="deployment-considerations"></a>
<a name="design-considerations-for-regenerative-braking"></a>
<a name="regenerative-braking-safety"></a>
<a name="phase-wire-soldering"></a>
<a name="power-cable-construction"></a>
<a name="power-connectorization"></a>
<a name="long-daisy-chains"></a>
**[Deployment Considerations](https://mjbots.github.io/moteus/guides/electrical-setup/)** - Production design guidelines and safety considerations

### Additional Topics

<a name="hall-sensor"></a>
**[Hall Effect Sensors](https://mjbots.github.io/moteus/reference/encoders/#hall-sensor)** - Using motors with hall effect sensors or configuring hall sensor pins

<a name="position"></a>
**[Position Mode](https://mjbots.github.io/moteus/guides/control-modes/#understanding-position-mode)** - Position control details

<a name="torque-control"></a>
**[Torque Control](https://mjbots.github.io/moteus/guides/control-modes/#torque-control)** - Direct torque control mode
