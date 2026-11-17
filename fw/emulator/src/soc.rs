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

//! A functional model of the parts of the STM32G474 outside of the
//! CPU core that the moteus firmware needs in order to boot and run
//! its control interrupt.
//!
//! Most peripherals are plain register files which read back what
//! was written.  Only where the firmware waits on, or otherwise
//! depends upon, hardware behavior is there anything more.
//!
//! SPI transfers, ADC conversions and CORDIC operations take time,
//! as the control interrupt waits on each of them.

use std::cell::RefCell;
use std::collections::{BTreeSet, HashMap, VecDeque};
use std::rc::Rc;

use crate::motor::Motor;
use zmu_cortex_m::bus::Bus;
use zmu_cortex_m::core::fault::Fault;

pub const CORE_HZ: u64 = 170_000_000;

const CCM_BASE: u32 = 0x1000_0000;
const CCM_SIZE: u32 = 0x8000;

const RTC_TR: u32 = 0x4000_2800;
const RTC_SSR: u32 = 0x4000_2808;
const RTC_ICSR: u32 = 0x4000_280c;
const RTC_PRER: u32 = 0x4000_2810;
const RTC_LSE_HZ: u64 = 32768;

const LPTIM1_ISR: u32 = 0x4000_7c00;

const CORDIC_CSR: u32 = 0x4002_0c00;
const CORDIC_WDATA: u32 = 0x4002_0c04;
const CORDIC_RDATA: u32 = 0x4002_0c08;
const CORDIC_CSR_RRDY: u32 = 1 << 31;

const FLASH_ACR: u32 = 0x4002_2000;
const RCC_CR: u32 = 0x4002_1000;
const RCC_CFGR: u32 = 0x4002_1008;
const RCC_BDCR: u32 = 0x4002_1090;
const RCC_CSR: u32 = 0x4002_1094;
const RCC_CRRCR: u32 = 0x4002_1098;

const TIM_CR1: u32 = 0x00;
const TIM_DIER: u32 = 0x0c;
const TIM_SR: u32 = 0x10;
const TIM_CNT: u32 = 0x24;
const TIM_PSC: u32 = 0x28;
const TIM_ARR: u32 = 0x2c;
const TIM_CCR1: u32 = 0x34;
const TIM_CCR2: u32 = 0x38;
const TIM_CCR3: u32 = 0x3c;

const ADC_ISR: u32 = 0x00;
const ADC_CR: u32 = 0x08;
const ADC_CFGR: u32 = 0x0c;
const ADC_SMPR1: u32 = 0x14;
const ADC_SMPR2: u32 = 0x18;
/// The common control register, relative to the first ADC of a
/// group.
const ADC_CCR: u32 = 0x308;
const ADC_ISR_ADRDY: u32 = 1 << 0;
const ADC_ISR_EOSMP: u32 = 1 << 1;
const ADC_ISR_EOC: u32 = 1 << 2;
const ADC_ISR_EOS: u32 = 1 << 3;
/// The sampling times selectable in SMPRx, in half ADC clock cycles.
const ADC_SAMPLE_HALF_CYCLES: [u64; 8] = [5, 13, 25, 49, 95, 185, 495, 1281];
/// A 12 bit conversion takes 12.5 ADC clock cycles.
const ADC_CONVERT_HALF_CYCLES: u64 = 25;
const ADC_PRESCALERS: [u64; 12] = [1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256];
const ADC_SQR1: u32 = 0x30;
const ADC_DR: u32 = 0x40;
/// What every ADC channel reads unless told otherwise: mid-scale,
/// which is zero current.
const ADC_DEFAULT: u32 = 2048;

const GPIO_MODER: u32 = 0x00;
const GPIO_PUPDR: u32 = 0x0c;
const GPIO_IDR: u32 = 0x10;
const GPIO_ODR: u32 = 0x14;
const GPIO_BSRR: u32 = 0x18;
const GPIO_BRR: u32 = 0x28;
const GPIOB: u32 = 0x4800_0400;
const GPIOC: u32 = 0x4800_0800;

/// What the registers of a DRV8323 hold after reset.  The firmware
/// looks for something other than all zeros or all ones in order to
/// find the gate driver.
const GATE_DRIVER_RESET: [u32; 16] = [
    0, 0, 0, 0x3ff, 0x7ff, 0x159, 0x283, 0, 0, 0, 0, 0, 0, 0, 0, 0,
];

// The moteus-r5 has an I/O expander on a bit-banged I2C.
const GPIOA: u32 = 0x4800_0000;
const EXPANDER_SDA: (u32, u32) = (GPIOA, 12);
const EXPANDER_SCL: (u32, u32) = (GPIOB, 8);

const ADC1: u32 = 0x5000_0000;
const ADC2: u32 = 0x5000_0100;
const ADC3: u32 = 0x5000_0400;
const ADC5: u32 = 0x5000_0600;
const ADC4: u32 = 0x5000_0500;
/// Matches whatever channel an ADC is converting.
const ADC_ANY_CHANNEL: u32 = 0xff;

const USART_ISR: u32 = 0x1c;

const SPI_CR1: u32 = 0x00;
const SPI_CR1_SPE: u32 = 1 << 6;
/// See where it is used.
const SPI_LATENCY: u64 = 4;
const SPI_CR2: u32 = 0x04;
const SPI_SR: u32 = 0x08;
const SPI_SR_RXNE: u32 = 1 << 0;
const SPI_SR_TXE: u32 = 1 << 1;
const SPI_SR_BSY: u32 = 1 << 7;
const SPI_DR: u32 = 0x0c;

const FDCAN_RXF0S: u32 = 0x90;
const FDCAN_RXF0A: u32 = 0x94;
const FDCAN_TXFQS: u32 = 0xc4;
const FDCAN_TXBAR: u32 = 0xcc;
const FDCAN_RAM_BASE: u32 = 0x4000_a400;
/// The size of each instance's message RAM.
const FDCAN_RAM_SIZE: u32 = 0x350;
/// Offsets of the first receive FIFO 0 element and the first transmit
/// buffer within an instance's message RAM.
const FDCAN_RAM_RX_FIFO0: u32 = 0xb0;
const FDCAN_RAM_TX_BUFFER: u32 = 0x278;
const FDCAN_ELEMENT_XTD: u32 = 1 << 30;
const FDCAN_ELEMENT_FDF: u32 = 1 << 21;
const FDCAN_ELEMENT_BRS: u32 = 1 << 20;

const CAN_FD_LENGTHS: [usize; 16] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64];

/// The controller boards which can be emulated.
///
/// The firmware determines which it is running on from resistors
/// fitted to PB10 and PB11, and where those are absent, by where it
/// finds a gate driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Board {
    /// The moteus-r4.11.
    R4,
    /// The moteus-n1, silk r1.3.
    N1,
    C1,
    X1,
    R5,
}

/// How a bit-banged SPI device is wired, as (port, pin).
struct SpiPins {
    cs: (u32, u32),
    mosi: (u32, u32),
    miso: (u32, u32),
    sck: (u32, u32),
}

impl Board {
    pub const ALL: [Board; 5] = [Board::R4, Board::N1, Board::C1, Board::X1, Board::R5];

    pub fn name(&self) -> &'static str {
        match self {
            Board::R4 => "moteus-r4",
            Board::N1 => "moteus-n1",
            Board::C1 => "moteus-c1",
            Board::X1 => "moteus-x1",
            Board::R5 => "moteus-r5",
        }
    }

    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL
            .into_iter()
            .find(|b| b.name() == name || b.name().trim_start_matches("moteus-") == name)
    }

    /// The (port, pin) of inputs with an external pull down.
    fn pulled_down(&self) -> &'static [(u32, u32)] {
        match self {
            // This is one of the hardware revision pins.
            Board::R4 => &[(GPIOA, 10)],
            Board::N1 => &[],
            Board::C1 => &[(GPIOB, 10)],
            Board::X1 => &[(GPIOB, 11)],
            Board::R5 => &[(GPIOB, 10), (GPIOB, 11)],
        }
    }

    fn encoder(&self) -> Encoder {
        match self {
            Board::R4 | Board::N1 | Board::C1 => Encoder::As5047,
            Board::X1 | Board::R5 => Encoder::Ma600,
        }
    }

    fn gate_driver(&self) -> SpiPins {
        match self {
            Board::R4 => SpiPins {
                cs: (GPIOC, 4),
                mosi: (GPIOA, 7),
                miso: (GPIOA, 6),
                sck: (GPIOA, 5),
            },
            _ => SpiPins {
                cs: (GPIOB, 0),
                mosi: (GPIOC, 13),
                miso: (GPIOC, 11),
                sck: (GPIOC, 10),
            },
        }
    }

    /// The gate driver output enable, which the firmware calls "hiz".
    fn gate_driver_power(&self) -> (u32, u32) {
        match self {
            Board::R4 => (GPIOB, 7),
            _ => (GPIOC, 15),
        }
    }

    /// The ADCs which measure the currents of phases A, B and C.
    ///
    /// The firmware reads phases A, B and C from what it calls
    /// current sense inputs 1, 3 and 2 respectively.
    fn current_sense(&self) -> [u32; 3] {
        match self {
            Board::R4 => [ADC3, ADC2, ADC1],
            Board::N1 => [ADC1, ADC3, ADC2],
            Board::C1 | Board::X1 | Board::R5 => [ADC3, ADC1, ADC2],
        }
    }

    /// The ADC and channel which measures the bus voltage, and its
    /// scale in volts per count.
    fn voltage_sense(&self) -> ((u32, u32), f32) {
        match self {
            // This is the only thing its ADC measures.
            Board::R4 => ((ADC4, ADC_ANY_CHANNEL), 0.017947),
            _ => ((ADC5, 2), 0.017947),
        }
    }

    /// Analog inputs which identify the board, as an ADC, channel and
    /// reading.
    fn identity_inputs(&self) -> &'static [((u32, u32), u32)] {
        match self {
            // The moteus-n1 encodes its version as a voltage.
            Board::N1 => &[((ADC2, 17), 4095)],
            _ => &[],
        }
    }
}

/// The on-board magnetic encoder, which is the only device on a
/// hardware SPI.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Encoder {
    As5047,
    Ma600,
}

/// What the MA600 will do with the next word it is sent.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Ma600State {
    #[default]
    Angle,
    /// Respond with this register value.
    Respond(u32),
    /// Treat the word as a register write.
    Write,
}

/// A CAN-FD frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanFrame {
    pub id: u32,
    pub extended: bool,
    pub data: Vec<u8>,
}

/// Returns the register base and message RAM base of the FDCAN
/// instance containing `addr`.
fn fdcan_base(addr: u32) -> Option<(u32, u32)> {
    let base = addr & !0x3ff;
    let index = match base {
        0x4000_6400 => 0,
        0x4000_6800 => 1,
        0x4000_6c00 => 2,
        _ => return None,
    };
    Some((base, FDCAN_RAM_BASE + index * FDCAN_RAM_SIZE))
}

fn timer_base(addr: u32) -> Option<u32> {
    let base = addr & !0x3ff;
    match base {
        // TIM2-7, TIM1, TIM8, TIM15-17, TIM20
        0x4000_0000 | 0x4000_0400 | 0x4000_0800 | 0x4000_0c00 | 0x4000_1000 | 0x4000_1400
        | 0x4001_2c00 | 0x4001_3400 | 0x4001_4000 | 0x4001_4400 | 0x4001_4800 | 0x4001_5000 => {
            Some(base)
        }
        _ => None,
    }
}

/// The NVIC interrupt number of a timer's update interrupt.
fn timer_update_irq(base: u32) -> Option<usize> {
    match base {
        0x4001_2c00 => Some(25),
        0x4000_0000 => Some(28),
        0x4000_0400 => Some(29),
        0x4000_0800 => Some(30),
        0x4000_0c00 => Some(50),
        _ => None,
    }
}

/// Returns the base of the ADC containing `addr`, excluding the
/// common registers.
fn adc_base(addr: u32) -> Option<u32> {
    let base = addr & !0xff;
    match base {
        0x5000_0000 | 0x5000_0100 | 0x5000_0400 | 0x5000_0500 | 0x5000_0600 => Some(base),
        _ => None,
    }
}

fn gpio_base(addr: u32) -> Option<u32> {
    let base = addr & !0x3ff;
    (0x4800_0000..0x4800_1c00).contains(&base).then_some(base)
}

fn usart_base(addr: u32) -> Option<u32> {
    let base = addr & !0x3ff;
    match base {
        0x4001_3800 | 0x4000_4400 | 0x4000_4800 | 0x4000_4c00 | 0x4000_5000 | 0x4000_8000 => {
            Some(base)
        }
        _ => None,
    }
}

fn spi_base(addr: u32) -> Option<u32> {
    let base = addr & !0x3ff;
    match base {
        0x4001_3000 | 0x4000_3800 | 0x4000_3c00 | 0x4001_3c00 => Some(base),
        _ => None,
    }
}

/// A motor connected to the phase outputs, whose currents the
/// current sense amplifiers see and whose rotor the on-board encoder
/// follows.
struct AttachedMotor {
    motor: Motor,
    amps_per_count: f32,
    /// What the current sense ADCs read for phases A, B and C.
    adc_counts: [u32; 3],
}

/// The progress of a hardware triggered conversion.
#[derive(Default, Clone, Copy)]
struct AdcState {
    /// When the conversion this describes was triggered.
    trigger: u64,
    /// The flags which have been raised.
    flags: u32,
    /// Those which will be raised once enough time has passed.
    pending: u32,
}

/// The state of the bit-banged SPI transfer to the gate driver.
#[derive(Default)]
struct GateDriverSpi {
    /// How many clock pulses have been seen since chip select.
    clocks: u32,
    /// The bits received so far.
    word: u32,
    /// What the device is driving on MISO.
    miso: bool,
}

/// The state of the center aligned timer which drives the PWM
/// outputs and the control interrupt.
struct PwmTimer {
    base: u32,
    start_cycle: u64,
    /// How many update events have been delivered since start.
    events: u64,
}

pub struct Soc {
    /// Log peripheral activity to stderr.
    pub trace: bool,
    board: Board,
    /// The current time, in core clock cycles.
    pub cycles: u64,
    /// Set whenever RAM which may hold code is written.
    pub code_written: bool,
    /// The number of peripheral register accesses made.
    pub accesses: u64,
    /// The time since the PWM timer last reached the top of its
    /// count, as of when the firmware most recently read the count.
    /// This is what the firmware reports as "final_timer".
    pub pwm_count_read: Option<u32>,

    ccm: Vec<u8>,
    regs: HashMap<u32, u32>,
    /// What to report for conversions, indexed by ADC base address
    /// and channel.  Anything absent reads as mid-scale.
    pub adc_values: HashMap<(u32, u32), u32>,
    /// Every ADC base address and channel the firmware has read.
    pub adc_channels_read: BTreeSet<(u32, u32)>,
    /// Frames the firmware has transmitted.
    pub can_tx: Vec<CanFrame>,
    /// Frames waiting to be received by the firmware.
    can_rx: VecDeque<CanFrame>,
    /// Whether the front of `can_rx` is in the receive FIFO.
    can_rx_loaded: bool,
    motor: Option<AttachedMotor>,
    bus_voltage: f32,
    /// Core clock cycles the processor was stalled by peripherals
    /// which insert wait states.
    pub stall_cycles: u64,
    /// The results of the most recent CORDIC operation which have yet
    /// to be read.
    cordic_results: VecDeque<u32>,
    /// When those results are available.
    cordic_ready: u64,
    /// The encoder position as of `encoder_cycle`, in revolutions.
    encoder_turns: f64,
    encoder_cycle: u64,
    /// In revolutions per core clock cycle.
    encoder_rate: f64,
    /// When the PWM timer last triggered the ADCs, which is at the
    /// top of its count.
    adc_trigger: u64,
    adc_state: HashMap<u32, AdcState>,
    gate_driver_spi: GateDriverSpi,
    /// How many clock pulses the I/O expander has seen since the last
    /// I2C start condition.
    expander_clocks: u32,
    /// Whether the I/O expander is acknowledging a byte.
    expander_ack: bool,
    /// Indexed by SPI base address, the words being received, and
    /// when each will have arrived.
    spi_rx: HashMap<u32, VecDeque<(u64, u32)>>,
    /// Indexed by SPI base address, a word written while it was
    /// disabled.
    spi_tx: HashMap<u32, u32>,
    gate_driver_regs: [u32; 16],
    ma600_state: Ma600State,
    ma600_regs: HashMap<u32, u32>,
    pwm: Option<PwmTimer>,
}

impl Soc {
    pub fn new(board: Board) -> Self {
        let mut result = Self::new_unidentified(board);
        result
            .adc_values
            .extend(board.identity_inputs().iter().copied());
        result
    }

    fn new_unidentified(board: Board) -> Self {
        Self {
            trace: false,
            board,
            cycles: 0,
            code_written: false,
            accesses: 0,
            pwm_count_read: None,
            ccm: vec![0; CCM_SIZE as usize],
            regs: HashMap::new(),
            adc_values: HashMap::new(),
            adc_channels_read: BTreeSet::new(),
            can_tx: Vec::new(),
            can_rx: VecDeque::new(),
            can_rx_loaded: false,
            motor: None,
            bus_voltage: 0.0,
            stall_cycles: 0,
            cordic_results: VecDeque::new(),
            cordic_ready: 0,
            encoder_turns: 0.0,
            encoder_cycle: 0,
            encoder_rate: 0.0,
            adc_trigger: 0,
            adc_state: HashMap::new(),
            gate_driver_spi: GateDriverSpi::default(),
            expander_clocks: 0,
            expander_ack: false,
            spi_rx: HashMap::new(),
            spi_tx: HashMap::new(),
            gate_driver_regs: GATE_DRIVER_RESET,
            ma600_state: Ma600State::default(),
            ma600_regs: HashMap::new(),
            pwm: None,
        }
    }

    /// Whether an ADC converts when the PWM timer says to.
    fn adc_hardware_triggered(&self, base: u32) -> bool {
        (self.reg(base + ADC_CFGR) >> 10) & 3 != 0
    }

    /// How many core clock cycles the sampling and conversion phases
    /// of an ADC take.
    fn adc_durations(&self, base: u32) -> (u64, u64) {
        // ADC1 and 2 share one common register, ADC3, 4 and 5 another.
        let ccr = self.reg((base & !0x3ff) + ADC_CCR);
        let divider = match (ccr >> 16) & 3 {
            // ASSUMPTION: the asynchronous clock is the core clock.
            0 => ADC_PRESCALERS[(((ccr >> 18) & 0xf) as usize).min(11)],
            1 => 1,
            2 => 2,
            _ => 4,
        };
        let channel = (self.reg(base + ADC_SQR1) >> 6) & 0x1f;
        let sample_select = if channel < 10 {
            self.reg(base + ADC_SMPR1) >> (3 * channel)
        } else {
            self.reg(base + ADC_SMPR2) >> (3 * (channel - 10))
        };
        let sample = ADC_SAMPLE_HALF_CYCLES[(sample_select & 7) as usize];
        (
            (sample * divider).div_ceil(2),
            (ADC_CONVERT_HALF_CYCLES * divider).div_ceil(2),
        )
    }

    fn adc_flags(&mut self, base: u32) -> u32 {
        let ready = self.reg(base + ADC_CR) & ADC_ISR_ADRDY;
        if !self.adc_hardware_triggered(base) {
            // Software triggered conversions complete at once.
            return ready | ADC_ISR_EOSMP | ADC_ISR_EOC | ADC_ISR_EOS;
        }
        let (sample, convert) = self.adc_durations(base);
        let trigger = self.adc_trigger;
        let elapsed = self.cycles.saturating_sub(trigger);
        let state = self.adc_state.entry(base).or_default();
        if state.trigger != trigger {
            state.trigger = trigger;
            state.pending = ADC_ISR_EOSMP | ADC_ISR_EOC | ADC_ISR_EOS;
        }
        if elapsed >= sample {
            state.flags |= state.pending & ADC_ISR_EOSMP;
            state.pending &= !ADC_ISR_EOSMP;
        }
        if elapsed >= sample + convert {
            state.flags |= state.pending;
            state.pending = 0;
        }
        ready | state.flags
    }

    /// Connect a simulated motor.  `amps_per_count` is the scale of
    /// the current sense ADCs.
    pub fn attach_motor(&mut self, amps_per_count: f32) {
        self.motor = Some(AttachedMotor {
            motor: Motor::new(),
            amps_per_count,
            adc_counts: [ADC_DEFAULT; 3],
        });
    }

    /// The attached motor, if any.
    pub fn motor(&mut self) -> Option<&mut Motor> {
        self.motor.as_mut().map(|m| &mut m.motor)
    }

    /// Advance the motor by one PWM period, with the outputs the
    /// firmware has set, and sample its currents as the ADCs will.
    fn step_motor(&mut self, timer: u32, half_period: u64) {
        if self.motor.is_none() {
            return;
        }
        let dt = (2 * half_period) as f32 / CORE_HZ as f32;
        let duty = |ccr: u32| self.reg(timer + ccr) as f32 / half_period as f32;
        // The firmware drives phases A, B and C from its outputs 1, 3
        // and 2 respectively.
        let duties = [duty(TIM_CCR1), duty(TIM_CCR3), duty(TIM_CCR2)];
        let power = self.board.gate_driver_power();
        let powered = self.reg(power.0 + GPIO_ODR) & (1 << power.1) != 0;
        let bus_voltage = self.bus_voltage;

        let attached = self.motor.as_mut().unwrap();
        if powered {
            // The motor sees each output relative to their average.
            let neutral = duties.iter().sum::<f32>() / 3.0;
            attached
                .motor
                .step(dt, duties.map(|d| (d - neutral) * bus_voltage));
        } else {
            attached.motor.step_hiz(dt);
        }

        let state = attached.motor.state();
        let counts = |amps: f32| {
            (ADC_DEFAULT as f32 + amps / attached.amps_per_count).clamp(0.0, 4095.0) as u32
        };
        // In order of phase A, B, C.
        attached.adc_counts = [
            counts(state.current_a),
            counts(state.current_b),
            counts(state.current_c),
        ];
    }

    /// The position of the on-board encoder's magnet, in revolutions.
    pub fn encoder_position(&self) -> f64 {
        if let Some(attached) = self.motor.as_ref() {
            return f64::from(attached.motor.state().position_rev);
        }
        self.encoder_turns + (self.cycles - self.encoder_cycle) as f64 * self.encoder_rate
    }

    /// Spin the on-board encoder's magnet at a constant rate from now
    /// on.
    pub fn set_encoder_velocity(&mut self, revolutions_per_s: f64) {
        self.encoder_turns = self.encoder_position();
        self.encoder_cycle = self.cycles;
        self.encoder_rate = revolutions_per_s / CORE_HZ as f64;
    }

    /// Start a CORDIC operation.  The firmware only ever uses the
    /// cosine function, which takes an angle where the full range of
    /// a signed 32 bit integer is one revolution, and produces the
    /// cosine and then the sine as signed 1.31 fixed point.
    fn cordic_write(&mut self, value: u32) {
        let radians = f64::from(value as i32) / 2147483648.0 * std::f64::consts::PI;
        let to_q31 = |v: f64| {
            (v * 2147483648.0)
                .round()
                .clamp(-2147483648.0, 2147483647.0)
        };
        self.cordic_results.clear();
        self.cordic_results
            .push_back(to_q31(radians.cos()) as i32 as u32);
        self.cordic_results
            .push_back(to_q31(radians.sin()) as i32 as u32);
        // ASSUMPTION: the result is ready the number of cycles given
        // by the PRECISION field after the write.  RM0440 says only
        // that this field sets the number of iterations, at 4 per
        // cycle.
        let precision = u64::from((self.reg(CORDIC_CSR) >> 4) & 0xf);
        // MEASURED: and three more.
        self.cordic_ready = self.cycles + precision + 3;
    }

    /// Read a CORDIC result.  Reading before it is ready makes the
    /// bus wait.
    fn cordic_read(&mut self) -> u32 {
        self.stall_cycles += self.cordic_ready.saturating_sub(self.cycles);
        self.cordic_results.pop_front().unwrap_or(0)
    }

    /// Set what the bus voltage sense input reads.
    pub fn set_bus_voltage(&mut self, volts: f32) {
        self.bus_voltage = volts;
        let (channel, scale) = self.board.voltage_sense();
        self.adc_values
            .insert(channel, ((volts / scale) as u32).min(4095));
    }

    /// Exchange one 16 bit word with the gate driver.
    ///
    /// For both the DRV8323 and DRV8353, bit 15 selects a read, bits
    /// 14:11 the register, and the remaining bits are data, which
    /// reads back as written.  No fault is ever reported.
    fn gate_driver_transfer(&mut self, word: u32) -> u32 {
        let read = word & 0x8000 != 0;
        let register = ((word >> 11) & 0xf) as usize;
        let previous = self.gate_driver_regs[register];
        if !read {
            self.gate_driver_regs[register] = word & 0x7ff;
        }
        previous
    }

    /// Exchange one 16 bit word with the on-board encoder.
    fn encoder_transfer(&mut self, word: u32) -> u32 {
        let angle = (self.encoder_position().rem_euclid(1.0) * 65536.0) as u32 & 0xffff;
        match self.board.encoder() {
            // 14 bits of angle.
            Encoder::As5047 => angle >> 2,
            Encoder::Ma600 => match std::mem::take(&mut self.ma600_state) {
                Ma600State::Respond(value) => value,
                Ma600State::Write => {
                    let value = word & 0xff;
                    self.ma600_regs.insert(word >> 8, value);
                    self.ma600_state = Ma600State::Respond(value);
                    angle
                }
                Ma600State::Angle => {
                    if word >> 8 == 0xd2 {
                        // Read a register.
                        let value = *self.ma600_regs.get(&(word & 0xff)).unwrap_or(&0);
                        self.ma600_state = Ma600State::Respond(value);
                    } else if word == 0xea54 {
                        self.ma600_state = Ma600State::Write;
                    }
                    angle
                }
            },
        }
    }

    /// React to the firmware changing GPIO outputs, which is how it
    /// talks to the gate driver.  This is SPI mode 1: the device
    /// shifts data out on the rising clock edge and samples on the
    /// falling one.
    fn gpio_outputs_changed(&mut self, base: u32, old: u32, new: u32) {
        let edge = |pin: (u32, u32)| -> Option<bool> {
            let mask = 1 << pin.1;
            (pin.0 == base && (old ^ new) & mask != 0).then_some(new & mask != 0)
        };
        if self.board == Board::R5 {
            // The expander is write only, so all that is needed is to
            // acknowledge every byte: the ninth clock of each.
            let scl_high = self.reg(EXPANDER_SCL.0 + GPIO_ODR) & (1 << EXPANDER_SCL.1) != 0;
            if edge(EXPANDER_SDA) == Some(false) && scl_high {
                // A start condition.
                self.expander_clocks = 0;
            }
            match edge(EXPANDER_SCL) {
                Some(true) => {
                    self.expander_clocks += 1;
                    self.expander_ack = self.expander_clocks % 9 == 0;
                }
                Some(false) => self.expander_ack = false,
                None => {}
            }
        }

        let pins = self.board.gate_driver();
        if edge(pins.cs) == Some(false) {
            self.gate_driver_spi = GateDriverSpi::default();
        }
        if !self.gate_driver_selected() {
            return;
        }
        match edge(pins.sck) {
            Some(true) => {
                // The first 5 bits are the command.  The register
                // contents follow, most significant bit first.
                let spi = &self.gate_driver_spi;
                let miso = if spi.clocks >= 5 && spi.clocks < 16 {
                    // The command is in the first 5 bits received.
                    let register = (spi.word >> (spi.clocks - 5)) & 0xf;
                    let contents = self.gate_driver_regs[register as usize];
                    contents & (1 << (15 - spi.clocks)) != 0
                } else {
                    false
                };
                self.gate_driver_spi.miso = miso;
            }
            Some(false) => {
                let mosi = self.reg(pins.mosi.0 + GPIO_ODR) & (1 << pins.mosi.1);
                let spi = &mut self.gate_driver_spi;
                spi.word = (spi.word << 1) | u32::from(mosi != 0);
                spi.clocks += 1;
                if spi.clocks == 16 {
                    let word = spi.word;
                    self.gate_driver_transfer(word);
                }
            }
            None => {}
        }
    }

    /// A description of the ADC configuration, for debugging.
    pub fn adc_debug(&self) -> String {
        [
            0x5000_0000u32,
            0x5000_0100,
            0x5000_0400,
            0x5000_0500,
            0x5000_0600,
        ]
        .iter()
        .map(|&base| {
            let (sample, convert) = self.adc_durations(base);
            format!(
                "ADC {base:08x} cr={:08x} cfgr={:08x} triggered={} sample={sample} \
                     convert={convert} cycles\n",
                self.reg(base + ADC_CR),
                self.reg(base + ADC_CFGR),
                self.adc_hardware_triggered(base),
            )
        })
        .collect()
    }

    fn gate_driver_selected(&self) -> bool {
        let cs = self.board.gate_driver().cs;
        // The pin only selects the device once it is being driven.
        let driven = (self.reg(cs.0 + GPIO_MODER) >> (2 * cs.1)) & 3 == 1;
        driven && self.reg(cs.0 + GPIO_ODR) & (1 << cs.1) == 0
    }

    /// A description of the GPIO configuration, for debugging.
    pub fn gpio_debug(&self) -> String {
        (0..7u32)
            .map(|i| {
                let base = 0x4800_0000 + i * 0x400;
                format!(
                    "GPIO{} moder={:08x} pupdr={:08x} odr={:04x} idr={:04x}\n",
                    (b'A' + i as u8) as char,
                    self.reg(base + GPIO_MODER),
                    self.reg(base + GPIO_PUPDR),
                    self.reg(base + GPIO_ODR),
                    self.gpio_inputs(base)
                )
            })
            .collect()
    }

    /// The level seen at each pin of a GPIO port.
    fn gpio_inputs(&self, base: u32) -> u32 {
        let moder = self.reg(base + GPIO_MODER);
        let pupdr = self.reg(base + GPIO_PUPDR);
        let odr = self.reg(base + GPIO_ODR);
        let mut result = 0;
        for pin in 0..16 {
            let output = (moder >> (2 * pin)) & 3 == 1;
            let pull_up = (pupdr >> (2 * pin)) & 3 == 1;
            let high = if (base, pin) == EXPANDER_SDA && self.expander_ack {
                false
            } else if output {
                odr & (1 << pin) != 0
            } else if (base, pin) == self.board.gate_driver().miso && self.gate_driver_selected() {
                self.gate_driver_spi.miso
            } else if self.board.pulled_down().contains(&(base, pin)) {
                false
            } else {
                // Anything not driven floats low unless pulled up.
                pull_up
            };
            result |= u32::from(high) << pin;
        }
        result
    }

    /// Queue a frame for the firmware to receive.
    pub fn can_send(&mut self, frame: CanFrame) {
        self.can_rx.push_back(frame);
    }

    /// Whether every queued frame has been received by the firmware.
    pub fn can_rx_empty(&self) -> bool {
        self.can_rx.is_empty()
    }

    fn can_load_rx(&mut self, ram: u32) {
        let Some(frame) = self.can_rx.front().cloned() else {
            return;
        };
        let element = ram + FDCAN_RAM_RX_FIFO0;
        let id = if frame.extended {
            frame.id | FDCAN_ELEMENT_XTD
        } else {
            frame.id << 18
        };
        let dlc = CAN_FD_LENGTHS
            .iter()
            .position(|&l| l >= frame.data.len())
            .unwrap_or(15);
        self.regs.insert(element, id);
        self.regs.insert(
            element + 4,
            FDCAN_ELEMENT_FDF | FDCAN_ELEMENT_BRS | ((dlc as u32) << 16),
        );
        for (i, chunk) in frame.data.chunks(4).enumerate() {
            let mut bytes = [0u8; 4];
            bytes[..chunk.len()].copy_from_slice(chunk);
            self.regs
                .insert(element + 8 + 4 * i as u32, u32::from_le_bytes(bytes));
        }
        self.can_rx_loaded = true;
    }

    fn can_transmit(&mut self, ram: u32) {
        let element = ram + FDCAN_RAM_TX_BUFFER;
        let header = self.reg(element);
        let extended = header & FDCAN_ELEMENT_XTD != 0;
        let length = CAN_FD_LENGTHS[((self.reg(element + 4) >> 16) & 0xf) as usize];
        let mut data = Vec::new();
        for i in 0..length.div_ceil(4) {
            data.extend(self.reg(element + 8 + 4 * i as u32).to_le_bytes());
        }
        data.truncate(length);
        self.can_tx.push(CanFrame {
            id: if extended {
                header & 0x1fff_ffff
            } else {
                (header >> 18) & 0x7ff
            },
            extended,
            data,
        });
    }

    /// The address of the timer generating PWM, once it has been
    /// started.
    pub fn pwm_timer(&self) -> Option<u32> {
        self.pwm.as_ref().map(|p| p.base)
    }

    /// The NVIC interrupt number of the PWM timer's update interrupt.
    pub fn pwm_irq(&self) -> Option<usize> {
        timer_update_irq(self.pwm_timer()?)
    }

    /// Advance to the current time.  Returns an interrupt to pend, if
    /// any.
    pub fn tick(&mut self) -> Option<usize> {
        let pwm = self.pwm.as_ref()?;
        let base = pwm.base;
        let half_period = u64::from(self.reg(base + TIM_ARR)).max(1);
        let events = (self.cycles - pwm.start_cycle) / half_period;
        if events == pwm.events {
            return None;
        }
        self.pwm.as_mut()?.events = events;
        if events % 2 == 1 {
            // The top of the count, where the firmware has the timer
            // trigger every ADC.
            //
            // ASSUMPTION: the trigger path through DMA and LPTIM1 adds
            // no delay.
            self.adc_trigger = self.pwm.as_ref()?.start_cycle + events * half_period;
            self.step_motor(base, half_period);
        }

        // An update event occurs at both the top and bottom of the
        // count when center aligned.
        *self.regs.entry(base + TIM_SR).or_insert(0) |= 1;
        if self.reg(base + TIM_DIER) & 1 == 0 {
            return None;
        }
        timer_update_irq(base)
    }

    /// The wait states a read of flash takes.
    pub fn flash_latency(&self) -> u32 {
        self.reg(FLASH_ACR) & 0xf
    }

    /// What the processor's clock is divided by to make that of the
    /// peripheral buses.  The firmware sets both alike.
    pub fn apb_divider(&self) -> u64 {
        match (self.reg(RCC_CFGR) >> 11) & 7 {
            0..=3 => 1,
            code => 2 << (code - 4),
        }
    }

    fn reg(&self, addr: u32) -> u32 {
        *self.regs.get(&addr).unwrap_or(&0)
    }

    /// Returns the count and whether it is counting down.
    fn pwm_count(&self, pwm: &PwmTimer) -> (u32, bool) {
        let half_period = u64::from(self.reg(pwm.base + TIM_ARR)).max(1);
        // MEASURED: a handler entered on the timer's update finds
        // that it has counted this much further than the time taken
        // to get there accounts for, with the peripheral buses at
        // full and at half speed.  Some of that will be the interrupt
        // being slower to arrive, and some the count being sampled
        // late in reading it, but only their sum is known.
        let late = 8 * self.apb_divider() - 6;
        let phase = (self.cycles + late - pwm.start_cycle) % (2 * half_period);
        if phase < half_period {
            (phase as u32, false)
        } else {
            ((2 * half_period - phase) as u32, true)
        }
    }

    fn read_timer(&mut self, base: u32, addr: u32, stored: u32) -> u32 {
        let offset = addr - base;
        if let Some(pwm) = self.pwm.as_ref().filter(|p| p.base == base) {
            let (count, down) = self.pwm_count(pwm);
            let half_period = self.reg(base + TIM_ARR);
            return match offset {
                TIM_CNT => {
                    self.pwm_count_read = Some(if down {
                        half_period - count
                    } else {
                        half_period + count
                    });
                    count
                }
                TIM_CR1 => (stored & !0x10) | (u32::from(down) << 4),
                _ => stored,
            };
        }
        if offset == TIM_CNT && self.reg(base + TIM_CR1) & 1 != 0 {
            // A free running up counter.
            let prescale = u64::from(self.reg(base + TIM_PSC)) + 1;
            let period = u64::from(self.reg(base + TIM_ARR)) + 1;
            return ((self.cycles / prescale) % period) as u32;
        }
        stored
    }

    fn read_rtc(&self, addr: u32, stored: u32) -> u32 {
        let prer = self.reg(RTC_PRER);
        let async_div = u64::from((prer >> 16) & 0x7f) + 1;
        let sync_div = u64::from(prer & 0x7fff) + 1;
        let ticks = (u128::from(self.cycles) * u128::from(RTC_LSE_HZ) / u128::from(CORE_HZ)) as u64
            / async_div;
        match addr {
            // The sub-second register counts down.
            RTC_SSR => (sync_div - 1 - (ticks % sync_div)) as u32,
            RTC_TR => {
                let seconds = ticks / sync_div;
                let bcd = |v: u64| (((v / 10) << 4) | (v % 10)) as u32;
                (bcd((seconds / 3600) % 24) << 16)
                    | (bcd((seconds / 60) % 60) << 8)
                    | bcd(seconds % 60)
            }
            // Init mode is entered at once, and the shadow
            // registers are always synchronized.
            RTC_ICSR => stored | ((stored >> 1) & 0x40) | 0x24,
            _ => stored,
        }
    }

    fn read_reg(&mut self, addr: u32) -> u32 {
        self.accesses += 1;
        let stored = self.reg(addr);
        if let Some(base) = timer_base(addr) {
            return self.read_timer(base, addr, stored);
        }
        if let Some(base) = adc_base(addr) {
            return match addr - base {
                ADC_ISR => self.adc_flags(base),
                ADC_DR
                    if self.motor.is_some()
                        && self.adc_hardware_triggered(base)
                        && self.board.current_sense().contains(&base) =>
                {
                    let phase = self.board.current_sense().iter().position(|&b| b == base);
                    self.motor.as_ref().unwrap().adc_counts[phase.unwrap()]
                }
                ADC_DR => {
                    let channel = (self.reg(base + ADC_SQR1) >> 6) & 0x1f;
                    self.adc_channels_read.insert((base, channel));
                    *self
                        .adc_values
                        .get(&(base, channel))
                        .or_else(|| self.adc_values.get(&(base, ADC_ANY_CHANNEL)))
                        .unwrap_or(&ADC_DEFAULT)
                }
                _ => stored,
            };
        }
        if let Some(base) = usart_base(addr) {
            return match addr - base {
                // The transmitter and receiver are enabled, and the
                // transmitter is idle.
                USART_ISR => stored | (1 << 22) | (1 << 21) | (1 << 7) | (1 << 6),
                _ => stored,
            };
        }
        if let Some(base) = gpio_base(addr) {
            return match addr - base {
                GPIO_IDR => {
                    let value = self.gpio_inputs(base);
                    if self.trace {
                        eprintln!("GPIO {base:08x} idr -> {value:04x}");
                    }
                    value
                }
                _ => stored,
            };
        }
        if let Some((base, ram)) = fdcan_base(addr) {
            return match addr - base {
                FDCAN_RXF0S => {
                    if !self.can_rx_loaded {
                        self.can_load_rx(ram);
                    }
                    // The fill level, with a get index of zero.
                    u32::from(self.can_rx_loaded)
                }
                // Only the first transmit buffer is ever used, and it
                // is always free.
                FDCAN_TXFQS => 3,
                _ => stored,
            };
        }
        if let Some(base) = spi_base(addr) {
            let now = self.cycles;
            let pending = self.spi_rx.entry(base).or_default();
            // A word can be read once its last bit has been clocked in.
            let received = pending.front().is_some_and(|(ready, _)| *ready <= now);
            return match addr - base {
                SPI_SR => {
                    let busy = !pending.is_empty() && !received;
                    // The transmitter always has room.
                    SPI_SR_TXE
                        | if received { SPI_SR_RXNE } else { 0 }
                        | if busy { SPI_SR_BSY } else { 0 }
                }
                SPI_DR if received => pending.pop_front().map_or(0, |(_, word)| word),
                SPI_DR => 0,
                _ => stored,
            };
        }
        match addr {
            RCC_CR => {
                // Each oscillator is ready as soon as it is enabled.
                let mut value = stored | 0x500;
                for on_bit in [16, 24] {
                    value |= (value & (1 << on_bit)) << 1;
                }
                value
            }
            // The clock switch takes effect at once.
            RCC_CFGR => (stored & !0xc) | ((stored & 3) << 2),
            RCC_BDCR | RCC_CSR | RCC_CRRCR => stored | ((stored & 1) << 1),
            RTC_TR | RTC_SSR | RTC_ICSR => self.read_rtc(addr, stored),
            // Register updates complete at once.
            LPTIM1_ISR => stored | 0x18,
            CORDIC_RDATA => self.cordic_read(),
            CORDIC_CSR => {
                let ready = self.cycles >= self.cordic_ready && !self.cordic_results.is_empty();
                (stored & !CORDIC_CSR_RRDY) | if ready { CORDIC_CSR_RRDY } else { 0 }
            }
            _ => stored,
        }
    }

    fn write_reg(&mut self, addr: u32, value: u32) {
        self.accesses += 1;
        let mut value = value;
        if let Some(base) = timer_base(addr) {
            match addr - base {
                // Status flags are cleared by writing zero.
                TIM_SR => value &= self.reg(addr),
                TIM_CR1 => {
                    let starting = value & 1 != 0 && self.reg(addr) & 1 == 0;
                    let center_aligned = (value >> 5) & 3 != 0;
                    if starting && center_aligned {
                        self.pwm = Some(PwmTimer {
                            base,
                            start_cycle: self.cycles,
                            events: 0,
                        });
                    }
                    let stopping = value & 1 == 0 && self.reg(addr) & 1 != 0;
                    if stopping && self.pwm.as_ref().is_some_and(|p| p.base == base) {
                        self.pwm = None;
                    }
                }
                _ => {}
            }
        } else if let Some(base) = adc_base(addr) {
            if addr - base == ADC_ISR {
                // Flags are cleared by writing one.
                self.adc_state.entry(base).or_default().flags &= !value;
            }
            if addr - base == ADC_CR {
                // Calibration completes at once.
                value &= !(1 << 31);
                // As do disable and stop requests.
                if value & 0x02 != 0 {
                    value &= !0x03;
                }
                if value & 0x10 != 0 {
                    value &= !0x14;
                }
                // A software triggered single conversion does too.
                let cfgr = self.reg(base + ADC_CFGR);
                let hardware_trigger = (cfgr >> 10) & 3 != 0;
                let continuous = cfgr & (1 << 13) != 0;
                if !hardware_trigger && !continuous {
                    value &= !0x04;
                }
            }
        } else if let Some(base) = gpio_base(addr) {
            // Set and reset writes act upon the output register.
            let odr = self.reg(base + GPIO_ODR);
            let new_odr = match addr - base {
                GPIO_ODR => value,
                GPIO_BSRR => (odr | (value & 0xffff)) & !(value >> 16),
                GPIO_BRR => odr & !value,
                _ => odr,
            };
            if new_odr != odr {
                self.regs.insert(base + GPIO_ODR, new_odr);
                self.gpio_outputs_changed(base, odr, new_odr);
            }
        } else if let Some((base, ram)) = fdcan_base(addr) {
            match addr - base {
                FDCAN_RXF0A => {
                    if self.can_rx_loaded {
                        self.can_rx.pop_front();
                        self.can_rx_loaded = false;
                    }
                }
                FDCAN_TXBAR => self.can_transmit(ram),
                _ => {}
            }
        } else if let Some(base) = spi_base(addr) {
            // A word written while the SPI is disabled waits in its
            // FIFO, and is sent when it is enabled.
            let enabled = |cr1: u32| cr1 & SPI_CR1_SPE != 0;
            let send = match addr - base {
                SPI_DR if enabled(self.reg(base + SPI_CR1)) => Some(value),
                SPI_DR => {
                    self.spi_tx.insert(base, value);
                    None
                }
                SPI_CR1 if enabled(value) && !enabled(self.reg(base + SPI_CR1)) => {
                    self.spi_tx.remove(&base)
                }
                _ => None,
            };
            if let Some(word) = send {
                let response = self.encoder_transfer(word & 0xffff);
                let cr1 = if addr - base == SPI_CR1 {
                    value
                } else {
                    self.reg(base + SPI_CR1)
                };
                let divider = (2u64 << ((cr1 >> 3) & 7)) * self.apb_divider();
                let bits = u64::from((self.reg(base + SPI_CR2) >> 8) & 0xf) + 1;
                // MEASURED: how long it is from the write which
                // starts a transfer until what was received can be
                // seen.
                let ready = self.cycles + bits * divider + SPI_LATENCY * self.apb_divider() + 1;
                self.spi_rx
                    .entry(base)
                    .or_default()
                    .push_back((ready, response));
            }
        }
        if addr == CORDIC_WDATA {
            self.cordic_write(value);
        }
        self.regs.insert(addr, value);
    }

    fn ccm_offset(addr: u32) -> Option<usize> {
        (CCM_BASE..CCM_BASE + CCM_SIZE)
            .contains(&addr)
            .then(|| (addr - CCM_BASE) as usize)
    }

    fn read(&mut self, addr: u32, size: usize) -> u32 {
        if let Some(offset) = Self::ccm_offset(addr) {
            let mut bytes = [0u8; 4];
            bytes[..size].copy_from_slice(&self.ccm[offset..offset + size]);
            return u32::from_le_bytes(bytes);
        }
        let shift = (addr & 3) * 8;
        self.read_reg(addr & !3) >> shift
    }

    fn write(&mut self, addr: u32, value: u32, size: usize) {
        if let Some(offset) = Self::ccm_offset(addr) {
            self.code_written = true;
            self.ccm[offset..offset + size].copy_from_slice(&value.to_le_bytes()[..size]);
            return;
        }
        if size == 4 {
            self.write_reg(addr, value);
            return;
        }
        let shift = (addr & 3) * 8;
        let mask = ((1u32 << (size * 8)) - 1) << shift;
        let old = self.reg(addr & !3);
        self.write_reg(addr & !3, (old & !mask) | ((value << shift) & mask));
    }
}

/// Attaches a shared [`Soc`] to the simulated processor's bus.
pub struct SocBus(pub Rc<RefCell<Soc>>);

impl Bus for SocBus {
    fn read32(&mut self, addr: u32) -> Result<u32, Fault> {
        Ok(self.0.borrow_mut().read(addr, 4))
    }

    fn read16(&self, addr: u32) -> Result<u16, Fault> {
        Ok(self.0.borrow_mut().read(addr, 2) as u16)
    }

    fn read8(&self, addr: u32) -> Result<u8, Fault> {
        Ok(self.0.borrow_mut().read(addr, 1) as u8)
    }

    fn write32(&mut self, addr: u32, value: u32) -> Result<(), Fault> {
        self.0.borrow_mut().write(addr, value, 4);
        Ok(())
    }

    fn write16(&mut self, addr: u32, value: u16) -> Result<(), Fault> {
        self.0.borrow_mut().write(addr, u32::from(value), 2);
        Ok(())
    }

    fn write8(&mut self, addr: u32, value: u8) -> Result<(), Fault> {
        self.0.borrow_mut().write(addr, u32::from(value), 1);
        Ok(())
    }

    fn in_range(&self, addr: u32) -> bool {
        Soc::ccm_offset(addr).is_some()
            // APB1, APB2 and AHB1 peripherals.
            || (0x4000_0000..0x4003_0000).contains(&addr)
            // GPIO.
            || (0x4800_0000..0x4800_2000).contains(&addr)
            // ADC, DAC, AES and RNG.
            || (0x5000_0000..0x5006_1000).contains(&addr)
            // System memory, which holds the device unique ID.
            || (0x1fff_0000..0x2000_0000).contains(&addr)
    }
}
