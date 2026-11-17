# moteus firmware emulator

This runs the unmodified moteus firmware image on the host, so that
properties of the control interrupt can be measured as part of the
normal build instead of only on real hardware.

```
tools/bazel test --config=host //fw/emulator:host
tools/bazel run --config=host //fw/emulator:emulator -- \
    --board=c1 --motor /path/to/moteus.elf 1000 \
    "conf set motor.poles 14" "conf set motor.resistance_ohm 0.047" \
    "conf set motor.Kv 304" "conf set servopos.position_min nan" \
    "conf set servopos.position_max nan" "d pos nan 2 nan" 500
```

The command line tool boots the firmware, sends it each diagnostic
channel command in turn, then measures the given number of control
interrupts and prints a per-function breakdown.  In place of a
command, a bare number runs that many control interrupts, and
`load N` applies a torque to the simulated motor that `--motor`
connects.

## Structure

- The CPU core is [zmu](https://github.com/jjkt/zmu), of which
  `zmu_cortex_m/` is a copy with changes, described below.  Only its
  functional behavior is used.
- `src/timing.rs` is the model of how many clock cycles each
  instruction takes on the Cortex-M4F in the STM32G474.  It works
  from raw opcodes and so is independent of zmu.
- `src/soc.rs` models the STM32G474 peripherals and what is attached
  to them on each board.  Nearly all peripherals are plain register
  files; only what the firmware waits upon or reads back has
  behavior.
- `src/machine.rs` loads the firmware, runs it, talks to it over the
  emulated CAN bus, and records each execution of the control
  interrupt.
- `firmware.bzl` builds the firmware for the stm32g4 from within a
  host build.

## Tests

- `modes_test_<board>` runs each supported board with a simulated
  motor: stopped, holding position, following a trajectory, turning
  at a constant speed, holding against a load, and with field
  weakening both merely enabled and actually required to exceed the
  base speed.  It checks that the motor does what was commanded,
  and that the firmware's "final_timer" in each is within a few tens
  of cycles of what `EXPECTED` in the test says it is.  Nothing in
  the emulator varies from run to run, so that only changes when the
  firmware does, and the test prints what to put there when it has.
  For the two cases the dynamometer test applies a limit to, it also
  streams telemetry as that test does and checks what is streamed,
  which is higher than most interrupts take.
- `cpu_test` checks zmu against three sets of vectors, each of which
  a script in `tests/` generates:
  - `fp_vectors.txt`, from the host's IEEE-754 arithmetic
  - `int_vectors.txt`, from an independent model of the documented
    behavior of the integer data processing instructions
  - `flow_vectors.txt`, likewise for loads, stores, branches and IT
    blocks

  Between them they cover the instructions the firmware uses.
  Exception entry and return are only checked by the firmware
  running correctly.
- `calibration_test` holds the timing model to what was measured on
  real hardware, as described below.
- `boot_test` checks only that the firmware boots.

## What is modelled

- The moteus-r4 (r4.11), moteus-n1, moteus-c1, moteus-x1 and
  moteus-r5, each identified the way the firmware does so.
- Instruction timing, starting from the Cortex-M4 TRM and corrected
  by measurement: load and store pipelining and what each kind of
  memory costs, address generation stalls, data dependent divides,
  branch refills, the floating point latencies including divides,
  square roots and multiply accumulates overlapping integer work, the
  fetches which runs of 32 bit instructions fall behind on, and
  exception entry, return and lazy floating point state saving.
- The peripheral bus clocks and flash wait states the firmware
  selects.
- Flash wait states with the prefetcher and the instruction and
  literal caches.  The control interrupt runs from CCM RAM, so this
  only matters for the library routines it calls.
- ADC sampling and conversion time, triggered from the PWM timer,
  and the time SPI transfers to the on-board encoder take.  With the
  default configuration both finish before the control interrupt
  needs them, so neither currently affects its duration.
- A motor, using the same model as the firmware simulation tests
  (`fw/test/spmsm_motor_simulator.h`, through `motor_sim.cc`).  It
  is driven by the PWM duty cycles the firmware sets, its phase
  currents appear on the current sense ADCs, and the on-board encoder
  follows its rotor.
- The CORDIC, including the wait for its result.
- The gate driver, the on-board encoder, the moteus-r5 I/O expander,
  and CAN-FD.

## Calibration against hardware

The timing model has been fitted to a real STM32G474, on a moteus-x1.

`calibration/` holds a series of programs which between them time
about 3000 short instruction sequences with the DWT cycle counter:
single instructions, load and store combinations to each kind of
memory and each pairing of them, branches and calls to variously
aligned targets, divides of different operands, floating point
chains, exception entry and return with and without floating point
state, long runs of code and every leading part of some of them,
sequences taken from the control interrupt itself, accesses to the
peripheral buses at full and half speed, and the latency of a timer
interrupt.  They
run from RAM with no firmware, so the emulator and a real controller
both run exactly the same bytes.

- `make_benchmarks.py` generates `benchmarks.txt`, the programs.
- `run_on_hardware.py` runs them on a controller through an stlink
  and openocd.  It resets the controller, runs each program from RAM
  in place of the firmware, and resets it again.  Flash is untouched
  and the firmware never runs, so the motor outputs stay disabled.
- `hardware_cycles_x1.txt` is what that measured.  Repeated runs are
  identical, and so is a run at 170MHz (`make_benchmarks.py
  --170mhz`) rather than the 16MHz the processor starts at.
- `calibration_test` runs the same programs in the emulator and
  requires the same answers.  It lists the dozen where the model is
  known to differ, and the groups which it only approximates along
  with how far out it may be.
- `compare.py` tabulates two sets of results.

To work on the model, write out what it predicts and compare:

```
tools/bazel build --config=host //fw/emulator:calibration_test
CALIBRATION_WRITE=/tmp/model.txt \
    BENCHMARKS=fw/emulator/calibration/benchmarks.txt \
    HARDWARE_CYCLES=fw/emulator/calibration/hardware_cycles_x1.txt \
    bazel-bin/fw/emulator/test-*/calibration_test
fw/emulator/calibration/compare.py \
    fw/emulator/calibration/hardware_cycles_x1.txt /tmp/model.txt
```

### Checking a whole control interrupt

The benchmarks establish what individual sequences cost.  Whether
that adds up to the right answer for the firmware as a whole is
checked by running the same firmware on a controller and in the
emulator.  Read the controller's flash back, so that the emulator
boots exactly what the controller has, configuration included:

```
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c init \
    -c "dump_image /tmp/flash.bin 0x08000000 0x80000" -c shutdown
bazel-bin/fw/emulator/emulator --board=x1 --motor /tmp/flash.bin 300 \
    "d pos 10000 nan 0 a40 v100" 400
```

and compare the final_timer that prints with the one the controller
has, which `mdw` reads while the controller runs.  It is in
`BldcServo::Impl::status_`, which `g_impl_` points to, and gdb gives
both where that pointer is and how far into what it points to
final_timer is, from the firmware which the controller is running:

```
gdb-multiarch -batch moteus.elf \
    -ex 'print &moteus::BldcServo::Impl::g_impl_' \
    -ex 'print &((moteus::BldcServo::Impl*)0)->status_.final_timer'
```

For a moteus-x1 running firmware built from this revision, with a
motor attached but not turning, final_timer is most often:

| mode                              | controller | emulator   |
|-----------------------------------|-----------:|-----------:|
| stopped                           |       2183 | 2176..2190 |
| holding position                  |  3491/3495 | 3485..3497 |
| the dynamometer's trajectory      |  3665/3671 | 3664..3680 |
| that, with field weakening        |       3907 | 3904..3916 |

That firmware was not used in fitting anything.  Two things about
the comparison matter:

- How final_timer is read matters.  The dynamometer test has the
  firmware stream its telemetry, and checks every servo_stats which
  arrives.  The firmware takes those while it is busy sending, when
  what the main loop reads from flash has pushed what the interrupt
  reads from flash out of the 8 line data cache, so they are higher
  than most interrupts: by 12 holding position, and by 32 with field
  weakening.  `--poll-ms=10` with "send tel rate NAME 50" for each of
  the channels that test streams has the emulator do the same, and
  report what was streamed:

  | mode                         | controller | emulator   |
  |------------------------------|-----------:|-----------:|
  | holding position             | 3499..3515 | 3498..3506 |
  | the dynamometer's trajectory | 3665..3687 | 3666..3682 |
  | that, with field weakening   | 3921..3951 | 3917..3951 |

  The table above is instead with the CAN bus quiet, read through
  the debugger.
- Around one interrupt in ten takes 12 or so longer on a controller,
  which the emulator also shows, as a range.

To find where a difference comes from, build with
`--copt=-DMOTEUS_PERFORMANCE_MEASURE`, which makes the firmware
record the cycle counter at each stage of the interrupt, in
`status_.dwt`.  The emulator's "mem" command reads the same from its
copy.  Finer than that, a hardware breakpoint at the start of the
interrupt handler at which the cycle counter is zeroed
(`mww 0xe0001004 0`), and another at the address of interest at
which it is read, gives the time at which a controller reaches that
address.  `EMULATOR_PC_PROFILE=file` makes the emulator write the
same for every address, in the fifth column.  Have the debugger
freeze the timers while the processor is halted (`DBGMCU_APB1FZR1`
and `DBGMCU_APB2FZR`), or the firmware will see the PWM cycle
overrun.  Single addresses are only good to a couple of cycles, but
where the two drift apart shows what to write a benchmark for, which
is how most of what `src/timing.rs` models was found.

## Current limitations

- Everything has been compared on one moteus-x1, and with the motor
  stationary.
- The bridge to the peripheral buses is approximated when those run
  at half speed, as the firmware has them.
- Loads of literals from the RAM being executed from are fitted to
  what was measured rather than understood, and long runs of them
  are a cycle or two out.
- The flash instruction cache is given half the lines RM0440 says it
  has, because that is what makes the one function in flash which
  the control interrupt calls, when stopped, miss as it does on a
  controller.  Something else must fill the real one.
- `calibration_test` lists every benchmark where the model is known
  to differ from a controller.
- Unaligned loads and stores are not charged the extra they cost.
- Other interrupts never preempt the control interrupt.

## zmu_cortex_m

`zmu_cortex_m/` is the crate of that name from
https://github.com/jjkt/zmu, as of commit
72feb753fea97babc00cc88f2c75e814efc1bfd2, which is Apache 2.0.  It
is kept here rather than fetched because zmu is no longer being
developed, and because running moteus took a number of fixes to it.
Each change is its own commit, which `git log zmu_cortex_m` lists.
In outline:

- The gdb server, and every dependency on another crate, is gone.
  The last of those was the arbitrary precision arithmetic which
  floating point was done with.  The host's IEEE-754 arithmetic
  takes its place, which is exact and far faster, but does not
  maintain the cumulative exception flags in FPSCR or honor its
  other modes.
- Instructions which were not implemented: RBIT, USAT, ORN with an
  immediate, the 32 bit encodings of REV, REV16 and REVSH, SXTAB,
  SXTAH, UXTAH, VFNMA, and VCVT between floating and fixed point.
- Floating point moves ignored the condition of an enclosing IT
  block.
- SMULL treated its operands as unsigned, SBFX sign extended from
  the wrong bit, BFI and BFC of a full 32 bit field did nothing, and
  the most negative integer divided by -1 panicked.
- `LDR.W pc, [pc, #-0]`, as linker veneers use, was decoded as a
  register offset load, and a literal load into the PC did not
  branch.
- NVIC registers were indexed incorrectly for interrupts above 31,
  and only 32 interrupts had any state.
- The IT block state was neither stacked, cleared, nor restored
  around exceptions.
- Writes to AIRCR and CCR, and any access to a private peripheral
  bus register which is not modelled, faulted.  Division by zero
  always trapped, whatever CCR.DIV_0_TRP said.
- Instructions executed from RAM were decoded every time.  They are
  now cached, which requires being told when that RAM is written.

Only the Cortex-M4F configuration is built or tested.
