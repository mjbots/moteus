# Flashing and building firmware

moteus controllers have both upgradeable firmware and a permissively licensed source code for the firmware.  This section describes how to flash new (or old) versions of the firmware, and how to build it from source.

## Flashing over CAN

The latest firmware can be downloaded from: [https://github.com/mjbots/moteus/releases](https://github.com/mjbots/moteus/releases)

Each firmware release attaches two ELFs:

- `moteus-fw-<version>+g<sha>.elf` — the application firmware. **This
  is the one you want for normal flashing.**
- `moteus-bl-<version>+g<sha>.elf` — the CAN bootloader. Only needed
  for advanced bootloader recovery via the SWD port (see below); the
  bootloader is preinstalled on every moteus and you do not normally
  flash it.

`<version>` is the semver release (e.g. `1.0.0` or `1.0.0-rc.1`) and
`<sha>` is the 10-character git short SHA of the commit the release
was built from (build metadata under semver — informational only).

Download `moteus-fw-...elf`, save it somewhere on your computer, and
substitute its path in place of `path/to/file.elf` in the following
command.

```
python3 -m moteus.moteus_tool --target 1 --flash path/to/file.elf
```


## Building firmware

To build the moteus firmware, an x86-64 Ubuntu 22.04, 24.04, or 26.04
system is required.

First install dependencies:

```
sudo python3-build python3-can python3-serial python3-setuptools \
     python3-pyelftools python3-qtpy python3-wheel \
     python3-importlib-metadata python3-scipy python3-usb \
     mypy nodejs wget curl
```

Then the firmware can be built with:

```
tools/bazel build --config=target //:target
```

## Flash from the SWD port

If CAN-FD is inoperative, then new firmware can be flashed using the SWD connector on each moteus.  First install dependencies:

```
sudo apt install openocd binutils-arm-none-eabi
```

Then attach a [stm32 programmer](https://mjbots.com/products/stm32-programmer) to the USB port on your computer and the SWD port on the moteus.

The firmware can be built and flashed using:

```
tools/bazel build --config=target //fw:flash
```

Or, if already built, flashed using:

```
./fw/flash.py
```

The python script can be used to flash pre-compiled images as well:

```
./fw/flash.py path/to/firmware.elf path/to/bootloader.elf
```

## Debug from the SWD port

gdb can be used for debugging the firmware when an [stm32 programmer](https://mjbots.com/products/stm32-programmer) is attached to the SWD port.  First, install dependencies:

```
sudo apt install gdb-multiarch
```

Then, in one terminal run:

```
./run_openocd_noreset.sh
```

In another terminal run:

```
gdb-multiarch -x moteus-debug.gdb bazel-out/stm32g4-opt/bin/moteus.elf
```

Debugging hints:

- **Flashing Red Light**: This means a debug assertion failed.  Using `bt` to get a backtrace will tell you the call site.
- **Pool Size**: The firmware allocates data from a fixed size pool during initialization.  If new structures are added, this size can be exceeded.  Either reduce the size of structures, or increase the size of the pool.
- **Optimizations**: The moteus firmware must have optimizations enabled globally.  This makes debugging challenging.  Some strategies:
 - Expose data you need to structures available to tview
 - Single step through assembly
 - Use the `__attribute__((optimize("O0")))` on individual functions to disable optimization
