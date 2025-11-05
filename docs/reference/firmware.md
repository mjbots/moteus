# Flashing and building firmware

moteus controllers have both upgradeable firmware and a permissively licensed source code for the firmware.  This section describes how to flash new (or old) versions of the firmware, and how to build it from source.

## Flashing over CAN

The latest firmware can be downloaded from: [https://github.com/mjbots/moteus/releases](https://github.com/mjbots/moteus/releases)

You need the file named YYYYMMDD-moteus-HASH.elf NOT the one named
"bootloader".

Download that file and save it somewhere on your computer, then
substitute its path in place of `path/to/file.elf` in the following
command.

```
python3 -m moteus.moteus_tool --target 1 --flash path/to/file.elf
```


## Building firmware

To build the moteus firmware, an x86-64 Ubuntu 20.04, 22.04, or 24.04
system is required.

```
tools/bazel/build --config=target //:target
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
