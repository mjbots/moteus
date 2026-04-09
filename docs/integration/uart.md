# UART

In systems where CAN-FD is not available, moteus can be commanded and monitored over a TTL level UART connection.

## Default configuration

By default, moteus controllers with a sufficiently new firmware version have the following pins configured for that purpose.

* moteus-r4: aux2 a/b
* moteus-c1: aux2 b/c
* moteus-n1/moteus-x1: aux1 d/e

Serial parameters:

* Baudrate: 921600
* Data bits: 8
* Stop bits: 1
* Parity: None

## Protocol

In this mode, moteus acts as if it were a fdcanusb device on the UART.  The moteus python and C++ library can be used directly if the operating system device name for the UART is given instead of the fdcanusb.  The only difference is that moteus permits checksums on command lines and always emits checksums on response lines.  Once any command has been sent with a checksum, all further commands require a checksum.

The checksum uses the following format:

```
data line *XX
```

Where the '*' character is followed by two hexadecimal digits.  Those are the CRC-8 using polynomial 0x97.  Sample calculation routines are:

=== "Python"

    ```python
    _CRC8_TABLE = [
        0x00, 0x97, 0xb9, 0x2e, 0xe5, 0x72, 0x5c, 0xcb,
        0x57, 0xc0, 0xee, 0x79, 0xb2, 0x25, 0x0b, 0x9c,
    ]

    def compute_crc8(data: bytes) -> int:
        """Compute CRC-8 using polynomial 0x97, nybble-at-a-time."""
        crc = 0
        for b in data:
            crc = _CRC8_TABLE[(crc ^ (b >> 4)) & 0x0f] ^ ((crc << 4) & 0xff)
            crc = _CRC8_TABLE[(crc ^ (b & 0x0f)) & 0x0f] ^ ((crc << 4) & 0xff)
        return crc

    ```

=== "C++"
    ```cpp
    static constexpr uint8_t kCrc8Table[16] = {
        0x00, 0x97, 0xb9, 0x2e, 0xe5, 0x72, 0x5c, 0xcb,
        0x57, 0xc0, 0xee, 0x79, 0xb2, 0x25, 0x0b, 0x9c,
    };

    static uint8_t ComputeCrc8(const char* data, size_t len) {
      uint8_t crc = 0;
      for (size_t i = 0; i < len; i++) {
        const uint8_t b = static_cast<uint8_t>(data[i]);
        crc = kCrc8Table[(crc ^ (b >> 4)) & 0x0f] ^ (crc << 4);
        crc = kCrc8Table[(crc ^ (b & 0x0f)) & 0x0f] ^ (crc << 4);
      }
      return crc;
    }
    ```

## Usage with moteus tools and library

To use moteus_tool or tview with a UART connection, you must specify the path to the UART device on the command line:

```bash
python -m moteus_gui.tview --fdcanusb /dev/ttyUSB0
```

or:

```bash
python -m moteus.moteus_tool --fdcanusb /dev/ttyUSB0 -t 1 --info
```

When integrating with the python library, you must manually construct a transport:

```python
fdcanusb = moteus.Fdcanusb('/dev/ttyUSB0')
c = moteus.Controller(id=1, transport=fdcanusb)
```

or for C++

```cpp
using mjbots;
moteus::Controller controller([]() {
  moteus::Controller::Options options;
  options.transport = std::make_shared<moteus::Fdcanusb>("/dev/ttyUSB0");
  return options;
}());
```

For usage with Arduino, see that [platform integration reference](../platforms/arduino.md).

## Caveats

### Reconfiguring

If a UART is the only means of communication, it can be challenging to configure the UART pins to a different port or a different baud rate. As with nearly all moteus configurable values, configuration changes take place *immediately*.  Thus it is recommended to first configure the new port location.  If the new port is on a lower number aux port, as soon as it is configured it will become the only means of control.  Then the host can be switched to that port in order to save configuration and reconfigure the original UART pins.

Similarly, if an error is present on a given aux port configuration, then that will disable UART control along with all other functions on that aux port, even if the error is associated with a different pin on that aux port.  It may be necessary to power cycle the device or use CAN-FD to restore control.

### Addressing

moteus emulates a subset of the fdcanusb protocol, including any configured ID and CAN prefix.  Thus if you change `id.id` or `can.prefix`, you will need to use the correct CAN IDs and prefixes in further communication with the device.

### Non-bus topology

When using UART control, only a single moteus device can be connected to a single host UART port.

### Flashing

It is not possible to flash new firmware using the UART transport.  Only CAN-FD and the SWD port with a st-link are supported.
