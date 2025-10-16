# C++ Client Library

moteus provides a C++ library that can be used to command and control moteus controllers using a supported CAN-FD adapter.

## Integration

The moteus C++ library is header only, so there are several possible options for integrating the C++ library into your project.

**Copy the source files**: One option is to copy all the relevant headers into your source tree:

- [https://github.com/mjbots/moteus/tree/main/lib/cpp/mjbots/moteus](https://github.com/mjbots/moteus/tree/main/lib/cpp/mjbots/moteus)

**CMake**: There exists a top level `CMakeLists.txt` file that can be used with [CMake](https://cmake.org)'s `FetchContent` to incorporate the headers into your project.

```
include(FetchContent)
FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG        f14f789fefeae9caf185b950cf4e32df01d022ea
)

FetchContent_MakeAvailable(moteus)

add_executable(myproject myproject.cc)
target_link_libraries(myproject moteus::cpp)
```

**bazel**: moteus does export [bazel](https://bazel.build) `BUILD` files and can be integrated into bazel projects using that mechanism as well.

## Usage

Basic usage is similar, although slightly different to the [python](python.md) library.  A minimal example follows:

```cpp
#include <iostream>
#include <unistd.h>
#include "moteus.h"

namespace moteus = mjbots::moteus;

int main(int argc, char** argv) {
  moteus::Controller::DefaultArgProcess(argc, argv);

  moteus::Controller c([]() {
    moteus::Controller::Options options;
    options.id = 1;
    return options;
  }());

  moteus::PositionMode::Command command;
  command.position = std::numeric_limits<double>::quiet_NaN();

  while (true) {
    const auto maybe_result = c.SetPosition(command);
    if (maybe_result) {
      const auto& v = maybe_result->values;
      std::cout << "Mode: " << v.mode
                << " Fault: " << v.fault
                << "Position: " << v.position
                << " Velocity: " << v.velocity
                << "\n";
    }
    ::usleep(10000);
  }
  return 0;
}
```

## Specifying alternate query registers

By default, only a subset of registers are queried by the library.

### Common register selection

To select other common options you can either (a) change the default query resolution or (b) pass an "override" query resolution.

### Option (a): Change the default

```cpp
moteus::Controller c([]() {
  moteus::Controller::Options options;
  options.query_format.power = moteus::kFloat;
  return options;
}());
```

### Option (b): Specify an "override"

```cpp
moteus::Controller c;
moteus::PositionMode::Command command;
moteus::Query::Formay query_override;

query_override.power = moteus::kFloat;
c.SetPosition(command, nullptr, &query_override);
```

### Less common register selection

For registers which do not have predefined fields in `Query::Format`, you can use the `extra` mechanism through either the constructor options or override argument.

```cpp
moteus::Controller c([]() {
  moteus::Controller::Options options;

  options.query_format.extra[0].register_number = moteus::Register::kAux1AnalogIn1;
  options.query_format.extra[0].resolution = moteus::kFloat;

  return options;
}());
```

## Specifying alternate command registers

As with queries, by default, only a subset of registers are sent with `SetPosition` or `MakePosition` commands.  Without additional work, only "position" and "velocity" are sent.  To select others, they can either be changed through (a) the defaults or by (b) overriding them.

### Option (a): Change the default

```cpp
moteus::Controller c([]() {
  moteus::Controller::Options options;
  options.position_format.feedforward_torque = moteus::kFloat;
  return options;
}());

moteus::PositionMode::Command command;
command.position = std::numeric_limits<double>::quiet_NaN();
command.velocity = 0.0;
command.feedforward_torque = 0.1;

c.SetPosition(command);
```

### Option (b): Specify an "override"

```cpp
moteus::Controller c;

moteus::PositionMode::Command command;
command.position = std::numeric_limits<double>::quiet_NaN();
command.velocity = 0.0;
command.feedforward_torque = 0.1;

moteus::PositionMode::Format format;
format.feedforward_torque = moteus::kFloat;

c.SetPosition(command, &format);
```

## Alternate Usage Modes

Like the python library, the C++ library has two modes of operation.  One designed for ease of use, and the other for maximizing bus utilization and command rate.  The former are the `Set*` variant methods used above.  Like python, there exist `Make*` variant methods which generate a frame, but do not send it.  Those can be used with a manually constructed transport.

```cpp
auto transport = moteus::MakeSingletonTransport();

moteus::Controller c([&]() {
  moteus::Controller::Options options;
  options.transport = transport;
  return options;
}());

std::vector<moteus::CanFdFrame> commands_to_send;
std::vector<moteus::CanFdFrame> replies_to_receive;

moteus::PositionMode::Command command;
commands_to_send.push_back(c.MakePosition(command));

transport->BlockingCycle(
  commands_to_send.data(),
  commands_to_send.size(),
  &replies_to_receive);
```
