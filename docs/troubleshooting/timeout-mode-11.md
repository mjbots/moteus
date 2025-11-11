# moteus reports mode 11 - Timeout

After performing some initial validation using tview, maybe you port your motion profile to a python or C++ application to get ready for a deployment.  However, often users at this point will start seeing moteus enter mode 11, the timeout mode, after which it stops accepting commands and acts to damp motion.  This guide tells you why that happens and how to fix it.

## Rationale

moteus implements a "watchdog timer" for commands that arrive over CAN-FD.  Once any command is received, the next one must be received within a certain time period.  If it doesn't, then moteus will enter the timeout mode.

There are a few reasons for this feature:

1. **Safety**: In an actual deployment, moteus could have been commanded to make a large or fast movement indefinitely.  If your application crashes, or the computer hosting it is powered down, you probably don't want moteus continuing.
2. **Convenience**: When testing an application, if you exit the application you typically want moteus to stop doing whatever it was doing as well.

Once in the timeout mode, moteus will only leave it upon receiving a stop command.  This prevents moteus from resuming unexpectedly.

When using diagnostic mode commands like when typing into the diagnostic channel console in tview, moteus defaults to an infinite watchdog duration for the sake of convenience.  This is why the problem most often appears after switching to a script.  Both the python and C++ library default to using the more efficient register mode commands.

## Fixes

If you get the timeout here are ways to resolve it:

### Disable or extend the timeout duration

The length of time moteus waits is configurable at:

```
servo.default_timeout_s
```

[(listed here in the reference)](../reference/configuration.md#servodefault_timeout_s)

To disable it, use `nan`, otherwise specify a floating point number of seconds.

### Structure your application to send commands at a regular interval

To maintain the benefits of the watchdog, your application needs to continually send commands to moteus.  That means even when the application is waiting, it still needs to be sending commands.  Fortunately, for many moteus commands, if the target velocity was 0, you can keep sending the same command over and over again.

=== "Python"

    ```python
    # Don't do this:
    #  await asyncio.sleep(5)

    # Instead, do this:
    start = time.time()
    while time.time() - start < 5:
      await c.set_position(velocity=0.0, query=True)
      await asyncio.sleep(0.02)
    ```

=== "C++"

    ```cpp
    // Don't do this.
    //  ::usleep(1000000);

    // Instead, do this:
    #include <chrono>

    double get_time() {
      using namespace std::chrono;
      return duration<double>(system_clock::now().time_since_epoch()).count();
    }

    int main() {
      // setup ...

      const auto start = get_time();
      while (get_time() - start < 5) {
        mjbots::moteus::PositionMode::Command cmd;
        cmd.velocity = 0.0;
        cmd.position = NaN;
        c.SetPosition(cmd);

        ::usleep(10000);
      }

      return 0;
    }
    ```

### A combination of the above two

It may be that even after restructuring your application to send commands more regularly, it makes sense to increase, but not disable the default watchdog timeout.
