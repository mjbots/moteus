# Raspberry Pi Installation #

The non-gui moteus library will work on a Raspberry Pi out of the box:

```
python -m venv --system-site-packages moteus-venv
source moteus-venv/bin/activate
pip install moteus
```

However, neither pypi nor piwheels has a functioning pyside2 library,
which is needed for `moteus_gui`, and thus tview.  Fortunately, it is
packaged in Raspberry Pi OS.  To use that version, you can do the
following:

```
sudo apt install python3-pyside2* python3-serial python3-can python3-matplotlib python3-qtconsole

source moteus-venv/bin/activate
pip install asyncqt importlib_metadata pyelftools
pip install --no-deps moteus moteus_gui
```

# Running moteus_tool and tview #

pi3hat specific options for these tools are documented at https://github.com/mjbots/pi3hat/blob/master/docs/reference.md#usage-with-client-side-tools

Note that to use the pi3hat, all python scripts must be run as root.
Using sudo is one mechanism to do so:

```
sudo apt install libraspberrypi-dev

source moteus-venv/bin/activate
pip install moteus-pi3hat

sudo moteus-venv/bin/tview
```
