# pybbd202 — Thorlabs BBD20x Stage Driver

Python library for controlling Thorlabs BBD202/BBD203 brushless DC servo controllers with the MLS203-1 XY stage via the APT protocol over USB serial.

**Author:** Thomas Ales | Feb 2026

## Requirements

- Python 3
- [pyserial](https://pypi.org/project/pyserial/)

```bash
pip install pyserial
```

The controller connects as a USB serial device (default `/dev/ttyUSB1` at 115200 baud with RTS/CTS flow control).

## Project Structure

| File | Description |
|---|---|
| `bbd20x.py` | `ThorlabsServoDriver` — main driver class for axis control, homing, moves, and status polling |
| `apt_messages.py` | `APTProtocol` — APT message registry, build/unpack logic for the Thorlabs binary protocol |
| `apt_constants.py` | `StatusBits` — IntFlag enum for decoding motor status bit fields |
| `serial_comms.py` | `SerialSnooper` — threaded serial listener that frames raw bytes into complete APT messages |

## Quick Start

```python
from bbd20x import ThorlabsServoDriver
import time

stage = ThorlabsServoDriver()
stage.connect()  # opens /dev/ttyUSB1, detects bays, starts worker threads

# Enable both axes
stage.enable_axis(0x21)  # X
stage.enable_axis(0x22)  # Y
time.sleep(0.5)

# Start status polling (keeps the controller watchdog alive)
stage.start_polling()

# Home both axes (required after power-up)
stage.home_axis(0x21, timeout=60.0)
stage.home_axis(0x22, timeout=60.0)

# Move X axis to 50 mm absolute
stage.move_axis_absolute(0x21, 50.0)

# Move Y axis 10 mm relative to current position
stage.move_axis_relative(0x22, 10.0)

# Read current positions
print(f"X: {stage.positions[0]:.3f} mm")
print(f"Y: {stage.positions[1]:.3f} mm")

# Clean up
stage.stop_polling()
stage.disconnect()
```

## Axis Addresses

| Address | Axis |
|---|---|
| `0x21` | X axis (bay 0) |
| `0x22` | Y axis (bay 1) |

State arrays (`positions`, `am_homed`, `am_moving`, etc.) use index 0 for X and index 1 for Y.

## API Reference

### Connection

#### `connect(port=None, spd=None)`

Opens the serial port, starts TX/RX/polling worker threads, and probes which bays are present. Defaults to `/dev/ttyUSB1` at 115200 baud.

```python
stage = ThorlabsServoDriver()
stage.connect()                          # use defaults
stage.connect(port="/dev/ttyUSB0")       # custom port
stage.connect(port="/dev/ttyUSB0", spd=115200)
```

#### `disconnect()`

Sends disconnect messages to the controller and all bays, stops worker threads, and closes the serial port.

### Axis Control

#### `enable_axis(axis)` / `disable_axis(axis)`

Enable or disable the motor driver for the given axis.

```python
stage.enable_axis(0x21)   # enable X
stage.disable_axis(0x22)  # disable Y
```

#### `toggle_enabled_state(axis)`

Toggles the enabled state of the specified axis based on the cached state.

#### `home_axis(axis, timeout=60.0)`

Blocking homing command. Sends the home request and waits for the homed response. Required after power-up before any moves can be made.

```python
stage.home_axis(0x21, timeout=30.0)
```

### Motion

#### `move_axis_absolute(axis, position_in_mm, timeout=10.0)`

Moves the axis to an absolute position in mm. Blocks until the move completes or the timeout expires.

- X axis range: 0 — 110 mm
- Y axis range: 0 — 75 mm

```python
stage.move_axis_absolute(0x21, 55.0)   # move X to 55 mm
stage.move_axis_absolute(0x22, 37.5)   # move Y to 37.5 mm
```

#### `move_axis_relative(axis, distance_in_mm, timeout=10.0)`

Moves the axis by a relative distance in mm from its current position. Blocks until complete.

```python
stage.move_axis_relative(0x21, 5.0)    # move X forward 5 mm
stage.move_axis_relative(0x22, -2.5)   # move Y backward 2.5 mm
```

### Velocity Parameters

#### `get_velocity_params(axis, timeout=5.0)`

Queries the controller for current velocity parameters. Returns a dict:

```python
params = stage.get_velocity_params(0x21)
# {'min_velocity': 0.0, 'acceleration': 500.0, 'max_velocity': 100.0}
```

Values are in mm/s and mm/s².

#### `set_velocity_params(axis, max_velocity=None, acceleration=None)`

Sets velocity and/or acceleration for the specified axis. Any parameter left as `None` keeps its current value.

```python
stage.set_velocity_params(0x21, max_velocity=50.0, acceleration=200.0)
stage.set_velocity_params(0x22, max_velocity=25.0)  # keep current accel
```

### Status Polling

#### `start_polling(interval=0.2)` / `stop_polling()`

Starts or stops periodic `REQ_USTATUSUPDATE` requests to each bay. The RX worker automatically ACKs these responses, which keeps the controller's communications watchdog alive. Polling also continuously updates the cached state properties.

```python
stage.start_polling(interval=0.1)  # poll every 100 ms
# ... do work ...
stage.stop_polling()
```

### Trigger Modes

The BBD20x supports configurable trigger I/O modes for both input and output. Trigger modes are set per-axis and can be combined.

#### `set_trigger(axis, mode)` / `get_trigger(axis, timeout=5.0)`

Set or query the trigger mode for an axis. Mode should be a `TriggerBitsServo` value from `apt_constants.py` or a combination of values using bitwise OR.

```python
from apt_constants import TriggerBitsServo

# Set a single trigger mode
stage.set_trigger(0x21, TriggerBitsServo.TRIGIN_HIGH)

# Read back the current trigger mode
mode = stage.get_trigger(0x21)
print(f"Trigger mode: {mode}")

# Combine modes with bitwise OR
stage.set_trigger(0x21, TriggerBitsServo.TRIGIN_HIGH | TriggerBitsServo.TRIGOUT_INMOTION)
```

#### Trigger Input Modes (TRIGIN_*)

| Mode | Value | Description |
|---|---|---|
| `TRIGIN_HIGH` | 0x01 | Trigger input detects logic high |
| `TRIGIN_RELMOVE` | 0x02 | Trigger input initiates a relative move |
| `TRIGIN_ABSMOVE` | 0x04 | Trigger input initiates an absolute move |
| `TRIGIN_HOMEMOVE` | 0x08 | Trigger input initiates a home move |

#### Trigger Output Modes (TRIGOUT_*)

| Mode | Value | Description |
|---|---|---|
| `TRIGOUT_HIGH` | 0x10 | Trigger output goes high |
| `TRIGOUT_INMOTION` | 0x20 | Trigger output high while axis is in motion |
| `TRIGOUT_MOTIONCOMPLETE` | 0x40 | Trigger output pulses when motion completes |
| `TRIGOUT_MAXVELOCITY` | 0x80 | Trigger output pulses at maximum velocity |
| `TRIGOUT_MAXV` | 0x90 | Combined: high + pulse at max velocity (shorthand) |

#### Convenience Methods

For common trigger modes, use the dedicated setter methods:

```python
stage.set_trigger_trigin_high(0x21)              # TRIGIN_HIGH
stage.set_trigger_trigin_relmove(0x21)           # TRIGIN_RELMOVE
stage.set_trigger_trigin_absmove(0x21)           # TRIGIN_ABSMOVE
stage.set_trigger_trigin_homemove(0x21)          # TRIGIN_HOMEMOVE

stage.set_trigger_trigout_high(0x21)             # TRIGOUT_HIGH
stage.set_trigger_trigout_inmotion(0x21)         # TRIGOUT_INMOTION
stage.set_trigger_trigout_motioncomplete(0x21)   # TRIGOUT_MOTIONCOMPLETE
stage.set_trigger_trigout_maxvelocity(0x21)      # TRIGOUT_MAXVELOCITY
stage.set_trigger_trigout_maxv(0x21)             # TRIGOUT_MAXV (recommended default)
```

### State Properties

These are updated automatically by status poll responses and move-completed messages:

| Property | Type | Description |
|---|---|---|
| `positions[ch]` | float | Current position in mm |
| `act_velocities[ch]` | float | Current velocity in mm/s |
| `current_demand[ch]` | int | Motor current demand (raw) |
| `am_homed[ch]` | bool | Axis has been homed |
| `am_moving[ch]` | bool | Axis is currently in motion |
| `am_error[ch]` | bool | Axis has an error condition |
| `am_enabled[ch]` | bool | Axis motor driver is enabled |
| `am_connected` | bool | Serial connection is active |
| `bays_present` | list | Bay addresses detected at connect |

Where `ch` is 0 for X, 1 for Y.

### Low-Level Messaging

#### `send_and_wait(msg_id, timeout=10.0, retries=1, **kwargs)`

Sends an APT message and blocks until the expected response arrives. On timeout, drains the serial buffer and retries. Raises `TimeoutError` if all attempts fail.

```python
info = stage.send_and_wait(0x0005, destination=0x11, source=0x01)
```

#### `send_message(msg_id, **kwargs)`

Fire-and-forget: builds and queues an APT message for transmission.

## Architecture

The driver uses three daemon threads:

1. **TX worker** — serializes all outbound writes through a queue to prevent write races on the serial port.
2. **RX worker** — reads parsed messages from `SerialSnooper`, dispatches responses to waiting callers, ACKs status updates, and updates cached state.
3. **Poll worker** — periodically sends status requests to each bay to keep the controller watchdog alive and state up to date.

`SerialSnooper` (from `serial_comms.py`) runs in its own thread and handles raw serial I/O, framing variable-length APT messages from the byte stream and placing complete packets on `rx_msg_queue`.

## Scaling Factors (MLS203-1)

| Parameter | Factor |
|---|---|
| Position | 20,000 encoder counts per mm |
| Velocity | 134,217.73 counts per mm/s |
| Acceleration | 13.744 counts per mm/s² |

These are defined in `ThorlabsServoDriver` and applied automatically by all move and velocity methods.

## Troubleshooting

**No bays detected on connect** — Verify the USB cable is connected and the controller is powered on. Check that the correct serial port is being used (`/dev/ttyUSB1` by default). On Linux, you may need to add your user to the `dialout` group.

**Moves timing out** — Ensure the axis is enabled and homed before issuing move commands. Increase the `timeout` parameter if the move distance is large or velocity is low.

**Controller stops responding** — Make sure `start_polling()` has been called. The controller has a communications watchdog that requires periodic messages to stay alive.

**Permission denied on serial port** — Run `sudo usermod -aG dialout $USER` and log out/in, or use `sudo`.

## License

MIT License — see [LICENSE](LICENSE).

---

*Documentation generated by Claude.*
