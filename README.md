# BBD202/203 Motion Controller Library

Python library for controlling Thorlabs BBD202/BBD203 motion controllers via FTDI interface using the APT protocol.

## Features

- Full control of X and Y axes
- Absolute and relative positioning
- Configurable velocity and acceleration
- Automatic position tracking
- Status monitoring with convenient properties
- Thread-safe operation
- Context manager support for automatic cleanup

## Requirements
The FTDI D2XX driver is required for this code to work correctly. You will need to disable linux's
ftdi_sio module in order to use it. I am not sure why, and I have no intent of diagnosing it.

```bash
pip install pyftdi
```

## Quick Start

```python
from bbd203_controller import MotionController

# Connect using context manager (automatic cleanup)
with MotionController() as mc:
    # Print hardware information
    print(f"Model: {mc.get_model()}")
    print(f"Serial: {mc.get_serial_number()}")
    print(f"Firmware: {mc.get_firmware_version()}")

    # Enable and home X-axis
    mc.set_channel_enable_state(mc.DEST_X_AXIS, enabled=True)
    mc.home_x_axis(timeout=20.0)

    # Move to absolute position
    mc.set_velocity_params(mc.DEST_X_AXIS,
                          min_velocity=0.0,
                          acceleration=100.0,
                          max_velocity=50.0)
    mc.set_move_abs_params(mc.DEST_X_AXIS, absolute_position=25.0)
    result = mc.move_absolute(mc.DEST_X_AXIS, timeout=30.0)

    print(f"Final position: {result['position']:.3f} mm")
```

## Connection Management

### Basic Connection

```python
from bbd203_controller import MotionController

# Manual connection
mc = MotionController()
mc.connect()

# Use the controller...

mc.disconnect()
```

### Using Context Manager (Recommended)

```python
# Automatic connection and cleanup
with MotionController() as mc:
    # Use the controller...
    pass  # Automatically disconnects when exiting context
```

### Custom FTDI URL

```python
mc = MotionController(url='ftdi://0x0403:0xfaf0/1', baudrate=115200)
```

## Axis Control

### Enabling Axes

```python
# Enable X-axis
mc.set_channel_enable_state(mc.DEST_X_AXIS, enabled=True)

# Enable Y-axis
mc.set_channel_enable_state(mc.DEST_Y_AXIS, enabled=True)

# Check if enabled
is_enabled = mc.get_channel_enable_state(mc.DEST_X_AXIS)
```

### Homing

```python
# Home X-axis (blocks until complete)
if mc.home_x_axis(timeout=20.0):
    print(f"X-axis homed at position: {mc.position_x} mm")
else:
    print("Homing timed out")

# Home Y-axis
mc.home_y_axis(timeout=20.0)
```

After homing, the position automatically resets to 0 mm.

## Motion Control

### Setting Velocity Parameters

```python
# Set velocity parameters for X-axis
mc.set_velocity_params(
    mc.DEST_X_AXIS,
    min_velocity=0.0,      # mm/s
    acceleration=100.0,    # mm/s²
    max_velocity=50.0      # mm/s
)

# Get current velocity parameters
params = mc.get_velocity_params(mc.DEST_X_AXIS)
print(f"Max velocity: {params['max_velocity']:.2f} mm/s")
print(f"Acceleration: {params['acceleration']:.2f} mm/s²")
```

### Setting Acceleration Only

```python
# Change just the acceleration, preserving velocity settings
mc.set_acceleration(mc.DEST_X_AXIS, 75.0)

# Get just the acceleration value
accel = mc.get_acceleration(mc.DEST_X_AXIS)
```

### Absolute Moves

```python
# Move to absolute position
mc.set_move_abs_params(mc.DEST_X_AXIS, absolute_position=30.0)
result = mc.move_absolute(mc.DEST_X_AXIS, timeout=30.0)

if result:
    print(f"Moved to: {result['position']:.3f} mm")
    print(f"Status: 0x{result['status_bits']:08X}")
```

### Relative Moves

```python
# Move relative to current position
mc.set_move_rel_params(mc.DEST_X_AXIS, relative_distance=5.0)
result = mc.move_relative(mc.DEST_X_AXIS, timeout=30.0)

# Move backwards
mc.set_move_rel_params(mc.DEST_X_AXIS, relative_distance=-2.5)
mc.move_relative(mc.DEST_X_AXIS, timeout=30.0)
```

### Stopping Motion

```python
# Controlled stop (gradual deceleration)
mc.stop_x_axis(stop_mode=mc.StopMode.CONTROLLED, wait_for_stopped=True)

# Immediate stop
mc.stop_x_axis(stop_mode=mc.StopMode.IMMEDIATE, wait_for_stopped=True)

# Stop both axes simultaneously
results = mc.stop_all_axes(stop_mode=mc.StopMode.CONTROLLED)
```

## Position Tracking

### Reading Current Position

```python
# Query position from controller (blocking)
position = mc.get_position(mc.DEST_X_AXIS, timeout=5.0)
print(f"X position: {position:.3f} mm")

# Access cached position (non-blocking)
x_pos = mc.position_x
y_pos = mc.position_y

# Get encoder counts (raw values)
x_counts = mc.encoder_count_x
```

## Status Monitoring

### Using Status Properties

```python
# Check various status flags
print(f"X-axis enabled: {mc.is_enabled_x}")
print(f"X-axis homed: {mc.is_homed_x}")
print(f"X-axis in motion: {mc.is_in_motion_x}")
print(f"X-axis settled: {mc.is_settled_x}")
print(f"X-axis has errors: {mc.has_errors_x}")
print(f"Power OK: {mc.power_ok_x}")
```

### Decoding Status Bits

```python
result = mc.move_absolute(mc.DEST_X_AXIS, timeout=30.0)

if result:
    status_bits = result['status_bits']

    # Get human-readable description
    description = MotionController.get_status_description(status_bits)
    print(description)

    # Check for errors
    if MotionController.has_errors(status_bits):
        print("ERROR: Motion completed with errors!")

    # Check motion state
    if MotionController.is_settled(status_bits):
        print("Stage is settled at target position")
```

### Available Status Checks

- `is_enabled_x` / `is_enabled_y` - Motor output enabled
- `is_homed_x` / `is_homed_y` - Axis has been homed
- `is_homing_x` / `is_homing_y` - Currently homing
- `is_in_motion_x` / `is_in_motion_y` - Currently moving
- `is_settled_x` / `is_settled_y` - Settled at target
- `is_tracking_x` / `is_tracking_y` - Within tracking window
- `is_connected_x` / `is_connected_y` - Motor recognized
- `has_errors_x` / `has_errors_y` - Any error condition
- `power_ok_x` / `power_ok_y` - Power supply OK
- `is_active_x` / `is_active_y` - Executing motion command
- `at_cw_limit_x` / `at_cw_limit_y` - At clockwise limit
- `at_ccw_limit_x` / `at_ccw_limit_y` - At counter-clockwise limit

## Multi-Axis Operations

### Simultaneous Moves (Using Threading)

```python
import threading

def move_x():
    mc.set_move_abs_params(mc.DEST_X_AXIS, 50.0)
    mc.move_absolute(mc.DEST_X_AXIS, timeout=30.0)

def move_y():
    mc.set_move_abs_params(mc.DEST_Y_AXIS, 30.0)
    mc.move_absolute(mc.DEST_Y_AXIS, timeout=30.0)

# Start both moves in parallel
x_thread = threading.Thread(target=move_x)
y_thread = threading.Thread(target=move_y)

x_thread.start()
y_thread.start()

# Wait for both to complete
x_thread.join()
y_thread.join()

print(f"Final position: ({mc.position_x:.2f}, {mc.position_y:.2f}) mm")
```

## Hardware Information

```python
with MotionController() as mc:
    # Individual fields
    print(f"Serial Number: {mc.get_serial_number()}")
    print(f"Model: {mc.get_model()}")
    print(f"Firmware: {mc.get_firmware_version()}")
    print(f"Hardware Version: {mc.get_hw_version()}")
    print(f"Number of Channels: {mc.get_num_channels()}")

    # All info at once
    info = mc.get_hw_info()
    for key, value in info.items():
        print(f"{key}: {value}")
```

## Complete Examples

### Example 1: Simple Linear Move

```python
from bbd203_controller import MotionController
import time

with MotionController() as mc:
    # Enable and home X-axis
    mc.set_channel_enable_state(mc.DEST_X_AXIS, enabled=True)
    time.sleep(0.5)

    print("Homing X-axis...")
    mc.home_x_axis(timeout=20.0)
    print(f"Homed at {mc.position_x} mm")

    # Set velocity for smooth motion
    mc.set_velocity_params(mc.DEST_X_AXIS, 0.0, 50.0, 25.0)

    # Move to 40mm
    print("Moving to 40mm...")
    mc.set_move_abs_params(mc.DEST_X_AXIS, 40.0)
    result = mc.move_absolute(mc.DEST_X_AXIS, timeout=30.0)

    if result and not mc.has_errors_x:
        print(f"Successfully moved to {result['position']:.3f} mm")
    else:
        print("Move failed or has errors")
```

### Example 2: Square Pattern with Two Axes

```python
from bbd203_controller import MotionController
import threading
import time

def move_to_position(mc, x, y, label):
    """Move to (x, y) with both axes moving simultaneously."""
    print(f"Moving to {label}: ({x}, {y}) mm")

    # Set parameters for both axes
    mc.set_move_abs_params(mc.DEST_X_AXIS, x)
    mc.set_move_abs_params(mc.DEST_Y_AXIS, y)
    time.sleep(0.1)

    # Execute moves in parallel
    results = [None, None]

    def move_x():
        results[0] = mc.move_absolute(mc.DEST_X_AXIS, timeout=30.0)

    def move_y():
        results[1] = mc.move_absolute(mc.DEST_Y_AXIS, timeout=30.0)

    x_thread = threading.Thread(target=move_x)
    y_thread = threading.Thread(target=move_y)

    x_thread.start()
    y_thread.start()
    x_thread.join()
    y_thread.join()

    if results[0] and results[1]:
        print(f"  Reached ({results[0]['position']:.2f}, {results[1]['position']:.2f}) mm")
        return True
    return False

with MotionController() as mc:
    # Enable both axes
    mc.set_channel_enable_state(mc.DEST_X_AXIS, enabled=True)
    mc.set_channel_enable_state(mc.DEST_Y_AXIS, enabled=True)
    time.sleep(0.5)

    # Home both axes
    print("Homing axes...")
    mc.home_x_axis(timeout=20.0)
    mc.home_y_axis(timeout=20.0)

    # Set velocity for both axes
    velocity = 50.0
    acceleration = 100.0
    mc.set_velocity_params(mc.DEST_X_AXIS, 0.0, acceleration, velocity)
    mc.set_velocity_params(mc.DEST_Y_AXIS, 0.0, acceleration, velocity)

    # Define 20mm square centered at (55, 37.5)
    center_x, center_y = 55.0, 37.5
    half_size = 10.0

    waypoints = [
        (center_x - half_size, center_y - half_size, "Bottom Left"),
        (center_x + half_size, center_y - half_size, "Bottom Right"),
        (center_x + half_size, center_y + half_size, "Top Right"),
        (center_x - half_size, center_y + half_size, "Top Left"),
        (center_x, center_y, "Center"),
    ]

    # Execute square pattern
    for x, y, label in waypoints:
        if not move_to_position(mc, x, y, label):
            print(f"Failed at {label}")
            break
        time.sleep(0.5)

    print("Square pattern complete!")
```

### Example 3: Velocity Ramping Test

```python
from bbd203_controller import MotionController
import time

with MotionController() as mc:
    mc.set_channel_enable_state(mc.DEST_X_AXIS, enabled=True)
    time.sleep(0.5)
    mc.home_x_axis(timeout=20.0)

    # Test at different velocities
    test_velocities = [10.0, 25.0, 50.0, 100.0]
    move_distance = 20.0

    for velocity in test_velocities:
        print(f"\n--- Testing at {velocity} mm/s ---")

        # Set velocity parameters
        mc.set_velocity_params(mc.DEST_X_AXIS, 0.0, 100.0, velocity)

        # Move forward
        mc.set_move_abs_params(mc.DEST_X_AXIS, move_distance)
        start_time = time.time()
        result = mc.move_absolute(mc.DEST_X_AXIS, timeout=30.0)
        elapsed = time.time() - start_time

        if result:
            print(f"  Moved {move_distance}mm in {elapsed:.2f}s")
            print(f"  Average speed: {move_distance/elapsed:.2f} mm/s")

        time.sleep(0.5)

        # Move back to start
        mc.set_move_abs_params(mc.DEST_X_AXIS, 0.0)
        mc.move_absolute(mc.DEST_X_AXIS, timeout=30.0)
        time.sleep(0.5)
```

### Example 4: Position Monitoring During Move

```python
from bbd203_controller import MotionController
import threading
import time

with MotionController() as mc:
    mc.set_channel_enable_state(mc.DEST_X_AXIS, enabled=True)
    time.sleep(0.5)
    mc.home_x_axis(timeout=20.0)

    # Set slow velocity for visible monitoring
    mc.set_velocity_params(mc.DEST_X_AXIS, 0.0, 50.0, 10.0)

    # Start move in background thread
    move_complete = threading.Event()

    def do_move():
        mc.set_move_abs_params(mc.DEST_X_AXIS, 50.0)
        mc.move_absolute(mc.DEST_X_AXIS, timeout=60.0)
        move_complete.set()

    move_thread = threading.Thread(target=do_move)
    move_thread.start()

    # Monitor position while moving
    print("Position monitoring:")
    while not move_complete.is_set():
        # Request current position
        pos = mc.get_position(mc.DEST_X_AXIS, timeout=1.0)
        if pos is not None:
            print(f"  Current position: {pos:.3f} mm, "
                  f"In motion: {mc.is_in_motion_x}, "
                  f"Settled: {mc.is_settled_x}")
        time.sleep(0.5)

    move_thread.join()
    print(f"Move complete! Final position: {mc.position_x:.3f} mm")
```

## Constants and Enumerations

### Axis Destinations

```python
mc.DEST_CONTROLLER  # 0x11 - Controller/motherboard
mc.DEST_X_AXIS      # 0x21 - X-axis
mc.DEST_Y_AXIS      # 0x22 - Y-axis
```

### Stop Modes

```python
from bbd203_controller import StopMode

StopMode.IMMEDIATE   # 1 - Instant stop
StopMode.CONTROLLED  # 2 - Controlled deceleration (default)
```

### Jog Modes

```python
from bbd203_controller import JogMode

JogMode.CONTINUOUS   # 1 - Continuous jogging
JogMode.SINGLE_STEP  # 2 - Single step jogging
```

### Channel Enable States

```python
from bbd203_controller import ChannelEnableState

ChannelEnableState.DISABLED  # 0x02
ChannelEnableState.ENABLED   # 0x01
```

## Scaling Factors

The library handles all unit conversions automatically:

- **Position**: 20,000 encoder counts per mm
- **Velocity**: 13,421.77 counts per mm/s
- **Acceleration**: 13.744 counts per mm/s²

## Error Handling

```python
from bbd203_controller import MotionController

try:
    with MotionController() as mc:
        # Invalid axis destination
        mc.set_channel_enable_state(0x99, enabled=True)
except ValueError as e:
    print(f"ValueError: {e}")

try:
    with MotionController() as mc:
        mc.set_channel_enable_state(mc.DEST_X_AXIS, enabled=True)
        mc.home_x_axis(timeout=5.0)  # Too short timeout

        if not mc.is_homed_x:
            print("Homing failed - axis not homed")
except Exception as e:
    print(f"Error: {e}")
```

## Advanced Features

### Message Callbacks

```python
from bbd203_controller import MotionController, AptMessage, MsgId

def on_move_stopped(msg: AptMessage):
    print(f"Axis stopped unexpectedly!")
    print(f"Source: 0x{msg.source:02X}")

with MotionController() as mc:
    # Register callback for stop events
    mc.register_callback(MsgId.MOT_MOVE_STOPPED, on_move_stopped)

    # Your motion code here...

    # Unregister when done
    mc.unregister_callback(MsgId.MOT_MOVE_STOPPED, on_move_stopped)
```

### Direct Message Access

```python
# Wait for a specific message type
msg = mc.wait_for_message(MsgId.MOT_MOVE_COMPLETED, timeout=30.0)

# Get next message from queue
msg = mc.get_message(timeout=0.1)

# Get all queued messages
messages = mc.get_all_messages()
```

### Manual Connection Control

```python
mc = MotionController()

# Connect with updates disabled
mc.connect(enable_updates=False)

# Manually start/stop status updates
mc.start_update_messages()
# ...
mc.stop_update_messages()

mc.disconnect()
```

## Tips for Linear Scans

For performing linear scans at controlled speeds:

1. **Set velocity parameters** before each scan to ensure consistent motion
2. **Use absolute moves** with pre-calculated waypoints for accuracy
3. **For continuous scanning**: Execute moves sequentially without waiting
4. **For synchronized 2-axis moves**: Use threading (see examples above)
5. **Monitor position** during moves if needed for data acquisition timing

### Simple 1D Linear Scan

```python
with MotionController() as mc:
    mc.set_channel_enable_state(mc.DEST_X_AXIS, enabled=True)
    mc.home_x_axis(timeout=20.0)

    # Scan parameters
    start_pos = 10.0  # mm
    end_pos = 90.0    # mm
    step_size = 2.0   # mm
    scan_speed = 20.0 # mm/s

    # Set velocity for consistent speed
    mc.set_velocity_params(mc.DEST_X_AXIS, 0.0, 100.0, scan_speed)

    # Execute scan
    position = start_pos
    while position <= end_pos:
        mc.set_move_abs_params(mc.DEST_X_AXIS, position)
        result = mc.move_absolute(mc.DEST_X_AXIS, timeout=30.0)

        if result:
            # Acquire data at this position
            print(f"Scan point at {result['position']:.3f} mm")
            # Your data acquisition code here...

        position += step_size
```

## Troubleshooting

### Controller Not Responding

If the controller stops responding after many commands:
- The library automatically sends ACK messages every second
- This is handled internally and should not require user intervention

### Moves Timing Out

- Increase the `timeout` parameter on move commands
- Check that velocity and acceleration are set appropriately
- Ensure the axis is enabled and homed

### Position Inaccurate After Homing

- Position automatically resets to 0 mm after homing completes
- Always wait for homing to complete before issuing move commands
- Check `is_homed_x` / `is_homed_y` properties to verify

### Unexpected Stops

- Register a callback for `MsgId.MOT_MOVE_STOPPED` to detect stop events
- Check error flags in status bits
- Ensure no limit switches are being triggered

## License

MIT License

## Author

Generated for BBD202/BBD203 motion controller control via APT protocol.
