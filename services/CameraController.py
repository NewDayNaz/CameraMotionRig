import time
import sys
import os
import threading
from inputs import get_gamepad
import serial
import serial.tools.list_ports

# Joystick values (normalized -1.0 to 1.0, will be converted to -32768 to 32768 for Arduino)
joystick_yaw = 0.0      # X-axis (pan)
joystick_pitch = 0.0    # Y-axis (tilt) 
joystick_zoom = 0.0     # Combined trigger value (negative = zoom out, positive = zoom in)

joystick_yaw_last = 0.0
joystick_pitch_last = 0.0
joystick_zoom_last = 0.0

# Smoothed values (for filtering)
joystick_yaw_smoothed = 0.0
joystick_pitch_smoothed = 0.0
joystick_zoom_smoothed = 0.0

JOYSTICK_MAX_INT = 32768  # Maximum integer value for Arduino

# Smoothing factor (0.0 = no smoothing, 1.0 = no change)
# Lower values = more smoothing (slower response)
JOYSTICK_SMOOTHING = 0.3  # 30% of new value, 70% of old value

# Rate limiting for sending updates
last_send_time = 0.0
MIN_SEND_INTERVAL = 0.05  # Send at most 20 times per second (50ms between sends)

# Threshold for sending updates (larger = less frequent sends)
JOYSTICK_SEND_THRESHOLD = 15  # Only send if change is > 1.5% of full range

# Joystick curve exponent for non-linear response
# Values > 1.0 make small movements less sensitive (more control at low deflection)
# 2.0 = quadratic curve (gentle at start, full range at end)
# 3.0 = cubic curve (even more gentle at start)
JOYSTICK_CURVE_EXPONENT = 2.5  # Quadratic-like curve for better low-end control
JOYSTICK_CURVE_EXPONENT_ZOOM = 1.5  # Less aggressive curve for zoom (triggers need more linear response)

# Deadzone thresholds - values within this range are treated as zero
# Prevents drift and unwanted movement when joystick is centered
# Can be tuned per-axis for different sensitivity requirements
JOYSTICK_DEADZONE_YAW = 1000    # ~6% of full range (32768) for pan axis
JOYSTICK_DEADZONE_PITCH = 1000  # ~6% of full range (32768) for tilt axis
JOYSTICK_DEADZONE_ZOOM = 10    # ~1.5% of full range (32768) for zoom axis (smaller for more sensitive control)

# Independent axis scaling factors (0.0 to 1.0)
# Allows fine-tuning sensitivity per axis
# Lower values = less responsive, higher values = more responsive
JOYSTICK_SCALE_YAW = 1.0    # Pan axis (full sensitivity)
JOYSTICK_SCALE_PITCH = 0.9  # Tilt axis (reduced sensitivity for smoother control)
JOYSTICK_SCALE_ZOOM = 1.0   # Zoom axis (increased sensitivity - triggers need more range)

def apply_joystick_curve(value, max_value, deadzone):
    """
    Apply a non-linear curve to joystick input for better sensitivity control.
    Small movements are reduced, full deflection still reaches maximum.
    Includes deadzone to prevent drift.
    
    Args:
        value: Raw joystick value (-max_value to max_value)
        max_value: Maximum joystick value (32768)
        deadzone: Deadzone threshold for this axis
    
    Returns:
        Curved value with same sign and range, or 0.0 if in deadzone
    """
    # Apply deadzone - treat small values as zero
    if abs(value) < deadzone:
        return 0.0
    
    # Normalize to -1.0 to 1.0
    normalized = value / max_value
    
    # Apply curve (preserve sign)
    sign = 1.0 if normalized >= 0 else -1.0
    abs_normalized = abs(normalized)
    
    # Power curve: small values get reduced more
    curved = sign * (abs_normalized ** JOYSTICK_CURVE_EXPONENT)
    
    # Scale back to original range
    return curved * max_value

arduino_back_last = 0
arduino_start_last = 0
arduino_bumper_l_last = 0
arduino_bumper_r_last = 0

arduino_selected_pos = 1

ARDUINO_ENABLE_SERIAL = True

ARDUINO_BAUDRATE = 115200

# Cache file to store the detected port
PORT_CACHE_FILE = os.path.join(os.path.dirname(__file__), ".esp32_port_cache")

def load_cached_port():
    """Load the cached port from file if it exists."""
    if os.path.exists(PORT_CACHE_FILE):
        try:
            with open(PORT_CACHE_FILE, 'r') as f:
                cached_port = f.read().strip()
                if cached_port:
                    return cached_port
        except Exception as e:
            print(f"Warning: Could not read port cache: {e}")
    return None

def save_cached_port(port):
    """Save the detected port to cache file."""
    try:
        with open(PORT_CACHE_FILE, 'w') as f:
            f.write(port)
    except Exception as e:
        print(f"Warning: Could not save port cache: {e}")

def clear_cached_port():
    """Clear the cached port."""
    try:
        if os.path.exists(PORT_CACHE_FILE):
            os.remove(PORT_CACHE_FILE)
    except Exception as e:
        print(f"Warning: Could not clear port cache: {e}")

def test_port(port_path, max_test_time=2.0):
    """
    Test if a specific port responds to STATUS command.
    Returns True if port is valid ESP32, False otherwise.
    
    Args:
        port_path: Path to serial port to test
        max_test_time: Maximum time to spend testing this port (seconds)
    """
    test_start = time.time()
    try:
        # Try to open port with short timeout
        test_serial = serial.Serial(port_path, ARDUINO_BAUDRATE, timeout=0.2)
        
        # Give port minimal time to initialize
        time.sleep(0.05)
        
        # Check if we've exceeded max test time
        if time.time() - test_start >= max_test_time:
            test_serial.close()
            return False
        
        # Clear any existing data in buffer
        try:
            test_serial.reset_input_buffer()
        except:
            pass
        
        # Send STATUS command (read-only, doesn't change anything)
        test_serial.write(b"STATUS\n")
        test_serial.flush()
        
        # Wait for response (up to max_test_time total, but shorter per-iteration)
        response = b""
        response_start = time.time()
        max_response_time = min(1.0, max_test_time - (time.time() - test_start))
        
        while (time.time() - response_start) < max_response_time:
            # Check overall timeout
            if time.time() - test_start >= max_test_time:
                break
                
            if test_serial.in_waiting > 0:
                response += test_serial.read(test_serial.in_waiting)
                # Check if we have a complete line
                if b'\n' in response:
                    break
            time.sleep(0.05)  # Slightly longer sleep to reduce CPU usage
        
        test_serial.close()
        
        # Check if response matches expected format: STATUS:PAN:... TILT:... ZOOM:...
        if response:
            response_str = response.decode('ascii', errors='ignore').strip()
            if response_str.startswith("STATUS:PAN:") and "TILT:" in response_str and "ZOOM:" in response_str:
                return True
        return False
        
    except (serial.SerialException, OSError, ValueError) as e:
        # Port doesn't exist, is busy, or invalid - skip it quickly
        return False
    except Exception:
        return False

def detect_esp32_port(force_rescan=False, timeout_seconds=15):
    """
    Automatically detect the ESP32 serial port by sending a STATUS command
    and checking for the expected response format.
    
    First checks cached port if available, then scans all ports if needed.
    Returns the port path if found, None otherwise.
    
    Args:
        force_rescan: If True, skip cached port and rescan all ports
        timeout_seconds: Maximum time to spend searching (default 15 seconds)
    """
    start_time = time.time()
    
    # Check cached port first (unless forced to rescan)
    if not force_rescan:
        cached_port = load_cached_port()
        if cached_port:
            elapsed = time.time() - start_time
            if elapsed >= timeout_seconds:
                print(f"\nTimeout: Could not find ESP32 within {timeout_seconds} seconds")
                return None
                
            print(f"Trying cached port: {cached_port}...", end=" ", flush=True)
            if test_port(cached_port, max_test_time=2.0):
                print("✓ Cached port is valid!")
                return cached_port
            else:
                print("✗ Cached port failed, clearing cache and rescanning...")
                clear_cached_port()
    
    # Get list of available serial ports
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("No serial ports found")
        return None
    
    print(f"Scanning {len(ports)} serial port(s) for ESP32...")
    
    # Calculate max time per port (leave some buffer for overhead)
    ports_remaining = len(ports)
    for port_info in ports:
        # Check timeout before each port test
        elapsed = time.time() - start_time
        if elapsed >= timeout_seconds:
            print(f"\nTimeout: Could not find ESP32 within {timeout_seconds} seconds")
            return None
        
        # Calculate remaining time and allocate per port
        remaining_time = timeout_seconds - elapsed
        # Reserve 0.5 seconds for overhead, divide rest among remaining ports
        max_time_per_port = max(1.0, (remaining_time - 0.5) / ports_remaining)
        ports_remaining -= 1
        
        port_path = port_info.device
        print(f"  Trying {port_path}...", end=" ", flush=True)
        
        port_test_start = time.time()
        if test_port(port_path, max_test_time=max_time_per_port):
            print(f"✓ Found ESP32!")
            # Save to cache for next time
            save_cached_port(port_path)
            return port_path
        else:
            elapsed_test = time.time() - port_test_start
            print(f"✗ ({elapsed_test:.1f}s)")
    
    print("ESP32 not found on any serial port")
    return None

# Auto-detect ESP32 port
ARDUINO_PORT = None
arduino = None

if ARDUINO_ENABLE_SERIAL:
    print("Searching for ESP32 device (15 second timeout)...")
    ARDUINO_PORT = detect_esp32_port(timeout_seconds=15)
    if ARDUINO_PORT:
        print(f"Connecting to ESP32 on {ARDUINO_PORT}...")
        try:
            arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE, timeout=0.1)
            print("Connected!")
        except Exception as e:
            print(f"ERROR: Failed to connect to {ARDUINO_PORT}: {e}")
            print("Exiting...")
            sys.exit(1)
    else:
        print("ERROR: Could not detect ESP32 within timeout period.")
        print("Please ensure the ESP32 is connected and try again.")
        print("Exiting...")
        sys.exit(1)


def send_cmd(cmd):
    """Send command to ESP32. Commands should end with newline."""
    if ARDUINO_ENABLE_SERIAL and arduino is not None:
        try:
            # Ensure command ends with newline for proper parsing
            if not cmd.endswith(b'\n'):
                cmd = cmd + b'\n'
            arduino.write(cmd)
        except (serial.SerialException, AttributeError) as e:
            print(f"[Serial] Error sending command: {e}")
            # Attempt to reconnect
            reconnect_serial()

def send_joystick_values(yaw, pitch, zoom):
    """Send joystick values to ESP32 in format: j,yaw,pitch,zoom
    Values should be integers from -32768 to 32768
    ESP32 will scale these to appropriate velocity ranges"""
    if ARDUINO_ENABLE_SERIAL and arduino is not None:
        try:
            # Format: j,yaw,pitch,zoom\n (newline required for command parsing)
            cmd = f"j,{int(yaw)},{int(pitch)},{int(zoom)}\n".encode('ascii')
            arduino.write(cmd)
        except (serial.SerialException, AttributeError) as e:
            print(f"[Serial] Error sending joystick values: {e}")
            # Attempt to reconnect
            reconnect_serial()


def tell_cmd(msg):
    if ARDUINO_ENABLE_SERIAL and arduino is not None:
        try:
            x = msg.encode("ascii")  # encode n send
            arduino.write(x)
        except (serial.SerialException, AttributeError) as e:
            print(f"[Serial] Error sending message: {e}")
            # Attempt to reconnect
            reconnect_serial()


def reconnect_serial():
    """Attempt to reconnect to ESP32, clearing cache if needed."""
    global arduino, ARDUINO_PORT, ARDUINO_ENABLE_SERIAL
    
    if arduino is not None:
        try:
            arduino.close()
        except:
            pass
        arduino = None
    
    # Clear cache and rescan (with shorter timeout for reconnection)
    clear_cached_port()
    print("Attempting to reconnect to ESP32...")
    ARDUINO_PORT = detect_esp32_port(force_rescan=True, timeout_seconds=10)
    
    if ARDUINO_PORT:
        try:
            arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE, timeout=0.1)
            print(f"Reconnected to ESP32 on {ARDUINO_PORT}")
            return True
        except Exception as e:
            print(f"Failed to reconnect: {e}")
            ARDUINO_ENABLE_SERIAL = False
            return False
    else:
        print("Could not reconnect to ESP32. Serial communication disabled.")
        ARDUINO_ENABLE_SERIAL = False
        return False

def serial_read_thread():
    """Thread function to continuously read and display messages from ESP32"""
    if not ARDUINO_ENABLE_SERIAL or arduino is None:
        return
    
    buffer = ""
    consecutive_errors = 0
    max_errors = 3
    
    while True:
        try:
            if arduino.in_waiting > 0:
                # Read available data
                data = arduino.read(arduino.in_waiting).decode('ascii', errors='ignore')
                buffer += data
                
                # Process complete lines (ending with \n)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:  # Only print non-empty lines
                        print(f"[ESP32] {line}")
                
                # Reset error counter on successful read
                consecutive_errors = 0
            else:
                # Small sleep to prevent CPU spinning when no data
                time.sleep(0.01)
        except serial.SerialException as e:
            consecutive_errors += 1
            print(f"[Serial Error] {e}")
            if consecutive_errors >= max_errors:
                print("[Serial] Multiple errors detected, attempting to reconnect...")
                if reconnect_serial():
                    consecutive_errors = 0
                else:
                    print("[Serial] Reconnection failed. Serial communication disabled.")
                    break
            time.sleep(1)  # Wait before retrying
        except Exception as e:
            consecutive_errors += 1
            print(f"[Error reading serial] {e}")
            if consecutive_errors >= max_errors:
                print("[Serial] Multiple errors detected, attempting to reconnect...")
                if reconnect_serial():
                    consecutive_errors = 0
                else:
                    print("[Serial] Reconnection failed. Serial communication disabled.")
                    break
            time.sleep(1)
        except KeyboardInterrupt:
            break


if ARDUINO_ENABLE_SERIAL and arduino is not None:
    # Start serial read thread to receive debug messages from ESP32
    serial_thread = threading.Thread(target=serial_read_thread, daemon=True)
    serial_thread.start()
    print("Serial read thread started - ESP32 debug messages will be displayed")
    print("Ready! Sending joystick values to ESP32...")

JOY_MAX_VALUE = 32768
JOY_DEADZONE = JOY_MAX_VALUE * 0.06  # deadzone after 9% of max is reached
JOY_UPPER_RAMP = JOY_MAX_VALUE * 0.98  # jump to max after 95% of max is reached

joy_x_left = False
joy_x_right = False
joy_y_forward = False
joy_y_backward = False

joy_rx_left = False
joy_rx_right = False
joy_ry_forward = False
joy_ry_backward = False

joy_x = 0
joy_y = 0
joy_rx = 0
joy_ry = 0

joy_dp_u = 0
joy_dp_d = 0
joy_dp_l = 0
joy_dp_r = 0

joy_back = 0
joy_start = 0

joy_bumper_l = 0
joy_bumper_r = 0

joy_trigger_l = 0
joy_trigger_r = 0

# add zoom via triggers
while True:
    try:
        events = get_gamepad()  # waits until a new event
        for event in events:
            print(event.ev_type, event.code, event.state)
            if event.ev_type == "Key":
                if event.code == "BTN_TL":
                    joy_bumper_l = event.state
                if event.code == "BTN_TR":
                    joy_bumper_r = event.state
                if event.code == "BTN_SELECT":
                    joy_back = event.state
                if event.code == "BTN_START":
                    joy_start = event.state
                if event.code == "BTN_SOUTH":
                    arduino_selected_pos = 1
                if event.code == "BTN_EAST":
                    arduino_selected_pos = 2
                if event.code == "BTN_NORTH":
                    arduino_selected_pos = 3
                if event.code == "BTN_WEST":
                    arduino_selected_pos = 4

                if joy_back != arduino_back_last:
                    arduino_back_last = joy_back
                    # Home all axes
                    send_cmd(b"HOME\n")

                if joy_start != arduino_start_last:
                    arduino_start_last = joy_start
                    # Stop all motion
                    send_cmd(b"STOP\n")

                if joy_bumper_r != arduino_bumper_r_last:
                    arduino_bumper_r_last = joy_bumper_r
                    print("save position")
                    # Save current position as preset
                    preset_idx = arduino_selected_pos
                    send_cmd(f"SAVE {preset_idx}\n".encode('ascii'))
                    
                if joy_bumper_l != arduino_bumper_l_last:
                    arduino_bumper_l_last = joy_bumper_l
                    print("goto saved position")
                    # Move to preset
                    preset_idx = arduino_selected_pos
                    send_cmd(f"GOTO {preset_idx}\n".encode('ascii'))

                # Speed control is now handled by Arduino based on joystick values

            if event.ev_type == "Absolute":
                if event.code == "ABS_HAT0Y":
                    if event.state == -1:
                        joy_dp_u = 1
                    if event.state == 0:
                        joy_dp_u = 0
                        joy_dp_d = 0
                    if event.state == 1:
                        joy_dp_d = 1
                if event.code == "ABS_HAT0X":
                    if event.state == -1:
                        joy_dp_l = 1
                    if event.state == 0:
                        joy_dp_l = 0
                        joy_dp_r = 0
                    if event.state == 1:
                        joy_dp_r = 1
                if event.code == "ABS_Z":
                    # Remove hard threshold - use full range for smoother control
                    # Apply minimal deadzone to handle trigger noise
                    if event.state < 5:
                        joy_trigger_l = 0
                    else:
                        joy_trigger_l = event.state / 255
                if event.code == "ABS_RZ":
                    # Remove hard threshold - use full range for smoother control
                    # Apply minimal deadzone to handle trigger noise
                    if event.state < 5:
                        joy_trigger_r = 0
                    else:
                        joy_trigger_r = event.state / 255
                if event.code == "ABS_X":
                    if event.state < JOY_DEADZONE and event.state > -JOY_DEADZONE:
                        joy_x_left = False
                        joy_x_right = False
                        joy_x = 0
                    elif event.state < -JOY_DEADZONE:
                        joy_x_left = True
                        joy_x_right = False
                        joy_x = event.state / JOY_MAX_VALUE
                    elif event.state > JOY_DEADZONE:
                        joy_x_left = False
                        joy_x_right = True
                        joy_x = event.state / JOY_MAX_VALUE

                if event.code == "ABS_Y":
                    if event.state < JOY_DEADZONE and event.state > -JOY_DEADZONE:
                        joy_y_forward = False
                        joy_y_backward = False
                        joy_y = 0
                    elif event.state < -JOY_DEADZONE:
                        joy_y_forward = True
                        joy_y_backward = False
                        joy_y = event.state / JOY_MAX_VALUE
                    elif event.state > JOY_DEADZONE:
                        joy_y_forward = False
                        joy_y_backward = True
                        joy_y = event.state / JOY_MAX_VALUE

                if event.code == "ABS_RX":
                    if event.state < JOY_DEADZONE and event.state > -JOY_DEADZONE:
                        joy_rx_left = False
                        joy_rx_right = False
                        joy_rx = 0
                    elif event.state < -JOY_DEADZONE:
                        joy_rx_left = True
                        joy_rx_right = False
                        joy_rx = event.state / JOY_MAX_VALUE
                    elif event.state > JOY_DEADZONE:
                        joy_rx_left = False
                        joy_rx_right = True
                        joy_rx = event.state / JOY_MAX_VALUE

                if event.code == "ABS_RY":
                    if event.state < JOY_DEADZONE and event.state > -JOY_DEADZONE:
                        joy_ry_forward = False
                        joy_ry_backward = False
                        joy_ry = 0
                    elif event.state < -JOY_DEADZONE:
                        joy_ry_forward = True
                        joy_ry_backward = False
                        joy_ry = event.state / JOY_MAX_VALUE
                    elif event.state > JOY_DEADZONE:
                        joy_ry_forward = False
                        joy_ry_backward = True
                        joy_ry = event.state / JOY_MAX_VALUE

                # Calculate joystick values for Arduino
                # Use right stick (RX, RY) for camera control, left stick (X, Y) for movement if needed
                # Right stick Y (RY) = Pitch (tilt)
                # Right stick X (RX) = Yaw (pan) - but we're using left stick X (joy_x) for yaw
                # Actually, let's use: joy_x for yaw, joy_ry for pitch
                
                # Convert normalized joystick values (-1.0 to 1.0) to integer range (-32768 to 32768)
                joystick_yaw_raw = joy_x * JOYSTICK_MAX_INT
                joystick_pitch_raw = joy_ry * JOYSTICK_MAX_INT
                
                # Combine triggers for zoom (left trigger = zoom out, right trigger = zoom in)
                # Negative value = zoom out, positive = zoom in
                # Process zoom like web UI: linear, no curve, no smoothing (ESP32 handles smoothing)
                # Web UI: velZ = normalized * 50 (steps/sec), sent directly
                # Joystick: we send -32768..32768, ESP32 scales to (value / 32768) * MAX_VEL_ZOOM
                # ESP32 MAX_VEL_ZOOM = 50, so max zoom via joystick is 50 steps/sec
                # To match web UI behavior: linear scaling, no curve, no smoothing
                trigger_diff = joy_trigger_r - joy_trigger_l
                
                # Apply deadzone (minimal for triggers)
                zoom_deadzone_normalized = JOYSTICK_DEADZONE_ZOOM / JOYSTICK_MAX_INT
                if abs(trigger_diff) < zoom_deadzone_normalized:
                    joystick_zoom_raw_scaled = 0.0
                else:
                    # Linear scaling to full joystick range (like web UI does linear scaling)
                    # This gives smooth, proportional response matching web UI behavior
                    joystick_zoom_raw_scaled = trigger_diff * JOYSTICK_MAX_INT
                
                # Apply non-linear curve to reduce sensitivity at small deflections (pan/tilt only)
                joystick_yaw_curved = apply_joystick_curve(joystick_yaw_raw, JOYSTICK_MAX_INT, JOYSTICK_DEADZONE_YAW)
                joystick_pitch_curved = apply_joystick_curve(joystick_pitch_raw, JOYSTICK_MAX_INT, JOYSTICK_DEADZONE_PITCH)
                
                # Apply exponential smoothing to reduce jerkiness
                # Pan/tilt: heavier smoothing for stability
                # Reset smoothed values to zero when in deadzone to prevent drift
                if abs(joystick_yaw_curved) < JOYSTICK_DEADZONE_YAW:
                    joystick_yaw_smoothed = 0.0  # Reset to prevent drift
                else:
                    joystick_yaw_smoothed = (JOYSTICK_SMOOTHING * joystick_yaw_curved + 
                                            (1.0 - JOYSTICK_SMOOTHING) * joystick_yaw_smoothed)
                
                if abs(joystick_pitch_curved) < JOYSTICK_DEADZONE_PITCH:
                    joystick_pitch_smoothed = 0.0  # Reset to prevent drift
                else:
                    joystick_pitch_smoothed = (JOYSTICK_SMOOTHING * joystick_pitch_curved + 
                                              (1.0 - JOYSTICK_SMOOTHING) * joystick_pitch_smoothed)
                
                # Zoom: light smoothing (0.6 = 60% new, 40% old) to prevent jitter while maintaining responsiveness
                # Reset smoothed values to zero when in deadzone to prevent drift
                zoom_smoothing = 0.6  # Lighter smoothing for zoom to match web UI feel
                if abs(joystick_zoom_raw_scaled) < JOYSTICK_DEADZONE_ZOOM:
                    joystick_zoom_smoothed = 0.0  # Reset to prevent drift
                else:
                    joystick_zoom_smoothed = (zoom_smoothing * joystick_zoom_raw_scaled + 
                                             (1.0 - zoom_smoothing) * joystick_zoom_smoothed)
                
                # Apply independent axis scaling for fine-tuning sensitivity
                joystick_yaw_scaled = joystick_yaw_smoothed * JOYSTICK_SCALE_YAW
                joystick_pitch_scaled = joystick_pitch_smoothed * JOYSTICK_SCALE_PITCH
                joystick_zoom_scaled = joystick_zoom_smoothed  # Zoom already scaled, use smoothed value
                
                # Detect when axes enter deadzone (transition from non-zero to zero)
                # We need to send explicit zero commands to stop drift
                # Use scaled values for deadzone detection (apply scaling factor to deadzone thresholds)
                yaw_deadzone_scaled = JOYSTICK_DEADZONE_YAW * JOYSTICK_SCALE_YAW
                pitch_deadzone_scaled = JOYSTICK_DEADZONE_PITCH * JOYSTICK_SCALE_PITCH
                # Zoom uses direct deadzone (no additional scaling since it's already linear)
                zoom_deadzone_scaled = JOYSTICK_DEADZONE_ZOOM
                
                yaw_in_deadzone = abs(joystick_yaw_scaled) < yaw_deadzone_scaled
                pitch_in_deadzone = abs(joystick_pitch_scaled) < pitch_deadzone_scaled
                zoom_in_deadzone = abs(joystick_zoom_scaled) < zoom_deadzone_scaled
                
                yaw_entered_deadzone = not (abs(joystick_yaw_last) < yaw_deadzone_scaled) and yaw_in_deadzone
                pitch_entered_deadzone = not (abs(joystick_pitch_last) < pitch_deadzone_scaled) and pitch_in_deadzone
                zoom_entered_deadzone = not (abs(joystick_zoom_last) < zoom_deadzone_scaled) and zoom_in_deadzone
                
                # Rate limiting: only send if enough time has passed
                current_time = time.time()
                time_since_last_send = current_time - last_send_time
                
                # Send if:
                # 1. Enough time has passed AND values changed significantly, OR
                # 2. Any axis entered deadzone (need to send zero to stop drift)
                # For zoom, use lower threshold to match web UI smoothness
                should_send = False
                if time_since_last_send >= MIN_SEND_INTERVAL:
                    # Use lower threshold for zoom to prevent jitter (match web UI behavior)
                    zoom_threshold = 5  # Much lower threshold for zoom to ensure smooth updates
                    if (abs(joystick_yaw_scaled - joystick_yaw_last) > JOYSTICK_SEND_THRESHOLD or
                        abs(joystick_pitch_scaled - joystick_pitch_last) > JOYSTICK_SEND_THRESHOLD or
                        abs(joystick_zoom_scaled - joystick_zoom_last) > zoom_threshold):
                        should_send = True
                
                # Always send when entering deadzone to stop drift (bypass rate limiting)
                if yaw_entered_deadzone or pitch_entered_deadzone or zoom_entered_deadzone:
                    should_send = True
                    # Force zero for axes that entered deadzone
                    if yaw_entered_deadzone:
                        joystick_yaw_scaled = 0.0
                    if pitch_entered_deadzone:
                        joystick_pitch_scaled = 0.0
                    if zoom_entered_deadzone:
                        joystick_zoom_scaled = 0.0
                
                if should_send:
                    send_joystick_values(int(joystick_yaw_scaled), 
                                         int(joystick_pitch_scaled), 
                                         int(joystick_zoom_scaled))
                    joystick_yaw_last = joystick_yaw_scaled
                    joystick_pitch_last = joystick_pitch_scaled
                    joystick_zoom_last = joystick_zoom_scaled
                    last_send_time = current_time
                    
                    print(
                        "Joy | Yaw:",
                        f"{joystick_yaw_scaled:.0f}",
                        "| Pitch:",
                        f"{joystick_pitch_scaled:.0f}",
                        "| Zoom:",
                        f"{joystick_zoom_scaled:.0f}",
                    )

    except KeyboardInterrupt:
        sys.exit(0)

print("done")
