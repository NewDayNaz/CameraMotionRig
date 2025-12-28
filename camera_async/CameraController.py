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

JOYSTICK_MAX_INT = 32768  # Maximum integer value for Arduino

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

def test_port(port_path):
    """
    Test if a specific port responds to STATUS command.
    Returns True if port is valid ESP32, False otherwise.
    """
    try:
        test_serial = serial.Serial(port_path, ARDUINO_BAUDRATE, timeout=0.5)
        time.sleep(0.1)  # Give port time to initialize
        
        # Clear any existing data in buffer
        test_serial.reset_input_buffer()
        
        # Send STATUS command (read-only, doesn't change anything)
        test_serial.write(b"STATUS\n")
        test_serial.flush()
        
        # Wait for response (up to 1 second)
        response = b""
        start_time = time.time()
        while time.time() - start_time < 1.0:
            if test_serial.in_waiting > 0:
                response += test_serial.read(test_serial.in_waiting)
                # Check if we have a complete line
                if b'\n' in response:
                    break
            time.sleep(0.01)
        
        test_serial.close()
        
        # Check if response matches expected format: STATUS:PAN:... TILT:... ZOOM:...
        response_str = response.decode('ascii', errors='ignore').strip()
        if response_str.startswith("STATUS:PAN:") and "TILT:" in response_str and "ZOOM:" in response_str:
            return True
        return False
        
    except Exception:
        return False

def detect_esp32_port(force_rescan=False):
    """
    Automatically detect the ESP32 serial port by sending a STATUS command
    and checking for the expected response format.
    
    First checks cached port if available, then scans all ports if needed.
    Returns the port path if found, None otherwise.
    """
    # Check cached port first (unless forced to rescan)
    if not force_rescan:
        cached_port = load_cached_port()
        if cached_port:
            print(f"Trying cached port: {cached_port}...", end=" ")
            if test_port(cached_port):
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
    
    for port_info in ports:
        port_path = port_info.device
        print(f"  Trying {port_path}...", end=" ")
        
        if test_port(port_path):
            print(f"✓ Found ESP32!")
            # Save to cache for next time
            save_cached_port(port_path)
            return port_path
        else:
            print(f"✗")
    
    print("ESP32 not found on any serial port")
    return None

# Auto-detect ESP32 port
ARDUINO_PORT = None
arduino = None

if ARDUINO_ENABLE_SERIAL:
    ARDUINO_PORT = detect_esp32_port()
    if ARDUINO_PORT:
        print(f"Connecting to ESP32 on {ARDUINO_PORT}...")
        arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE, timeout=0.1)
        print("Connected!")
    else:
        print("WARNING: Could not detect ESP32. Serial communication disabled.")
        ARDUINO_ENABLE_SERIAL = False


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
    
    # Clear cache and rescan
    clear_cached_port()
    ARDUINO_PORT = detect_esp32_port(force_rescan=True)
    
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


def thread_function(name):
    last_cmd = ""
    cur_cmd = ""

    while True:
        with open("/home/pi/camera_async/send_cmd.txt", "r") as f:
            cur_cmd = f.read().strip()
            if cur_cmd != last_cmd and cur_cmd != "":
                last_cmd = cur_cmd
                print("send", cur_cmd)
                send_cmd(bytes(cur_cmd, "ascii"))
        time.sleep(0.05)


x = threading.Thread(target=thread_function, args=(1,))
x.start()

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
                    # Save current position as preset (0-indexed, so subtract 1)
                    preset_idx = arduino_selected_pos - 1
                    send_cmd(f"SAVE {preset_idx}\n".encode('ascii'))
                    
                if joy_bumper_l != arduino_bumper_l_last:
                    arduino_bumper_l_last = joy_bumper_l
                    print("goto saved position")
                    # Move to preset (0-indexed, so subtract 1)
                    preset_idx = arduino_selected_pos - 1
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
                    if event.state < 100:
                        joy_trigger_l = 0
                    elif event.state >= 100:
                        joy_trigger_l = event.state / 255
                if event.code == "ABS_RZ":
                    if event.state < 100:
                        joy_trigger_r = 0
                    elif event.state >= 100:
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
                joystick_yaw = joy_x * JOYSTICK_MAX_INT
                joystick_pitch = joy_ry * JOYSTICK_MAX_INT
                
                # Combine triggers for zoom (left trigger = zoom out, right trigger = zoom in)
                # Negative value = zoom out, positive = zoom in
                joystick_zoom = (joy_trigger_r - joy_trigger_l) * JOYSTICK_MAX_INT
                
                # Send joystick values if they've changed (with small threshold to reduce spam)
                threshold = 100  # Only send if change is significant
                if (abs(joystick_yaw - joystick_yaw_last) > threshold or
                    abs(joystick_pitch - joystick_pitch_last) > threshold or
                    abs(joystick_zoom - joystick_zoom_last) > threshold):
                    
                    send_joystick_values(joystick_yaw, joystick_pitch, joystick_zoom)
                    joystick_yaw_last = joystick_yaw
                    joystick_pitch_last = joystick_pitch
                    joystick_zoom_last = joystick_zoom
                    
                    print(
                        "Joy | Yaw:",
                        f"{joystick_yaw:.0f}",
                        "| Pitch:",
                        f"{joystick_pitch:.0f}",
                        "| Zoom:",
                        f"{joystick_zoom:.0f}",
                    )

    except KeyboardInterrupt:
        sys.exit(0)

print("done")
