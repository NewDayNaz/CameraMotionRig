import time
import sys
import threading
from inputs import get_gamepad
import serial

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

ARDUINO_PORT = "/dev/ttyACM0"  # ESP32 USB serial port (may be /dev/ttyUSB0 on some systems)
ARDUINO_BAUDRATE = 115200

arduino = None
if ARDUINO_ENABLE_SERIAL:
    arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE, timeout=0.1)  # Add timeout for non-blocking reads


def send_cmd(cmd):
    """Send command to ESP32. Commands should end with newline."""
    if ARDUINO_ENABLE_SERIAL:
        # Ensure command ends with newline for proper parsing
        if not cmd.endswith(b'\n'):
            cmd = cmd + b'\n'
        arduino.write(cmd)

def send_joystick_values(yaw, pitch, zoom):
    """Send joystick values to ESP32 in format: j,yaw,pitch,zoom
    Values should be integers from -32768 to 32768
    ESP32 will scale these to appropriate velocity ranges"""
    if ARDUINO_ENABLE_SERIAL:
        # Format: j,yaw,pitch,zoom\n (newline required for command parsing)
        cmd = f"j,{int(yaw)},{int(pitch)},{int(zoom)}\n".encode('ascii')
        arduino.write(cmd)


def tell_cmd(msg):
    if ARDUINO_ENABLE_SERIAL:
        msg = msg
        x = msg.encode("ascii")  # encode n send
        arduino.write(x)


def serial_read_thread():
    """Thread function to continuously read and display messages from ESP32"""
    if not ARDUINO_ENABLE_SERIAL or arduino is None:
        return
    
    buffer = ""
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
            else:
                # Small sleep to prevent CPU spinning when no data
                time.sleep(0.01)
        except serial.SerialException as e:
            print(f"[Serial Error] {e}")
            time.sleep(1)  # Wait before retrying
        except Exception as e:
            print(f"[Error reading serial] {e}")
            time.sleep(1)
        except KeyboardInterrupt:
            break


if ARDUINO_ENABLE_SERIAL:
    print("Waiting for serial connection...")
    time.sleep(10)
    print("Connected! Sending joystick values to Arduino...")
    # Start serial read thread to receive debug messages from Arduino
    serial_thread = threading.Thread(target=serial_read_thread, daemon=True)
    serial_thread.start()
    print("Serial read thread started - Arduino debug messages will be displayed")

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
