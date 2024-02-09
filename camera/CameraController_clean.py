import time
import sys
from inputs import get_gamepad 
import serial

ARDUINO_PITCH_MAX_SPEED = 1800
ARDUINO_YAW_MAX_SPEED = 1600

ARDUINO_PITCH_HALF_SPEED = ARDUINO_PITCH_MAX_SPEED * 4
ARDUINO_YAW_HALF_SPEED = ARDUINO_YAW_MAX_SPEED * 4

ARDUINO_PITCH_SPEED = 0
ARDUINO_YAW_SPEED = 0
ARDUINO_PITCH_SPEED_LAST = 0
ARDUINO_YAW_SPEED_LAST = 0

ARDUINO_ZOOM_LAST = 0

ARDUINO_ENABLE_SERIAL = True

ARDUINO_PORT = "/dev/ttyACM0" #put your port here
ARDUINO_BAUDRATE = 115200

arduino = None
if ARDUINO_ENABLE_SERIAL:
    arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE)

def send_cmd(cmd):
    return cmd
    # if ARDUINO_ENABLE_SERIAL:
    #     arduino.write(cmd)

def tell_cmd(msg):
    if ARDUINO_ENABLE_SERIAL:
        msg = msg
        x = msg.encode('ascii') # encode n send
        arduino.write(x)

if ARDUINO_ENABLE_SERIAL:
    print('Waiting for serial connection...')
    time.sleep(5)

    pitch_speed = str(ARDUINO_PITCH_MAX_SPEED).encode()
    yaw_speed = str(ARDUINO_YAW_MAX_SPEED).encode()

    send_cmd(b'p') # set pitch step speed (higher is slower)
    send_cmd(pitch_speed)
    send_cmd(b'y') # set yaw step speed
    send_cmd(yaw_speed)

JOY_MAX_VALUE = 32768
JOY_DEADZONE = 1400 # deadzone after 9% of max is reached
JOY_UPPER_RAMP = JOY_MAX_VALUE * .95 # jump to max after 95% of max is reached

JOY_X_LEFT = False
JOY_X_RIGHT = False
JOY_Y_FORWARD = False
JOY_Y_BACKWARD = False

JOY_RX_LEFT = False
JOY_RX_RIGHT = False
JOY_RY_FORWARD = False
JOY_RY_BACKWARD = False

JOY_X = 0
JOY_Y = 0
JOY_RX = 0
JOY_RY = 0

JOY_BUMPER_L = 0
JOY_BUMPER_R = 0

JOY_TRIGGER_L = 0
JOY_TRIGGER_R = 0

# add zoom via triggers

while True:
    try:
        events = get_gamepad() #waits until a new event
        for event in events:
            # print(event.ev_type, event.code, event.state)
            if event.ev_type == "Key":
                if event.code == "BTN_TL":
                    JOY_BUMPER_L = event.state
                if event.code == "BTN_TR":
                    JOY_BUMPER_R = event.state
            if event.ev_type == "Absolute":
                if event.code == "ABS_Z":
                    if event.state < 100:
                        JOY_TRIGGER_L = 0
                    elif event.state >= 100:
                        JOY_TRIGGER_L = event.state / 255
                if event.code == "ABS_RZ":
                    if event.state < 100:
                        JOY_TRIGGER_R = 0
                    elif event.state >= 100:
                        JOY_TRIGGER_R = event.state / 255
                if event.code == "ABS_X":
                    if event.state < JOY_DEADZONE and event.state > -JOY_DEADZONE:
                        JOY_X_LEFT = False
                        JOY_X_RIGHT = False
                        JOY_X = 0
                    elif event.state < -JOY_DEADZONE:
                        JOY_X_LEFT = True
                        JOY_X_RIGHT = False
                        JOY_X = event.state / JOY_MAX_VALUE
                    elif event.state > JOY_DEADZONE:
                        JOY_X_LEFT = False
                        JOY_X_RIGHT = True
                        JOY_X = event.state / JOY_MAX_VALUE

                if event.code == "ABS_Y":
                    if event.state < JOY_DEADZONE and event.state > -JOY_DEADZONE:
                        JOY_Y_FORWARD = False
                        JOY_Y_BACKWARD = False
                        JOY_Y = 0
                    elif event.state < -JOY_DEADZONE:
                        JOY_Y_FORWARD = True
                        JOY_Y_BACKWARD = False
                        JOY_Y = event.state / JOY_MAX_VALUE
                    elif event.state > JOY_DEADZONE:
                        JOY_Y_FORWARD = False
                        JOY_Y_BACKWARD = True
                        JOY_Y = event.state / JOY_MAX_VALUE

                if event.code == "ABS_RX":
                    if event.state < JOY_DEADZONE and event.state > -JOY_DEADZONE:
                        JOY_RX_LEFT = False
                        JOY_RX_RIGHT = False
                        JOY_RX = 0
                    elif event.state < -JOY_DEADZONE:
                        JOY_RX_LEFT = True
                        JOY_RX_RIGHT = False
                        JOY_RX = event.state / JOY_MAX_VALUE
                    elif event.state > JOY_DEADZONE:
                        JOY_RX_LEFT = False
                        JOY_RX_RIGHT = True
                        JOY_RX = event.state / JOY_MAX_VALUE

                if event.code == "ABS_RY":
                    if event.state < JOY_DEADZONE and event.state > -JOY_DEADZONE:
                        JOY_RY_FORWARD = False
                        JOY_RY_BACKWARD = False
                        JOY_RY = 0
                    elif event.state < -JOY_DEADZONE:
                        JOY_RY_FORWARD = True
                        JOY_RY_BACKWARD = False
                        JOY_RY = event.state / JOY_MAX_VALUE
                    elif event.state > JOY_DEADZONE:
                        JOY_RY_FORWARD = False
                        JOY_RY_BACKWARD = True
                        JOY_RY = event.state / JOY_MAX_VALUE

                ARDUINO_ZOOM = 0
                if JOY_TRIGGER_L > 0:
                    ARDUINO_ZOOM = -JOY_TRIGGER_L
                elif JOY_TRIGGER_R > 0:
                    ARDUINO_ZOOM = JOY_TRIGGER_R

            tell_cmd("j,{x},{y},{z}\n".format(x = JOY_X, y = JOY_RY, z = ARDUINO_ZOOM, lb = JOY_BUMPER_L, rb = JOY_BUMPER_R))

            print("Joy | X:", JOY_X, "| Y:", JOY_Y, "| RX:", JOY_RX, "| RY:", JOY_RY, "| Zoom:", ARDUINO_ZOOM, "| LB:", JOY_BUMPER_L, "| RB:", JOY_BUMPER_R)
            # print(event.ev_type, event.code, event.state)

    except KeyboardInterrupt:
        sys.exit(0)

print("done")
