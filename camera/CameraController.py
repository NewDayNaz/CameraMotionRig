import time
import sys
from inputs import get_gamepad 
import serial

ARDUINO_PITCH_MAX_SPEED = 10000
ARDUINO_YAW_MAX_SPEED = 1800 * 5

ARDUINO_PITCH_HALF_SPEED = ARDUINO_PITCH_MAX_SPEED * 5
ARDUINO_YAW_HALF_SPEED = ARDUINO_YAW_MAX_SPEED * 5

ARDUINO_PITCH_SPEED = 0
ARDUINO_YAW_SPEED = 0
ARDUINO_PITCH_SPEED_LAST = 0
ARDUINO_YAW_SPEED_LAST = 0

ARDUINO_PITCH_LAST = 0
ARDUINO_YAW_LAST = 0
ARDUINO_ZOOM_LAST = 0

ARDUINO_ENABLE_SERIAL = True

ARDUINO_PORT = "/dev/ttyACM0" #put your port here
ARDUINO_BAUDRATE = 115200

arduino = None
if ARDUINO_ENABLE_SERIAL:
    arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE)

def send_cmd(cmd):
    if ARDUINO_ENABLE_SERIAL:
        arduino.write(cmd)

def tell_cmd(msg):
    if ARDUINO_ENABLE_SERIAL:
        msg = msg
        x = msg.encode('ascii') # encode n send
        arduino.write(x)

if ARDUINO_ENABLE_SERIAL:
    print('Waiting for serial connection...')
    time.sleep(4)
    send_cmd(b'0')
    time.sleep(0.1)

    pitch_speed = str(ARDUINO_PITCH_MAX_SPEED).encode()
    yaw_speed = str(ARDUINO_YAW_MAX_SPEED).encode()

    send_cmd(b'p') # set pitch step speed (higher is slower)
    send_cmd(pitch_speed)
    send_cmd(b'y') # set yaw step speed
    send_cmd(yaw_speed)

JOY_MAX_VALUE = 32768
JOY_DEADZONE = JOY_MAX_VALUE * .06 # deadzone after 9% of max is reached
JOY_UPPER_RAMP = JOY_MAX_VALUE * .98 # jump to max after 95% of max is reached

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

                if abs(JOY_RY) <= 0.7:
                    ARDUINO_PITCH_SPEED = ARDUINO_PITCH_HALF_SPEED
                else:
                    ARDUINO_PITCH_SPEED = ARDUINO_PITCH_MAX_SPEED

                if abs(JOY_X) <= 0.7:
                    ARDUINO_YAW_SPEED = ARDUINO_YAW_HALF_SPEED
                else:
                    ARDUINO_YAW_SPEED = ARDUINO_YAW_MAX_SPEED

                if JOY_BUMPER_R > 0:
                    ARDUINO_PITCH_SPEED = round(ARDUINO_PITCH_SPEED / 2)
                if JOY_BUMPER_L > 0:
                    ARDUINO_YAW_SPEED = round(ARDUINO_YAW_SPEED / 2)

                

                if ARDUINO_PITCH_SPEED != ARDUINO_PITCH_SPEED_LAST:
                    ARDUINO_PITCH_SPEED_LAST = ARDUINO_PITCH_SPEED
                    send_cmd(b'p') # set pitch step speed (higher is slower)
                    send_cmd(str(ARDUINO_PITCH_SPEED).encode())

                if ARDUINO_YAW_SPEED != ARDUINO_YAW_SPEED_LAST:
                    ARDUINO_YAW_SPEED_LAST = ARDUINO_YAW_SPEED
                    send_cmd(b'y') # set yaw step speed
                    send_cmd(str(ARDUINO_YAW_SPEED).encode())

                ARDUINO_ZOOM_SPEED = 2000

                ZOOM_MOVE = 0
                if JOY_TRIGGER_L > 0:
                    ZOOM_MOVE = 1
                elif JOY_TRIGGER_R > 0:
                    ZOOM_MOVE = 2

                if ZOOM_MOVE != ARDUINO_ZOOM_LAST:
                    ARDUINO_ZOOM_LAST = ZOOM_MOVE
                    if ZOOM_MOVE == 1:
                        send_cmd(b'4')
                    elif ZOOM_MOVE == 2:
                        send_cmd(b'5')
                    else:
                        send_cmd(b'6')
                        send_cmd(b'6')

                # send_cmd(b'p') # set pitch step speed (higher is slower)
                # send_cmd(pitch_speed)
                # send_cmd(b'y') # set yaw step speed
                # send_cmd(yaw_speed)
                # print("Joy | Pitch:", ARDUINO_PITCH_MAX_SPEED * JOY_RY, "| Yaw:", ARDUINO_YAW_MAX_SPEED * JOY_X)

                PITCH_MOVE = 0
                if JOY_RY == 0:
                    PITCH_MOVE = 0
                    # send_cmd(b'c')
                if JOY_RY > 0:
                    PITCH_MOVE = 1
                    # send_cmd(("a" + ARDUINO_PITCH_SPEED + ";").encode())
                    # send_cmd(b'a')
                if JOY_RY < 0:
                    PITCH_MOVE = 2
                    # send_cmd(("b" + ARDUINO_PITCH_SPEED + ";").encode())
                    # send_cmd(b'b')

                YAW_MOVE = 0
                if JOY_X == 0:
                    YAW_MOVE = 0
                    # send_cmd(b'3')
                if JOY_X > 0:
                    YAW_MOVE = 1 
                    # send_cmd(b'1')
                    # send_cmd(("1" + ARDUINO_YAW_SPEED + ";").encode())
                if JOY_X < 0:
                    YAW_MOVE = 2
                    # send_cmd(b'2')
                    # send_cmd(("2" + ARDUINO_YAW_SPEED + ";").encode())

                if ARDUINO_PITCH_LAST != PITCH_MOVE:
                    ARDUINO_PITCH_LAST = PITCH_MOVE
                    if PITCH_MOVE == 1:
                        send_cmd(b'a')
                    elif PITCH_MOVE == 2:
                        send_cmd(b'b')
                    else:
                        send_cmd(b'c')
                        send_cmd(b'c')

                if ARDUINO_YAW_LAST != YAW_MOVE:
                    ARDUINO_YAW_LAST = YAW_MOVE
                    if YAW_MOVE == 1:
                        send_cmd(b'1')
                    elif YAW_MOVE == 2:
                        send_cmd(b'2')
                    else:
                        send_cmd(b'3')
                        send_cmd(b'3')

                # j,pitch,yaw,pitchSpeed,yawSpeed,zoom,zoomSpeed
                # tell_cmd("j,{pitch},{yaw},{zoom},{pitch_speed}\n".format(pitch = PITCH_MOVE, yaw = YAW_MOVE, zoom = ZOOM_MOVE, pitch_speed = ARDUINO_PITCH_SPEED, yaw_speed = ARDUINO_YAW_SPEED, zoom_speed = ARDUINO_ZOOM_SPEED))
                print("Joy | X:", JOY_X, "| Y:", JOY_Y, "| RX:", JOY_RX, "| RY:", JOY_RY, "| Pitch:", ARDUINO_PITCH_SPEED, "| Yaw:", ARDUINO_YAW_SPEED, "| Zoom:", ZOOM_MOVE)
                # print(event.ev_type, event.code, event.state)

    except KeyboardInterrupt:
        sys.exit(0)

print("done")
