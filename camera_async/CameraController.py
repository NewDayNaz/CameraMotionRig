import time
import sys
import threading
from inputs import get_gamepad
import serial


class ArduinoController:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.arduino = serial.Serial(port, baudrate)

    def send_cmd(self, cmd):
        self.arduino.write(cmd + b" ")

    def tell_cmd(self, msg):
        msg = msg.encode("ascii")
        self.arduino.write(msg)

class JoystickController:
    def __init__(self):
        self.initialize_joystick_values()
        self.initialize_arduino_values()

    def initialize_joystick_values(self):
        self.JOY_MAX_VALUE = 32768
        self.JOY_DEADZONE = self.JOY_MAX_VALUE * 0.06
        self.JOY_UPPER_RAMP = self.JOY_MAX_VALUE * 0.98

        self.JOY_X_LEFT = False
        self.JOY_X_RIGHT = False
        self.JOY_Y_FORWARD = False
        self.JOY_Y_BACKWARD = False

        self.JOY_RX_LEFT = False
        self.JOY_RX_RIGHT = False
        self.JOY_RY_FORWARD = False
        self.JOY_RY_BACKWARD = False

        self.JOY_X = 0
        self.JOY_Y = 0
        self.JOY_RX = 0
        self.JOY_RY = 0

        self.JOY_DP_U = 0
        self.JOY_DP_D = 0
        self.JOY_DP_L = 0
        self.JOY_DP_R = 0

        self.JOY_BACK = 0
        self.JOY_START = 0

        self.JOY_BUMPER_L = 0
        self.JOY_BUMPER_R = 0

        self.JOY_TRIGGER_L = 0
        self.JOY_TRIGGER_R = 0

    def initialize_arduino_values(self):
        self.ARDUINO_PITCH_MAX_SPEED = 10000 * 1.3
        self.ARDUINO_YAW_MAX_SPEED = 1800 * 4.8

        self.ARDUINO_PITCH_HALF_SPEED = self.ARDUINO_PITCH_MAX_SPEED * 3
        self.ARDUINO_YAW_HALF_SPEED = self.ARDUINO_YAW_MAX_SPEED * 3

        self.ARDUINO_PITCH_SPEED = 0
        self.ARDUINO_YAW_SPEED = 0
        self.ARDUINO_PITCH_SPEED_LAST = 0
        self.ARDUINO_YAW_SPEED_LAST = 0

        self.ARDUINO_PITCH_LAST = 0
        self.ARDUINO_YAW_LAST = 0
        self.ARDUINO_ZOOM_LAST = 0

        self.ARDUINO_BACK_LAST = 0
        self.ARDUINO_START_LAST = 0
        self.ARDUINO_BUMPER_L_LAST = 0
        self.ARDUINO_BUMPER_R_LAST = 0

        self.ARDUINO_SELECTED_POS = 1

    def handle_joystick_events(self, arduino_controller, arduino_zoom_controller, events):
        for event in events:
            self.process_key_events(event, arduino_controller, arduino_zoom_controller)
            self.process_absolute_events(event, arduino_controller, arduino_zoom_controller)

    def process_key_events(self, event, arduino_controller, arduino_zoom_controller):
        if event.ev_type == "Key":
            if event.code == "BTN_TL":
                self.JOY_BUMPER_L = event.state
            if event.code == "BTN_TR":
                self.JOY_BUMPER_R = event.state
            if event.code == "BTN_SELECT":
                self.JOY_BACK = event.state
            if event.code == "BTN_START":
                self.JOY_START = event.state
            if event.code == "BTN_SOUTH":
                self.ARDUINO_SELECTED_POS = 1
            if event.code == "BTN_EAST":
                self.ARDUINO_SELECTED_POS = 2
            if event.code == "BTN_NORTH":
                self.ARDUINO_SELECTED_POS = 3
            if event.code == "BTN_WEST":
                self.ARDUINO_SELECTED_POS = 4

            self.handle_back_button(arduino_controller, arduino_zoom_controller)
            self.handle_start_button(arduino_controller, arduino_zoom_controller)
            self.handle_bumper_r_button(arduino_controller, arduino_zoom_controller)
            self.handle_bumper_l_button(arduino_controller, arduino_zoom_controller)
            self.handle_pitch_speed_change(arduino_controller, arduino_zoom_controller)
            self.handle_yaw_speed_change(arduino_controller, arduino_zoom_controller)

    def process_absolute_events(self, event, arduino_controller, arduino_zoom_controller):
        if event.ev_type == "Absolute":
            self.handle_hat_events(event)
            self.handle_trigger_events(event)
            self.handle_joystick_events(event)
            self.handle_pitch_movement(arduino_controller, arduino_zoom_controller)
            self.handle_yaw_movement(arduino_controller, arduino_zoom_controller)
            self.handle_zoom_movement(arduino_controller, arduino_zoom_controller)

    def handle_back_button(self, arduino_controller, arduino_zoom_controller):
        if self.JOY_BACK != self.ARDUINO_BACK_LAST:
            self.ARDUINO_BACK_LAST = self.JOY_BACK
            arduino_controller.send_cmd(b"ea")
            arduino_zoom_controller.send_cmd(b"ea")

    def handle_start_button(self, arduino_controller, arduino_zoom_controller):
        if self.JOY_START != self.ARDUINO_START_LAST:
            self.ARDUINO_START_LAST = self.JOY_START
            arduino_controller.send_cmd(b"eb")
            arduino_zoom_controller.send_cmd(b"eb")

    def handle_bumper_r_button(self, arduino_controller, arduino_zoom_controller):
        if self.JOY_BUMPER_R != self.ARDUINO_BUMPER_R_LAST:
            self.ARDUINO_BUMPER_R_LAST = self.JOY_BUMPER_R
            print("save position")
            if self.ARDUINO_SELECTED_POS == 1:
                arduino_controller.send_cmd(b"s")
                arduino_zoom_controller.send_cmd(b"s")
            if self.ARDUINO_SELECTED_POS == 2:
                arduino_controller.send_cmd(b"s2")
                arduino_zoom_controller.send_cmd(b"s2")
            if self.ARDUINO_SELECTED_POS == 3:
                arduino_controller.send_cmd(b"s3")
                arduino_zoom_controller.send_cmd(b"s3")
            if self.ARDUINO_SELECTED_POS == 4:
                arduino_controller.send_cmd(b"s4")
                arduino_zoom_controller.send_cmd(b"s4")

    def handle_bumper_l_button(self, arduino_controller, arduino_zoom_controller):
        if self.JOY_BUMPER_L != self.ARDUINO_BUMPER_L_LAST:
            self.ARDUINO_BUMPER_L_LAST = self.JOY_BUMPER_L
            print("goto saved position")
            if self.ARDUINO_SELECTED_POS == 1:
                arduino_controller.send_cmd(b"t")
                arduino_zoom_controller.send_cmd(b"t")
            if self.ARDUINO_SELECTED_POS == 2:
                arduino_controller.send_cmd(b"t2")
                arduino_zoom_controller.send_cmd(b"t2")
            if self.ARDUINO_SELECTED_POS == 3:
                arduino_controller.send_cmd(b"t3")
                arduino_zoom_controller.send_cmd(b"t3")
            if self.ARDUINO_SELECTED_POS == 4:
                arduino_controller.send_cmd(b"t4")
                arduino_zoom_controller.send_cmd(b"t4")

    def handle_pitch_speed_change(self, arduino_controller, arduino_zoom_controller):
        if self.ARDUINO_PITCH_SPEED != self.ARDUINO_PITCH_SPEED_LAST:
            self.ARDUINO_PITCH_SPEED_LAST = self.ARDUINO_PITCH_SPEED
            arduino_controller.send_cmd(b"p")
            arduino_controller.send_cmd(str(self.ARDUINO_PITCH_SPEED).encode())
            arduino_zoom_controller.send_cmd(b"p")
            arduino_zoom_controller.send_cmd(str(self.ARDUINO_PITCH_SPEED).encode())

    def handle_yaw_speed_change(self, arduino_controller, arduino_zoom_controller):
        if self.ARDUINO_YAW_SPEED != self.ARDUINO_YAW_SPEED_LAST:
            self.ARDUINO_YAW_SPEED_LAST = self.ARDUINO_YAW_SPEED
            arduino_controller.send_cmd(b"y")
            arduino_controller.send_cmd(str(self.ARDUINO_YAW_SPEED).encode())
            arduino_zoom_controller.send_cmd(b"y")
            arduino_zoom_controller.send_cmd(str(self.ARDUINO_YAW_SPEED).encode())

    def handle_hat_events(self, event):
        if event.code == "ABS_HAT0Y":
            if event.state == -1:
                self.JOY_DP_U = 1
            if event.state == 0:
                self.JOY_DP_U = 0
                self.JOY_DP_D = 0
            if event.state == 1:
                self.JOY_DP_D = 1
        if event.code == "ABS_HAT0X":
            if event.state == -1:
                self.JOY_DP_L = 1
            if event.state == 0:
                self.JOY_DP_L = 0
                self.JOY_DP_R = 0
            if event.state == 1:
                self.JOY_DP_R = 1

    def handle_trigger_events(self, event):
        if event.code == "ABS_Z":
            if event.state < 100:
                self.JOY_TRIGGER_L = 0
            elif event.state >= 100:
                self.JOY_TRIGGER_L = event.state / 255
        if event.code == "ABS_RZ":
            if event.state < 100:
                self.JOY_TRIGGER_R = 0
            elif event.state >= 100:
                self.JOY_TRIGGER_R = event.state / 255

    def handle_joystick_events(self, event):
        if event.code == "ABS_X":
            if event.state < self.JOY_DEADZONE and event.state > -self.JOY_DEADZONE:
                self.JOY_X_LEFT = False
                self.JOY_X_RIGHT = False
                self.JOY_X = 0
            elif event.state < -self.JOY_DEADZONE:
                self.JOY_X_LEFT = True
                self.JOY_X_RIGHT = False
                self.JOY_X = event.state / self.JOY_MAX_VALUE
            elif event.state > self.JOY_DEADZONE:
                self.JOY_X_LEFT = False
                self.JOY_X_RIGHT = True
                self.JOY_X = event.state / self.JOY_MAX_VALUE

        if event.code == "ABS_Y":
            if event.state < self.JOY_DEADZONE and event.state > -self.JOY_DEADZONE:
                self.JOY_Y_FORWARD = False
                self.JOY_Y_BACKWARD = False
                self.JOY_Y = 0
            elif event.state < -self.JOY_DEADZONE:
                self.JOY_Y_FORWARD = True
                self.JOY_Y_BACKWARD = False
                self.JOY_Y = event.state / self.JOY_MAX_VALUE
            elif event.state > self.JOY_DEADZONE:
                self.JOY_Y_FORWARD = False
                self.JOY_Y_BACKWARD = True
                self.JOY_Y = event.state / self.JOY_MAX_VALUE

        if event.code == "ABS_RX":
            if event.state < self.JOY_DEADZONE and event.state > -self.JOY_DEADZONE:
                self.JOY_RX_LEFT = False
                self.JOY_RX_RIGHT = False
                self.JOY_RX = 0
            elif event.state < -self.JOY_DEADZONE:
                self.JOY_RX_LEFT = True
                self.JOY_RX_RIGHT = False
                self.JOY_RX = event.state / self.JOY_MAX_VALUE
            elif event.state > self.JOY_DEADZONE:
                self.JOY_RX_LEFT = False
                self.JOY_RX_RIGHT = True
                self.JOY_RX = event.state / self.JOY_MAX_VALUE

        if event.code == "ABS_RY":
            if event.state < self.JOY_DEADZONE and event.state > -self.JOY_DEADZONE:
                self.JOY_RY_FORWARD = False
                self.JOY_RY_BACKWARD = False
                self.JOY_RY = 0
            elif event.state < -self.JOY_DEADZONE:
                self.JOY_RY_FORWARD = True
                self.JOY_RY_BACKWARD = False
                self.JOY_RY = event.state / self.JOY_MAX_VALUE
            elif event.state > self.JOY_DEADZONE:
                self.JOY_RY_FORWARD = False
                self.JOY_RY_BACKWARD = True
                self.JOY_RY = event.state / self.JOY_MAX_VALUE

    def handle_pitch_movement(self, arduino_controller, arduino_zoom_controller):
        if abs(self.JOY_RY) <= 0.7:
            self.ARDUINO_PITCH_SPEED = self.ARDUINO_PITCH_HALF_SPEED
        else:
            self.ARDUINO_PITCH_SPEED = self.ARDUINO_PITCH_MAX_SPEED

        pitch_move = 0
        if self.JOY_RY == 0:
            pitch_move = 0
        if self.JOY_RY > 0:
            pitch_move = 1
        if self.JOY_RY < 0:
            pitch_move = 2

        if self.ARDUINO_PITCH_LAST != pitch_move:
            self.ARDUINO_PITCH_LAST = pitch_move
            if pitch_move == 1:
                arduino_controller.send_cmd(b"a")
                arduino_zoom_controller.send_cmd(b"a")
            elif pitch_move == 2:
                arduino_controller.send_cmd(b"b")
                arduino_zoom_controller.send_cmd(b"b")
            else:
                arduino_controller.send_cmd(b"c")
                arduino_controller.send_cmd(b"c")
                arduino_zoom_controller.send_cmd(b"c")
                arduino_zoom_controller.send_cmd(b"c")

    def handle_yaw_movement(self, arduino_controller, arduino_zoom_controller):
        if abs(self.JOY_X) <= 0.7:
            self.ARDUINO_YAW_SPEED = self.ARDUINO_YAW_HALF_SPEED
        else:
            self.ARDUINO_YAW_SPEED = self.ARDUINO_YAW_MAX_SPEED

        yaw_move = 0
        if self.JOY_X == 0:
            yaw_move = 0
        if self.JOY_X > 0:
            yaw_move = 1
        if self.JOY_X < 0:
            yaw_move = 2

        if self.ARDUINO_YAW_LAST != yaw_move:
            self.ARDUINO_YAW_LAST = yaw_move
            if yaw_move == 1:
                arduino_controller.send_cmd(b"1")
                arduino_zoom_controller.send_cmd(b"1")
            elif yaw_move == 2:
                arduino_controller.send_cmd(b"2")
                arduino_zoom_controller.send_cmd(b"2")
            else:
                arduino_controller.send_cmd(b"3")
                arduino_controller.send_cmd(b"3")
                arduino_zoom_controller.send_cmd(b"3")
                arduino_zoom_controller.send_cmd(b"3")

    def handle_zoom_movement(self, arduino_controller, arduino_zoom_controller):
        zoom_move = 0
        if self.JOY_TRIGGER_L > 0:
            zoom_move = 1
        elif self.JOY_TRIGGER_R > 0:
            zoom_move = 2

        if self.ARDUINO_ZOOM_LAST != zoom_move:
            self.ARDUINO_ZOOM_LAST = zoom_move
            if zoom_move == 1:
                arduino_controller.send_cmd(b"4")
                arduino_zoom_controller.send_cmd(b"4")
            elif zoom_move == 2:
                arduino_controller.send_cmd(b"5")
                arduino_zoom_controller.send_cmd(b"5")
            else:
                arduino_controller.send_cmd(b"6")
                arduino_controller.send_cmd(b"6")
                arduino_zoom_controller.send_cmd(b"6")
                arduino_zoom_controller.send_cmd(b"6")

    # Add other event handling methods here, sending commands to both controllers
    def run_joystick_loop(self, arduino_controller, arduino_zoom_controller):
        while True:
            try:
                events = get_gamepad()
                self.handle_joystick_events(arduino_controller, arduino_zoom_controller, events)
            except KeyboardInterrupt:
                sys.exit(0)


def command_server_thread_func(filepath, arduino_controller, arduino_zoom_controller):
    LAST_CMD = ""
    CUR_CMD = ""

    while True:
        with open(filepath, "r") as f:
            CUR_CMD = f.read().strip()
            if CUR_CMD != LAST_CMD and CUR_CMD != "":
                LAST_CMD = CUR_CMD
                print("send", CUR_CMD)
                arduino_controller.send_cmd(bytes(CUR_CMD, "ascii"))
                arduino_zoom_controller.send_cmd(bytes(CUR_CMD, "ascii"))
        time.sleep(0.05)


def main():
    ARDUINO_ENABLE_SERIAL = True
    ARDUINO_PORT = "/dev/ttyACM0"
    ARDUINO_BAUDRATE = 115200
    ARDUINO_ZOOM_PORT = "/dev/ttyACM1"

    arduino_controller = ArduinoController(ARDUINO_PORT, ARDUINO_BAUDRATE)
    arduino_zoom_controller = ArduinoController(ARDUINO_ZOOM_PORT, ARDUINO_BAUDRATE)

    if ARDUINO_ENABLE_SERIAL:
        print("Waiting for serial connection...")
        time.sleep(10)
        arduino_controller.send_cmd(b"0")
        arduino_zoom_controller.send_cmd(b"0")
        time.sleep(0.1)

        pitch_speed = str(arduino_controller.ARDUINO_PITCH_MAX_SPEED).encode()
        yaw_speed = str(arduino_controller.ARDUINO_YAW_MAX_SPEED).encode()

        arduino_controller.send_cmd(b"p")
        arduino_controller.send_cmd(pitch_speed)
        arduino_controller.send_cmd(b"y")
        arduino_controller.send_cmd(yaw_speed)
        
        arduino_zoom_controller.send_cmd(b"p")
        arduino_zoom_controller.send_cmd(pitch_speed)
        arduino_zoom_controller.send_cmd(b"y")
        arduino_zoom_controller.send_cmd(yaw_speed)

    joystick_controller = JoystickController()

    x = threading.Thread(
        target=command_server_thread_func,
        args=(
            "/home/pi/camera_async/send_cmd.txt",
            arduino_controller,
            arduino_zoom_controller,
        ),
    )
    x.start()

    joystick_controller.run_joystick_loop(arduino_controller)


if __name__ == "__main__":
    main()
