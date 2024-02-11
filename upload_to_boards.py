import time
import sys
import threading 
import serial
import subprocess
import re

ARDUINO_CLI = "/home/pi/arduino-cli"
ARDUINO_BAUDRATE = 115200

regex = r"^(\/dev\/tty.*?) .* (arduino:avr:uno)"

cmd_ret = subprocess.check_output([ARDUINO_CLI, 'board', 'list', '--timeout', '5s'])
cmd_ret = cmd_ret.decode('utf-8').split('\n')
serial_ports = []

for line in cmd_ret:
    matches = re.finditer(regex, line, re.MULTILINE)
    for matchNum, match in enumerate(matches, start=1):
        groups = match.groups()
        if len(groups) == 2:
            print("Detected arduino board connected to %s" % (groups[0]))
            serial_ports.append(groups[0])

arduino_modules = []
for port in serial_ports:
    arduino = serial.Serial(port, ARDUINO_BAUDRATE)
    time.sleep(1)
    arduino.write(bytes('info', 'ascii') + b' ')
    ser_read = arduino.readline().decode('utf-8').split("\r\n")
    arduino = None
    time.sleep(1)
    arduino_modules.append((port, ser_read[0]))
    print("Arduino board on %s is the %s" % (port, ser_read[0]))

for arduino in arduino_modules:
    port = arduino[0]
    module = arduino[1]

    code_module = ""
    if module == "main_module":
        code_module = "camera_async"
    elif module == "zoom_module":
        code_module = "camera_zoom_async"

    print("Uploading %s to %s..." % (code_module, port))

    compile_builder = [
        ARDUINO_CLI, 'compile', code_module,
        '--fqbn', 'arduino:avr:uno',
        '-p', port
    ]
    
    compile_cmd = subprocess.check_output(compile_builder)
    print(compile_cmd.decode('utf-8'))

    upload_builder = [
        ARDUINO_CLI, 'upload', code_module,
        '--fqbn', 'arduino:avr:uno',
        '-p', port
    ]
    upload_cmd = subprocess.check_output(upload_builder)
    print(upload_cmd.decode('utf-8'))

print("Done")