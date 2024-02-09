import serial
import time
port = "/dev/ttyACM0" #put your port here
baudrate = 115200
ser = serial.Serial(port, baudrate)
def tell(msg):
    msg = msg
    x = msg.encode('ascii') # encode n send
    ser.write(x)

def hear():
    msg = ser.read_until() # read until a new line
    mystring = msg.decode('ascii')  # decode n return 
    return mystring

print('Waiting for serial connection..')
time.sleep(5)

while True:
    val = input().strip() # take user input
    # tell(val + ' ') # send it to arduino
    ser.write(b'b ')
    var = input() # listen to arduino
    print(var) #print what arduino sent
    ser.write(b'c ')