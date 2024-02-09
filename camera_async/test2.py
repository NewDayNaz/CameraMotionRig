serPort = '/dev/ttyACM0'
baudRate = 115200

from pyduinobridge import Bridge_py
myBridge = Bridge_py()
myBridge.begin(serPort, baudRate, numIntValues_FromPy=1, numFloatValues_FromPy=1)
myBridge.setSleepTime(2)

testData = []
testData.append("<LED1,200,0.5>")
testData.append("<LED2,800,1.0>")
testData.append("<LED1,1000,0.0>")
testData.append("<LED2,100,1.0>")
testData.append("<LED1,100,0.0>")
testData.append("<LED2,1000,1.0>")

# When using this function, the program sends a list of strings and receives a list of strings.
dataFromArduino = myBridge.writeAndRead_Strings(testData)
print(dataFromArduino)

# To hide the transmitted and received messages from the Python terminal:
myBridge.setVerbosity(0)

# When using this function, the program sends a header, a list of ints and a list of floats.
dataFromArduino2 = myBridge.writeAndRead_HeaderAndTwoLists("LED1", [800], [0.])
# The values received are
header_FromArdu, listInts_FromArdu, listFloats_FromArdu, millis_FromArdu = dataFromArduino2

myBridge.close()