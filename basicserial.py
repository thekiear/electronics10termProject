import serial

try:
    ser = serial.Serial(0)
    print ser.portstr()
    ser.close()
except:
    print "unable to open serial port"
