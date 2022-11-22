import serial
from pynmeagps import NMEAReader

#test script to verify that the computer receives the correct string containing GPS data.
ser = serial.Serial('/dev/rfcomm0', 9600)

try:
    nmr = NMEAReader(ser)
    while True:
        (raw_data, msg) = nmr.read()
        print(f"lat: {msg.lat} - lon: {msg.lon}")
except KeyboardInterrupt:
    ser.close()