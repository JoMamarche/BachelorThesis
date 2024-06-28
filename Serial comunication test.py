import serial
import time


ser = serial.Serial('COM6', 115200, timeout=1.0)
time.sleep(3)
ser.reset_input_buffer()
print("Serial OK")

try:
    while True:
        time.sleep(1)
        ser.write("red\n".encode('utf-8'))
        time.sleep(1)
        ser.write("blue\n".encode('utf-8'))
        time.sleep(1)
        ser.write("yellow\n".encode('utf-8'))
        time.sleep(1)
        ser.write("all\n".encode('utf-8'))
        time.sleep(1)
        ser.write("off\n".encode('utf-8'))
        time.sleep(1)
        
        
except KeyboardInterrupt:
    print("close serial comunication")
    ser.close()