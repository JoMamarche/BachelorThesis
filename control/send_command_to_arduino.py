import serial
import time

def send_command_to_arduino():
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    time.sleep(1)
    ser.reset_input_buffer()
    print("Serial port opened")


    try:
        time.sleep(1)
        print('Sending s to arduino')
        ser.write("100s,".encode('utf-8')) 
    
    except KeyboardInterrupt:
        print("Close serial communication")
        ser.close()
