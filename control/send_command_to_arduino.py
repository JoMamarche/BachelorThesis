import serial
import time

def send_command_to_arduino(command, port='dev/ttyACM0', baud_rate=115200):

    try:
        with serial.Serial(port, baud_rate, timeout=1) as ser:
            ser.write(command.encode())
            print(f'Sent command: {command}')
    