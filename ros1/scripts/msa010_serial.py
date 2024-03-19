import serial
import numpy as np
import cv2
import time
import json
import struct

'''
default settings:
ISP Response: [b'+ISP=1\r\n', b'OK\r\n']
BINN Response: [b'+BINN=1\r\n', b'OK\r\n']
DISP Response: [b'+DISP=1\r\n', b'OK\r\n']
BAUD Response: [b'+BAUD=2\r\n', b'OK\r\n']
UNIT Response: [b'+UNIT= 0\r\n', b'OK\r\n']
FPS Response: [b'+FPS=15\r\n', b'OK\r\n']
ANTIMMI Response: [b'+ANTIMMI=15\r\n', b'OK\r\n']
'''

BAUD = {0: 9600, 1: 57600, 2: 115200, 3: 230400, 4: 460800, 5: 921600, 6: 1000000, 7: 2000000, 8: 3000000}


def intrinsicParam(ser):
    command = "AT+COEFF?\r"
    ser.write(command.encode("ASCII"))
    response = ser.readlines()
    print("COEFF Response:", response)
    response_string = b''.join(response).decode()
    response_string = response_string.strip()
    response_string = '\n'.join(response_string.split('\n')[2:])
    cparms = json.loads(response_string)
    fx = cparms["fx"] / 262144
    fy = cparms["fy"] / 262144
    u0 = cparms["u0"] / 262144
    v0 = cparms["v0"] / 262144
    print(fx, fy, u0, v0)
    print("------------------------------")


def printSettings(ser):
    command = "AT+ISP?\r"
    ser.write(command.encode("ASCII"))
    response = ser.readlines()
    print("ISP Response:", response)
    command = "AT+BINN?\r"
    ser.write(command.encode("ASCII"))
    response = ser.readlines()
    print("BINN Response:", response)
    command = "AT+DISP?\r"
    ser.write(command.encode("ASCII"))
    response = ser.readlines()
    print("DISP Response:", response)
    command = "AT+BAUD?\r"
    ser.write(command.encode("ASCII"))
    response = ser.readlines()
    print("BAUD Response:", response)
    command = "AT+UNIT?\r"
    ser.write(command.encode("ASCII"))
    response = ser.readlines()
    print("UNIT Response:", response)
    command = "AT+FPS?\r"
    ser.write(command.encode("ASCII"))
    response = ser.readlines()
    print("FPS Response:", response)
    command = "AT+ANTIMMI?\r"
    ser.write(command.encode("ASCII"))
    response = ser.readlines()
    print("ANTIMMI Response:", response)
    print("------------------------------")


"""Set TOF sensor parameters
Inputs:
    ISP: 0: turn ISP off; 1: turn ISP on
    BINN: 1: output 100x100 pixel frame; 2: output 50x50 pixel frame; 4: output 25x25 pixel frame
    DISP: 0: all off; 1: lcd display on; 2: usb display on; 3: lcd and usb display on; 4: uart display on; 
          5: lcd and uart display on; 6: usb and uart display on; 7: lcd, usb and uart display on
    BAUD: 0: 9600; 1: 57600; 2: 115200; 3: 230400; 4: 460800; 5: 921600; 6: 1000000; 7: 2000000; 8: 3000000
    UNIT: 0: auto; 1-10: quantizated by unit(mm)
    FPS: 1-19: set frame per second
    ANTIMMI: -1: disable anti-mmi; 0: auto anti-mmi; 1-41: manual anti-mmi usb display on

Return:
    Set parameters through serial
"""
def setSettings(ser, isp_value=None, binn_value=None, disp_value=None, baud_value=None, 
                     unit_value=None, fps_value=None, antimmi_value=None):
    if isp_value is not None: 
        command = "AT+ISP=%1d\r" % isp_value
        ser.write(command.encode("ASCII"))
        response = ser.readlines()
        print("ISP Response:", response)
    if binn_value is not None: 
        command = "AT+BINN=%1d\r" % binn_value
        ser.write(command.encode("ASCII"))
        response = ser.readlines()
        print("BINN Response:", response)
    if disp_value is not None: 
        command = "AT+DISP=%1d\r" % disp_value
        ser.write(command.encode("ASCII"))
        print("Set DISP value as ", disp_value)
        # response = ser.readlines()
        # print("DISP Response:", response)
    if baud_value is not None: 
        command = "AT+BAUD=%1d\r" % baud_value
        ser.write(command.encode("ASCII"))
        ser.baudrate = BAUD[baud_value]         # change the baudrate of the serial 
        response = ser.readlines()
        print("BAUD Response:", response)
    if unit_value is not None: 
        command = "AT+UNIT=%1d\r" % unit_value
        ser.write(command.encode("ASCII"))
        response = ser.readlines()
        print("UNIT Response:", response)
    if fps_value is not None: 
        command = "AT+FPS=%1d\r" % fps_value
        ser.write(command.encode("ASCII"))
        response = ser.readlines()
        print("FPS Response:", response)
    if antimmi_value is not None:
        command = "AT+ANTIMMI=%1d\r" % antimmi_value
        ser.write(command.encode("ASCII"))
        response = ser.readlines()
        print("ANTIMMI Response:", response)
    print("------------------------------")


def testPort(ser):
    command = "AT\r"
    ser.write(command.encode("ASCII"))
    response = ser.readline().decode("ASCII").strip()
    print("Response:", response)


def display_image(frame_data):
    cv2.imshow("Figure", frame_data)
    cv2.waitKey(1)


ser = serial.Serial()

ser.port = "/dev/ttyUSB0"
ser.baudrate = 115200
ser.bytesize = serial.EIGHTBITS
ser.parity = serial.PARITY_NONE
ser.stopbits = serial.STOPBITS_ONE
ser.xonxoff = False
ser.rtscts = False
ser.dsrdtr = False
ser.timeout = 0.2
# ser.write_timeout = 
# ser.inter_byte_timeout = 
# ser.exclusive = 

ser.open()

print("Connected to Serial: ", ser.is_open)
printSettings(ser)

setSettings(ser, baud_value=5)
printSettings(ser)

intrinsicParam(ser)

setSettings(ser, isp_value=1, binn_value=1, unit_value=0, fps_value=10)
printSettings(ser)

print("Serial Initialization Completed.")

setSettings(ser, disp_value=4)

print("Start Receiving Depth Image.")


""" Display Image """

image_array = np.zeros((100, 100), dtype=np.uint8)

while True:
    try:
        header = ser.read(2)
        # print(header)

        if header == b'\x00\xff':
            packet_length = ser.read(2)

            other_content = ser.read(16)

            image_data = b''  

            while len(image_data) < 10000:
                image_data += ser.read(10000 - len(image_data))

            check_byte = ser.read(1)

            end = ser.read(1)
            # print(end)

            if end == b'\xdd':
                image_pixels = np.frombuffer(image_data, dtype=np.uint8)
                image_array = np.reshape(image_pixels, (100, 100))
                image_disp = cv2.resize(image_array, (500, 500))
                display_image(image_disp)

    except KeyboardInterrupt:
        break

setSettings(ser, disp_value=0)
print("Serial Closing.")

ser.close()
