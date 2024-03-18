import serial
import numpy as np
import cv2
import time
import json


def display_image(frame_data):
    cv2.imshow("Figure", frame_data)
    cv2.waitKey(1)

ser = serial.Serial(port = "/dev/depth_camera",
                    baudrate = 115200,
                    bytesize = serial.EIGHTBITS,
                    parity = serial.PARITY_NONE,
                    stopbits = serial.STOPBITS_ONE,
                    xonxoff = False,
                    rtscts = False,
                    dsrdtr = False
                    )

print(ser.is_open)

isp_value = 1                               # turn ISP on
command = "AT+ISP=%1d\r" % isp_value
ser.write(command.encode("utf-8"))

binn_value = 1                              # output 100x100 pixel frame
command = "AT+BINN=%1d\r" % binn_value
ser.write(command.encode("utf-8"))

disp_value = 2                              # usb display on
command = "AT+DISP=%1d\r" % disp_value
ser.write(command.encode("utf-8"))

baud_value = 2                              # 115200
command = "AT+BAUD=%1d\r" % baud_value
ser.write(command.encode("utf-8"))

unit_value = 0                              # auto
command = "AT+UNIT=%1d\r" % unit_value
ser.write(command.encode("utf-8"))

fps_value = 10                              # 10 fps
command = "AT+FPS=%1d\r" % fps_value
ser.write(command.encode("utf-8"))

antimmi_value = -1                          # disable anti-mmi
command = "AT+ANTIMMI=%1d\r" % antimmi_value
ser.write(command.encode("utf-8"))

# command = "AT+COEFF?"
# ser.write(command.encode("utf-8"))
# response = ser.readline().decode().strip()
# cparms = json.loads(response)
# print(cparms)

# ser.write(b"AT+COEFF?\r")
# response = ser.readline()
# print(response)

# Initialize the image array
image_array = np.zeros((100, 100), dtype=np.uint8)

time.sleep(3)

print("connected")

# Read data from the serial port
while True:
    try:
        header = ser.read(2)
        if header == b'\x00\xff':
            packet_length = ser.read(2)

            other_content = ser.read(16)

            image_data = ser.read(10000)

            check_byte = ser.read(1)

            end = ser.read(1)
            if end == b'\xdd':
                image_pixels = np.frombuffer(image_data, dtype=np.uint8)
                image_array = np.reshape(image_pixels, (100, 100))
                image_disp = cv2.resize(image_array, (500, 500))
                display_image(image_disp)

    except KeyboardInterrupt:
        # Exit the loop if Ctrl+C is pressed
        break

# Close the serial port
ser.close()