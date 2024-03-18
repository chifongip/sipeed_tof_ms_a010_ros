#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

import serial
import numpy as np
import cv2
import time
import json


class msa010Driver:
    def __init__(self):
        self.device = rospy.get_param("msa010_ros_driver/device", "/dev/depth_camera")
        self.frame_id = rospy.get_param("msa010_ros_driver/frame_id", "dep_cam_front_link")
        self.debug = rospy.get_param("msa010_ros_driver/debug", 1)
        self.fx = rospy.get_param("msa010_ros_driver/fx", 75)
        self.fy = rospy.get_param("msa010_ros_driver/fy", 75)
        self.u0 = rospy.get_param("msa010_ros_driver/u0", 50)
        self.v0 = rospy.get_param("msa010_ros_driver/v0", 50)

        self.header = Header()
        self.header.frame_id = self.frame_id

        self.cam_info = CameraInfo()
        self.cam_info.height = 100
        self.cam_info.width = 100
        self.cam_info.distortion_model = "plumb_bob"
        self.cam_info.D = [0, 0, 0, 0, 0]
        self.cam_info.K = [self.fx, 0, self.u0, 0, self.fy, self.v0, 0, 0, 1]
        self.cam_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        self.cam_info.P = [self.fx, 0, self.u0, 0, 0, self.fy, self.v0, 0, 0, 0, 1, 0]

        self.depth_img_pub = rospy.Publisher("depth/image_raw", Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher("depth/camera_info", CameraInfo, queue_size=1)

        self.ser = serial.Serial(port = self.device,
                                baudrate = 115200,
                                bytesize = serial.EIGHTBITS,
                                parity = serial.PARITY_NONE,
                                stopbits = serial.STOPBITS_ONE,
                                xonxoff = False,
                                rtscts = False,
                                dsrdtr = False,
                                timeout = 1.0
                                )

        print(self.ser.is_open)

        if self.debug == 0:
            self.initCamera(self.ser)

            print("connected")

            self.bridge = CvBridge()
            self.image_array = np.zeros((100, 100), dtype=np.uint8)

            rospy.loginfo("Start MSA010 Driver.")

            self.msa010Publisher(self.ser)

        elif self.debug == 1:
            self.intrinsicParam(self.ser)


    def intrinsicParam(self, ser):
        command = "AT+COEFF?\r"
        ser.write(command.encode("utf-8"))
        response = ser.readlines()
        response_string = b''.join(response).decode()
        response_string = response_string.strip()
        response_string = '\n'.join(response_string.split('\n')[2:])
        cparms = json.loads(response_string)
        fx = cparms["fx"] / 262144
        fy = cparms["fy"] / 262144
        u0 = cparms["u0"] / 262144
        v0 = cparms["v0"] / 262144
        print(fx, fy, u0, v0)


    def initCamera(self, ser):
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


    def display_image(self, frame_data):
        cv2.imshow("Figure", frame_data)
        cv2.waitKey(1)


    def msa010Publisher(self, ser):
        while not rospy.is_shutdown():
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

                        # header 
                        self.header.stamp = rospy.Time.now()

                        # camera info
                        self.cam_info.header = self.header
                        self.camera_info_pub.publish(self.cam_info)

                        # depth image 
                        img_msg = self.bridge.cv2_to_imgmsg(image_array, encoding="8UC1")
                        img_msg.header = self.header
                        self.depth_img_pub.publish(img_msg)
                        # image_disp = cv2.resize(image_array, (500, 500))
                        # self.display_image(image_disp)

                    else:
                        ser.close()
                        time.sleep(0.1)
                        ser.open()
                        
            except:
                pass

        ser.close()


if __name__ == '__main__':
    try:
        rospy.init_node('msa010_driver', anonymous=True)
        msa010_driver = msa010Driver()
    except rospy.ROSInterruptException:
        pass