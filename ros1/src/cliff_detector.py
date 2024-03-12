#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class ImageSubscriber:
    def __init__(self):
        self.img_topic = rospy.get_param("cliff_detector/img_topic", "sipeed_tof_ms_a010/depth/image_raw")
        self.info_topic = rospy.get_param("cliff_detector/info_topic", "sipeed_tof_ms_a010/depth/camera_info")
        self.frame_id = rospy.get_param("cliff_detector/frame_id", "dep_cam_front_link")
        
        self.cam_height = rospy.get_param("cliff_detector/cam_height", 0.3)
        self.cam_angle = rospy.get_param("cliff_detector/cam_angle", 15)
        self.cliff_threshold = rospy.get_param("cliff_detector/cliff_threshold", 0.1)

        self.img_freq = rospy.get_param("cliff_detector/img_freq", 10)
        self.range_min = rospy.get_param("cliff_detector/range_min", 0.2)
        self.range_max = rospy.get_param("cliff_detector/range_max", 2.5)

        cam_x = rospy.get_param("cliff_detector/cam_x", 0.2)
        cam_y = rospy.get_param("cliff_detector/cam_y", 0.0)
        cam_z = rospy.get_param("cliff_detector/cam_z", 0.3)

        self.row_upper = rospy.get_param("cliff_detector/row_upper", 70)
        self.col_left = rospy.get_param("cliff_detector/col_left", 20)
        self.col_right = rospy.get_param("cliff_detector/col_right", 80)

        self.skip_row_upper = rospy.get_param("cliff_detector/skip_row_upper", 1)
        self.skip_row_bottom = rospy.get_param("cliff_detector/skip_row_bottom", 1)

        self.kernel = np.ones((3, 3), np.uint8)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_info_received = False

        self.img_height = 100
        self.img_width = 100
        self.delta_row = np.zeros(100) 
        self.vertical_fov = np.deg2rad(60)
        self.horizontal_fov = np.deg2rad(70)

        self.dist_to_ground = np.zeros(100)

        # ZYX = 0, 15, 0
        self.trans = np.array([[ 0.9659258, 0.0000000, 0.2588190, cam_x], 
                               [ 0.0000000, 1.0000000, 0.0000000, cam_y], 
                               [-0.2588190, 0.0000000, 0.9659258, cam_z],
                               [0.0, 0.0, 0.0, 1.0]])
        
        # ZYX = 35, 15, 0
        # self.trans = np.array([[ 0.7912401, -0.5735765, 0.2120122, cam_x], 
        #                        [ 0.5540323,  0.8191521, 0.1484525, cam_y], 
        #                        [-0.2588190,  0.0000000, 0.9659258, cam_z],
        #                        [0.0, 0.0, 0.0, 1.0]])

        # ZYX = -35, 15, 0
        # self.trans = np.array([[ 0.7912401, 0.5735765, 0.2120122, cam_x], 
        #                        [-0.5540323, 0.8191521,-0.1484525, cam_y], 
        #                        [-0.2588190, 0.0000000, 0.9659258, cam_z],
        #                        [0.0, 0.0, 0.0, 1.0]])
        
        self.bridge = CvBridge()

        self.img_sub = rospy.Subscriber(self.img_topic, Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(self.info_topic, CameraInfo, 
            self.camera_info_callback, queue_size=1)
        
        self.cliff_pub = rospy.Publisher("depth/cliff", Image, queue_size=1)
        self.debug_pub = rospy.Publisher("depth/debug_image", Image, queue_size=1)

        self.scan_pub = rospy.Publisher("depth/scan", LaserScan, queue_size=1)


    def calcDeltaAngleForImgRows(self):
        for i in range(self.img_height):
            self.delta_row[i] = self.vertical_fov * (i - self.cy - 0.5) / (self.img_height - 1)


    def calcGroundDistancesForImgRows(self): 
        alpha = np.deg2rad(self.cam_angle)
        for i in range(self.img_height):
            if self.delta_row[i] + alpha > 0:
                self.dist_to_ground[i] = self.cam_height * np.sin(np.pi / 2 - self.delta_row[i]) \
                    / np.cos(np.pi / 2 - self.delta_row[i] - alpha)
            else:
                self.dist_to_ground[i] = 100


    def camera_info_callback(self, msg):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]
        self.camera_info_received = True

        self.calcDeltaAngleForImgRows()
        self.calcGroundDistancesForImgRows()
        
        self.camera_info_sub.unregister()


    def image_callback(self, msg):
        try:
            if not self.camera_info_received:
                rospy.logwarn("Camera calibration parameters not received yet.")
                return
            
            # convert ROS image message to OpenCV format
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            img = np.rot90(img, 2)

            # img_cliff = img.copy()
            img_cliff = np.zeros([100, 100])
            img_edge = np.zeros([100, 100])

            for col in range(self.col_left, self.col_right):
                for row in range(self.img_height - 1, self.row_upper - 1, -1):
                    dst = pow(((img[row, col]) / 5.1), 2) / 1000
                    if dst < (self.dist_to_ground[row] + self.cliff_threshold): 
                        img_cliff[row][col] = 255
                    else:
                        img_cliff[row][col] = 0
                    # if dst > (self.dist_to_ground[row] + self.cliff_threshold) and img[row, col] != 255:
                    #     # and dst > self.range_min and dst < self.range_max:
                    #     img_cliff[row][col] = 0
                    # else:
                    #     img_cliff[row][col] = 255
            
            img_cliff = cv2.dilate(img_cliff, self.kernel, iterations=3)
            img_cliff = cv2.erode(img_cliff, self.kernel, iterations=3)

            img_cliff = img_cliff.astype(np.float32)

            p_dst = np.zeros(100)
            
            for col in range(self.col_left, self.col_right):
                for row in range(self.img_height - 1 - self.skip_row_bottom, self.row_upper - 1 + self.skip_row_upper, -1):   # ignore the first and last row
                    if img_cliff[row][col] == 255 and img_cliff[row - 1][col] == 0:
                        img_edge[row][col] = 255

                        u = (col - self.cx) / self.fx
                        v = (row - self.cy) / self.fy
                        dst = pow(((img[row, col]) / 5.1), 2) / 1000

                        x = dst 
                        y = -dst * u 
                        z = -dst * v 

                        p = np.array([x, y, z, 1])

                        p_trans = np.dot(self.trans, p)

                        p_dst[col] = math.sqrt(pow(p_trans[0], 2) + pow(p_trans[1], 2))

                        break

            img_edge = img_edge.astype(np.float32)

            scan_msg = LaserScan()

            scan_msg.header.stamp = msg.header.stamp
            scan_msg.header.frame_id = "laser_link"

            scan_msg.angle_min = -self.horizontal_fov / 2
            scan_msg.angle_max = self.horizontal_fov / 2
            scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (self.img_width - 1)
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 1 / self.img_freq
            scan_msg.range_min = self.range_min
            scan_msg.range_max = self.range_max
            scan_msg.ranges = p_dst[::-1]
            self.scan_pub.publish(scan_msg)

            img_cliff_msg = self.bridge.cv2_to_imgmsg(img_cliff, encoding="32FC1")
            img_cliff_msg.header = msg.header
            self.cliff_pub.publish(img_cliff_msg)

            img_edge_msg = self.bridge.cv2_to_imgmsg(img_edge, encoding="32FC1")
            img_edge_msg.header = msg.header
            self.debug_pub.publish(img_edge_msg)

        except Exception as e:
            print(e)


if __name__ == '__main__':
    try:
        rospy.init_node('image_subscriber', anonymous=True)
        image_subscriber = ImageSubscriber()
        rospy.loginfo("Start Cliff Detector.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass