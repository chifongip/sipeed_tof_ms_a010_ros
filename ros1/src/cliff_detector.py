#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PointStamped
from cv_bridge import CvBridge
import cv2
import tf
import tf2_ros
import numpy as np

class ImageSubscriber:
    def __init__(self):
        self.img_topic = rospy.get_param("img_topic", "sipeed_tof_ms_a010/depth/image_raw")
        self.info_topic = rospy.get_param("info_topic", "sipeed_tof_ms_a010/depth/camera_info")
        self.frame_id = rospy.get_param("frame_id", "dep_cam_front_link")
        
        self.cam_height = rospy.get_param("cam_height", 0.3)
        self.cam_angle = rospy.get_param("cam_angle", 15)
        self.cliff_threshold = rospy.get_param("cliff_threshold", 0.1)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_info_received = False

        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber(self.img_topic, Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(self.info_topic, CameraInfo, 
            self.camera_info_callback, queue_size=1)


    def camera_info_callback(self, msg):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]
        self.camera_info_received = True
        self.camera_info_sub.unregister()
        try:
            trans = self.tfBuffer.lookup_transform("base_link", self.frame_id, rospy.Time.now(), rospy.Duration(10.0))
            trans_matrix = tf.transformations.translation_matrix((trans.transform.translation.x,
                                                                  trans.transform.translation.y,
                                                                  trans.transform.translation.z))
            quaternion = (trans.transform.rotation.x,
                          trans.transform.rotation.y,
                          trans.transform.rotation.z,
                          trans.transform.rotation.w)
            rotation_matrix = tf.transformations.quaternion_matrix(quaternion)
            self.trans_matrix = tf.transformations.concatenate_matrices(trans_matrix, rotation_matrix)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)


    def image_callback(self, msg):
        try:
            if not self.camera_info_received:
                rospy.logwarn("Camera calibration parameters not received yet.")
                return
            
            # Convert ROS Image message to OpenCV format
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            height, width = img.shape
            col = 50
            # for col in range(width):
            for row in range(height - 1, 0, -1):
                u_curr = (col - self.cx) / self.fx
                v_curr = (row - self.cy) / self.fy
                dst_curr = pow(((img[row, col]) / 5.1), 2) / 1000

                x_curr = dst_curr  
                y_curr = -dst_curr * u_curr  
                z_curr = -dst_curr * v_curr

                p_curr = np.array([x_curr, y_curr, z_curr, 1])
                p_trans = np.dot(self.trans_matrix, p_curr)

                p_curr_trans = p_trans[:3]

                print(p_curr_trans)



                    # dst_next = pow(((img[row - 1, col]) / 5.1), 2) / 1000

                    # z_next = -dst_next * self.cy

                    # z_diff = z_curr - z_next

                    # if z_diff > self.cliff_threshold:
                    #     cv2.circle(img, (col, row), 3, (0, 0, 255), -1)
                    #     break
            print("-------------------------")
            # Process the image as desired
            # Example: Display the image
            cv2.imshow('Image', img)
            cv2.waitKey(1)  # Refresh the display

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