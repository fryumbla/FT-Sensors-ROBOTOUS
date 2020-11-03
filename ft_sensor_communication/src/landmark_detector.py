#!/usr/bin/env python
"""
author:  Eugene Auh
version: 0.1.0
brief: Detect landmark for example Aruco, QR, etc. It is used to apply on EKF-SLAM or anything else.

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy of this
software and associated documentation files (the ""Software""), to deal in the 
Software without restriction, including without limitation the rights to use, copy, 
modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, 
and to permit persons to whom the Software is furnished to do so, subject to the 
following conditions:
The above copyright notice and this permission notice shall be included in all copies 
or substantial portions of the Software.
THE SOFTWARE IS PROVIDED *AS IS*, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE 
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import rospy
import cv2
import cv2.aruco as aruco
import cv_bridge
import numpy as np
import math

from sensor_msgs.msg import Image, CameraInfo
# from bebop_slam_project.msg import ArucoInfo


class LandmarkDetector():
    def cameraInfoCb(self, msg):
        if self.camera_info is False:
            self.h = msg.height
            self.w = msg.width
            self.dist_model = msg.distortion_model
            self.K = np.array(msg.K).reshape((3, 3))
            self.D = np.array(msg.D)[0:4]
            self.fx = self.K[0, 0]
            self.cx = self.K[0, 2]
            self.cy = self.K[1, 2]

            # Generate undistort rectiry map of the image
            self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.K, self.D, None, self.K, (self.w, self.h), 5)

            self.camera_info = True

        else:
            pass

    def imageCb(self, msg):
        if self.camera_info is True:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Use if there is distortion
            # cv_img = cv2.remap(cv_img, self.mapx, self.mapy, cv2.INTER_LINEAR)

            cv_gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(cv_gray, self.aruco_dict, parameters=self.aruco_param)

            if ids is not None:
                # Draw marker ids and corners on image
                cv_img = aruco.drawDetectedMarkers(cv_img, corners, ids)

                # Draw marker poses on image
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, self.aruco_size, self.K, None)
                for i in range(0, len(ids)):
                    cv_img = aruco.drawAxis(cv_img, self.K, None, rvec[i], tvec[i], 0.1)

                tran_list = list(tvec)
                if len(tran_list) != 1:
                    
                    obj1_tran = tran_list[0][0]
                    obj2_tran = tran_list[1][0]
                    obj1_tran_x = obj1_tran[0].item()
                    obj1_tran_y = obj1_tran[1].item()
                    obj2_tran_x = obj2_tran[0].item()
                    obj2_tran_y = obj2_tran[1].item()

                    sq_dist = (obj1_tran_x - obj2_tran_x)**2 + (obj1_tran_y - obj2_tran_y)**2
                    dist = math.sqrt(sq_dist)
                    print("Distance: " + str(dist))
                # rospy.sleep(0.1)

                # landmark = ArucoInfo()
                # landmark.header.frame_id = 'camera_optical'
                # landmark.header.stamp = rospy.Time.now()
                # landmark.ids = ids.flatten('C').tolist()
                # landmark.corners = np.array(corners).flatten('C').tolist()
                # landmark.rvec = rvec.flatten('C').tolist()
                # landmark.tvec = tvec.flatten('C').tolist()
                # Publish Aruco marker informations
                # self.landmark_pub.publish(landmark)

            # Publish image message with Aruco markers
            self.image_with_landmark_publisher.publish(self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8"))

        else:
            pass

    def __init__(self):
        # Initialize node
        rospy.init_node('landmark_detector')
        self.rate = 10
        self.r = rospy.Rate(self.rate)

        # Parameters for camera and Aruco
        self.camera_info = False
        self.bridge = cv_bridge.CvBridge()
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
        self.aruco_param = aruco.DetectorParameters_create()
        self.aruco_size = rospy.get_param('/aruco/size', 0.030)

        # Publishers and Subscribers
        self.image_with_landmark_publisher = rospy.Publisher('/kinect2/hd/image_with_landmark', Image, queue_size=10)
        # self.landmark_pub = rospy.Publisher('/landmark', ArucoInfo, queue_size=10)
        rospy.Subscriber('/kinect2/hd/camera_info', CameraInfo, self.cameraInfoCb)
        rospy.Subscriber('/kinect2/hd/image_color_rect', Image, self.imageCb)

    def loop(self):
        self.r.sleep()


if __name__ == "__main__":
    try:
        landmark_detector = LandmarkDetector()
        while not rospy.is_shutdown():
            landmark_detector.loop()
    except rospy.ROSInterruptException:
        pass