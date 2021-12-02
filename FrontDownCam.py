#!/usr/bin/env python
from __future__ import print_function

import math
import roslib
#roslib.load_manifest('drone')
import sys
import message_filters
import rospy
import cv2
import numpy as np
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String
from std_msgs.msg import Int16  # For error/angle plot publishing
from sensor_msgs.msg import Image
#from bebop_msgs.msg import CommonCommonStateBatteryStateChanged  # For battery percentage
from cv_bridge import CvBridge, CvBridgeError



class FrontDownCam:

    def __init__(self):
        self.pub_vel = rospy.Publisher('drone/cmd_vel', Twist, queue_size=1)
        self.pub_land = rospy.Publisher('drone/land', Empty, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = message_filters.Subscriber('/drone/front_camera/image_raw', Image)
        self.image_sub2 = message_filters.Subscriber('/drone/down_camera/image_raw', Image)
        self.takeoff_sub = rospy.Subscriber('/drone/takeoff', Empty, self.isTakeoff)
        self.land_sub = rospy.Subscriber('/drone/land', Empty, self.isLand)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub,self.image_sub2],10,0.1)
        self.ts.registerCallback(self.callback)

        self.Kp = 0.112                 # Ku=0.14 T=6. PID: p=0.084,i=0.028,d=0.063. PD: p=0.112, d=0.084/1. P: p=0.07
        self.Ki = 0
        self.kd = 1
        self.integral = 0
        self.derivative = 0
        self.last_error = 0
        self.Kp_ang = 0.01             # Ku=0.04 T=2. PID: p=0.024,i=0.024,d=0.006. PD: p=0.032, d=0.008. P: p=0.02/0.01
        self.Ki_ang = 0
        self.kd_ang = 0
        self.integral_ang = 0
        self.derivative_ang = 0
        self.last_ang = 0
        self.was_line = 0
        self.line_side = 0
        self.battery = 0
        self.line_back = 1
        self.landed = 0
        self.takeoffed = 0
        self.error = []
        self.angle = []
        self.fly_time = 0.0
        self.start = 0.0
        self.stop = 0.0
        self.velocity = 0.06

    def cam_down(self):
        cam = Twist()
        cam.angular.y = -90.00
        self.pub_camdown.publish(cam)

    # Detect the line and piloting
    def line_detect(self, cv_image):
        pass

    # Detect landing sign
    def land_detect(self, cv_front_cam):
        land_mask = cv2.cvtColor(cv_front_cam, cv2.COLOR_BGR2HSV)

        #land_mask = cv2.GaussianBlur(land_mask, (21, 21), 0)
        low_red = np.array([105, 0, 0])
        up_red = np.array([179, 255, 255])
        land_mask2 = cv2.inRange(land_mask, low_red, up_red)
        contours_blk2, _ = cv2.findContours(land_mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blk2.sort(key=cv2.minAreaRect)
        #cv2.imshow("land mask", land_mask)
        # print(len(contours_blk2))
        if len(contours_blk2) > 0 and cv2.contourArea(contours_blk2[0]) > 30000:
            self.landing()

    # Landing the drone
    def landing(self):
        land = Empty()
        self.pub_land.publish(land)
        self.takeoffed = 0
        self.stop = time.time()
        # self.errorPlot()

    def isTakeoff(self, data):
        time.sleep(3.5)
        self.start = time.time()
        self.takeoffed = 1
        self.landed = 0

    def isLand(self, data):
        self.landed = 1
        self.takeoffed = 0
        self.stop = time.time()
        #self.errorPlot()

    # Image processing @ 10 FPS
    def callback(self, front_cam):

        cv2.imshow("Image window", cv_image)
        cv2.imshow("mask", mask)
        cv2.waitKey(1) & 0xFF

        try:
            cv_front_cam = self.bridge.imgmsg_to_cv2(front_cam, "bgr8")
            #cv_down_cam = self.bridge.imgmsg_to_cv2(down_cam, "bgr8")
        except CvBridgeError as e:
            print(e)

        #cv_front_cam = self.bridge.imgmsg_to_cv2(front_cam, "bgr8")
        cv2.imshow("Front_Camera", cv_front_cam)
        #cv_down_cam = self.bridge.imgmsg_to_cv2(down_cam, "bgr8")
        #cv2.imshow("Down_Camera", cv_down_cam)

        #if self.takeoffed and (not self.landed):
            #self.line_detect(cv_image)
            #self.land_detect(cv_down_cam)

        self.land_detect(cv_front_cam)

        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #    rospy.signal_shutdown('Quit')
        #    cv2.destroyAllWindows()
        cv2.waitKey(1) & 0xFF


def main():
    rospy.init_node('FrontDownCam', anonymous=True)
    ic = FrontDownCam()
    #time.sleep(3)
    try:
        #ts = message_filters.ApproximateTimeSynchronizer([ic.image_sub,ic.image_sub2],10,0.1)
        #ts.registerCallback(callback)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
