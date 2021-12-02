#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

class DroneAction():
    # def takeoff():
    #     pub = rospy.Publisher("drone/takeoff", Empty, queue_size=1)
    #     rospy.init_node('takeoff', anonymous=True)
    #     rate = rospy.Rate(10)  # 10hz
    #     while not rospy.is_shutdown():
    #         pub.publish(Empty())
    #         rate.sleep()

    def __init__(self):

        self.ctrl_c = False
        self.rate = rospy.Rate(10)

    def publish_once_in_cmd_vel(self, cmd):
        """
        This is because publishing in topics sometimes fails teh first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while not self.ctrl_c:
            connections = self._pub_cmd_vel.get_num_connections()
            if connections > 0:
                self._pub_cmd_vel.publish(cmd)
                rospy.loginfo("Publish in cmd_vel...")
                break
            else:
                self.rate.sleep()

    def up_drone(self, linearz):
        rospy.loginfo("Fly Up...")
        self._move_msg.linear.x = 2.5
        #self._move_msg.angular.x = 0.0
        #self._move_msg.angular.y = 0.0
        self._move_msg.linear.z = linearz
        self.publish_once_in_cmd_vel(self._move_msg)

    # function that stops the drone from any movement
    def stop_drone(self):
        rospy.loginfo("Stopping...")
        self._move_msg.linear.x = 0.0
        self._move_msg.angular.z = 0.0
        self.publish_once_in_cmd_vel(self._move_msg)

    # function that makes the drone turn 90 degrees
    def turn_drone(self):
        rospy.loginfo("Turning...")
        self._move_msg.linear.x = 0.0
        self._move_msg.angular.z = 0.0
        self.publish_once_in_cmd_vel(self._move_msg)

    # function that makes the drone move forward
    def move_forward_drone(self, moveforward):
        rospy.loginfo("Moving forward...")
        self._move_msg.linear.x = moveforward
        self._move_msg.linear.z = 0.0
        self.publish_once_in_cmd_vel(self._move_msg)

    def move_square(self):
        # this callback is called when the action server is called.
        # this is the function that computes the Fibonacci sequence
        # and returns the sequence to the node that called the action server

        # helper variables
        r = rospy.Rate(10)

        # define the different publishers and messages that will be used
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._move_msg = Twist()
        self._pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
        self._takeoff_msg = Empty()
        self._pub_land = rospy.Publisher('/drone/land', Empty, queue_size=1)
        self._land_msg = Empty()

        # make the drone takeoff
        i = 0
        while not i == 3:
            self._pub_takeoff.publish(self._takeoff_msg)
            rospy.loginfo('Taking off...')
            time.sleep(0.5)
            i += 1

        # self.up_drone()

        i = 0
        while not i == 3:
            self.up_drone(0.8)
            time.sleep(0.35)
            i += 1

        # i = 0
        # while not i == 3:
        #     self.up_drone()
        #     rospy.loginfo('Up...')
        #     #time.sleep(1)
        #     i += 1

        # define the seconds to move in each side of the square (which is taken from the goal) and the seconds to turn
        sideSeconds = 2.0
        turnSeconds = 1.8

        i = 0
        for i in range(0, 5):
            # Logic that makes the robot move forward and turn
            self.move_forward_drone(0.6)
            time.sleep(sideSeconds)
            # self.turn_drone()
            # time.sleep(turnSeconds)

            # the sequence is computed at 1 Hz frequency
            r.sleep()

        i = 0
        while not i == 3:
            self.up_drone(2)
            time.sleep(0.4)
            i += 1

        i = 0
        for i in range(0, 5):
            # Logic that makes the robot move forward and turn
            self.move_forward_drone(-1.0)
            time.sleep(sideSeconds)
            # self.turn_drone()
            # time.sleep(turnSeconds)

            # the sequence is computed at 1 Hz frequency
            r.sleep()

        i = 0
        while not i == 3:
            self.up_drone(-1.9)
            time.sleep(0.4)
            i += 1

        i = 0
        for i in range(0, 5):
            # Logic that makes the robot move forward and turn
            self.move_forward_drone(0.9)
            time.sleep(sideSeconds)
            # self.turn_drone()
            # time.sleep(turnSeconds)

            # the sequence is computed at 1 Hz frequency
            r.sleep()

        self.stop_drone()
        i = 0

        # i = 0
        # while not i == 3:
        #     self.up_drone()
        #     time.sleep(0.4)
        #     i += 1

        while not i == 3:
            self._pub_land.publish(self._land_msg)
            rospy.loginfo('Landing...')
            time.sleep(1)
            i += 1


if __name__ == '__main__':
    rospy.init_node('move_square')
    act = DroneAction()
    try:
        act.move_square()
    except rospy.ROSInterruptException:
        pass
