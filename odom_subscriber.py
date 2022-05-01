#!/usr/bin/env python3

import rospy
# import Odometry from the nav_msgs package instead:
from nav_msgs.msg import Odometry
# additionally, import the "euler_from_quaternion" function from the tf library
# for converting the raw orientation values from the odometry message:
from tf.transformations import euler_from_quaternion

class Subscriber:

    def callback(self, odom_data):
        # in order to get theta_z (yaw), we need to convert the 4 orientation
        # values (x, y, z, w) as provided in the odometry message.  These are in
        # quaternions, so we use the euler_from_quaternion function to convert to
        # roll, pitch and yaw.
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')

        # x and y don't need to be converted, so we just obtain those directly from
        # the odometry message as follows:
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        # Then, print out the values that we are interested in:
        if self.counter > 2:
            self.counter = 0
            print("x = {:.3f}, y = {:.3f}, theta_z = {:.3f}".format(pos_x, pos_y, yaw))
        else:
            self.counter += 1

    def __init__(self):
        # in the initialisation, the node name can be changed, but this isn't essential:
        rospy.init_node('odom_subscriber_node', anonymous=True)
        # When setting up the subscriber, the "odom" topic needs to be specified
        # and the message type (Odometry) needs to be provided
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        # an optional status message:
        rospy.loginfo("odom subscriber is active...")
        self.counter = 0

    def main_loop(self):
        # set the node to remain active until closed manually:
        rospy.spin()

if __name__ == '__main__':
    subscriber_instance = Subscriber()
    subscriber_instance.main_loop()