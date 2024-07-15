#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def display_robot_position(msg):
    print('X: %s, Y: %s' %(msg.pose.pose.position.x, msg.pose.pose.position.y))

def main():
    # initialize Ros Node
    rospy.init_node('move_display_robot', anonymous=True)

    # Create publisher to cmd_vel topic
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Subscribe to Odom topic
    odom_sub = rospy.Subscriber('/odom', Odometry, display_robot_position)

    # Define rate at which to publish
    rate = rospy.Rate(10) # 10 Hz

    # Publish the velocity message
    while not rospy.is_shutdown():
        # Create a new Twist message
        vel_msg = Twist()

        # Set linear and angular velocities
        vel_msg.linear.x = 0.3 # move in x-direction with 0.5 m/s
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.3 # rotate with 0.1 rad/sec

        velocity_publisher.publish(vel_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass    



