#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def display_min_distance(msg):
    print('Minimum distance is: %s' %min(msg.ranges))

def main():
    #initialize ROS node
    rospy.init_node('move_laser_scan', anonymous=True)

    #Create publisher to cmd_vel topic
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    #subscribe to Laserscan topic
    laser_scan_sub = rospy.Subscriber('/laser/scan', LaserScan, display_min_distance)
    
    #define at rate at which to publish
    rate = rospy.Rate(10) # 10Hz

    #publish velocity message
    while not rospy.is_shutdown():
        #Create new twist message
        vel_msg = Twist()

        #ser linear and angular velocities
        vel_msg.linear.x = 0.3 # move in x-direction with 0.3 m/s 
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0.3 # rotate with 0.3 rad/sec

        velocity_publisher.publish(vel_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
