#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_turtle(velocity_data):
    """
    Function to move the turtlesim based on the received velocity data.
    """
    # Create a publisher to the /turtle1/cmd_vel topic
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    # Set the rate of publishing
    rate = rospy.Rate(10) # 10 Hz

    # Check if rospy is up and running
    while not rospy.is_shutdown():
        # Log the received data
        rospy.loginfo(f"Moving the turtle: linear.x={velocity_data.linear.x}, angular.z={velocity_data.angular.z}")

        # Publish the velocity data
        pub.publish(velocity_data)

        # Sleep to maintain the rate
        rate.sleep()

def callback(data):
    if data.linear.x is None or data.angular.z is None:
        rospy.logerr("Received invalid Twist message: linear.x or angular.z is None")
        return

    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%s, %s, %s]" % (data.linear.x, data.linear.y, data.linear.z))
    rospy.loginfo("Angular Components: [%s, %s, %s]" % (data.angular.x, data.angular.y, data.angular.z))

    # Move the turtle based on the received message
    move_turtle(data)

def listener():
    """
    Function to initialize the ROS node and subscribe to the /turtle1/cmd_vel topic.
    """
    # Initialize the node
    rospy.init_node('turtlesim_listener', anonymous=True)

    # Subscribe to the /turtle1/cmd_vel topic
    rospy.Subscriber("/turtle1/cmd_vel", Twist, callback)

    # Keep the node running until it's shut down
    rospy.spin()

if __name__ == '__main__':
    listener()

