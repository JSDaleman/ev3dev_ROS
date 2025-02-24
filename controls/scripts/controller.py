#!/usr/bin/env python3

__author__ = "Juan Sebastian Daleman Martinez"
__copyright__ = "Copyright 2025, Ev3 ROS a través de MQTT"
__license__ = "MIT"
__version__ = "0.0.1"
__maintainer__ = "Juan Sebastian Daleman Martine"
__email__ = "jdaleman@unal.edu.co"
__status__ = "Development"

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class Controller():

    def __init__(self):
        rospy.init_node("controller_node", anonymous=True)

        self.lego_id = rospy.get_param('/robot_id', 1)
        rospy.set_param('/robot_name', self.lego_id)
        self.robot_name = f"LegoEV3{self.lego_id:02d}"

        self.publish_topic_wheels_vel = f"{self.robot_name}/wheels_vel"
        self.publish_topic_command = f"{self.robot_name}/command"

        self.pub_wheels_vel = rospy.Publisher(self.publish_topic_wheels_vel, JointState, queue_size=10)
        self.pub_command = rospy.Publisher(self.publish_topic_command, String, queue_size=10)

        self.subscribe_topic_drive_control = f"{self.robot_name}/drive_control"
        self.subscribe_topic_vel_control = f"{self.robot_name}/vel_control"
        self.subscribe_topic_command_control = f"{self.robot_name}/command_control"

        self.sub_drive = rospy.Subscriber(self.subscribe_topic_drive_control, JointState, self.drive_control_callback)
        self.sub_vel = rospy.Subscriber(self.subscribe_topic_vel_control, Twist, self.vel_control_callback)
        self.sub_command = rospy.Subscriber(self.subscribe_topic_command_control, String, self.command_control_callback)

        rospy.loginfo("Controller Node Initialized")
        self.rate = rospy.Rate(1) 

    def drive_control_callback(self, msg):
        self.pub_wheels_vel.publish(msg)
        #rospy.loginfo(f"Wheels velocity: {msg}")

    def vel_control_callback(self, msg):
        rospy.loginfo(f"velocities: {msg}")

    def command_control_callback(self, msg):
        self.pub_command.publish(msg)
        #rospy.loginfo(f"Command: {msg}")

    def run(self):
        rospy.spin()

        #while not rospy.is_shutdown():
        #    self.rate.sleep()

if __name__ == '__main__':
    try:
        node = Controller()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Controller Node interrupted")