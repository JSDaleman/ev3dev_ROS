#!/usr/bin/env python3

__author__ = "Juan Sebastian Daleman Martinez"
__copyright__ = "Copyright 2025, Ev3 ROS a través de MQTT"
__license__ = "MIT"
__version__ = "0.0.1"
__maintainer__ = "Juan Sebastian Daleman Martinez"
__email__ = "jdaleman@unal.edu.co"
__status__ = "Development"

import tf
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist

class DifferentialForwardKinematics:

    def __init__(self, wheel_radius=0.028 , robot_track=0.12):
        
        self.wheel_radius = wheel_radius# m
        self.robot_track = robot_track # m

    def compute_robot_velocity(self, v_left, v_right):
        """
        Calcula la velocidad lineal y angular del robot en base a las velocidades de las ruedas.
        
        v = (v_r + v_l) / 2 * R
        ω = (v_r - v_l) / L * R
        """
        linear_x = (self.wheel_radius / 2) * (v_right + v_left)
        angular_z = (self.wheel_radius /self.robot_track) * (v_right - v_left)
        linear_x = round(linear_x, 3)
        angular_z = round(angular_z, 3)
        return linear_x, angular_z

class ForwardKinematicsNode:
    def __init__(self):
        rospy.init_node("forward_kinematics_node", anonymous=True)
        self.kinematics = DifferentialForwardKinematics()

        self.lego_id = rospy.get_param('/robot_id', 1)
        rospy.set_param('/robot_name', self.lego_id)
        self.robot_name = f"LegoEV3{self.lego_id:02d}"

        # Publicadores
        self.publish_topic_odometry = f"{self.robot_name}/odom_kinematic"
        self.publish_topic_cmd_vel = f"{self.robot_name}/cmd_vel_kinematic"
        self.pub_odometry = rospy.Publisher(self.publish_topic_odometry, Odometry, queue_size=10)
        self.pub_cmd_vel = rospy.Publisher(self.publish_topic_cmd_vel, Twist, queue_size=10)

        # Suscriptor de velocidades de ruedas
        self.subscribe_topic_wheels_vel = f"{self.robot_name}/wheels_vel"
        self.sub_wheel_velocities = rospy.Subscriber(self.subscribe_topic_wheels_vel, JointState, self.wheel_vel_callback)

        # Variables de posición y orientación
        self.x = 0.0  # Posición en x
        self.y = 0.0  # Posición en y
        self.theta = 0.0  # Orientación en radianes

        self.last_time = rospy.Time.now()

        rospy.loginfo("Forward kinematics node initialized")

    def wheel_vel_callback(self, msg):
        """ Callback que convierte velocidades de ruedas en odometría """
        try:
            # Obtener velocidades de las ruedas
            wheel_left_index = msg.name.index("wheel_left")
            wheel_right_index = msg.name.index("wheel_right")

            v_left = msg.velocity[wheel_left_index]
            v_right = msg.velocity[wheel_right_index]

            # Calcular velocidades del robot
            linear_x, angular_z = self.kinematics.compute_robot_velocity(v_left, v_right)

            # Integrar posición usando ecuaciones de movimiento
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            self.last_time = current_time

            delta_x = linear_x * np.cos(self.theta) * dt
            delta_y = linear_x * np.sin(self.theta) * dt
            delta_theta = angular_z * dt

            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            # Publicar mensaje de odometría
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time
            odom_msg.header.frame_id = "odom_control"
            odom_msg.child_frame_id = "base_link_control"

            # Posición
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.theta)
            odom_msg.pose.pose.orientation = Quaternion(*quaternion)

            # Velocidades
            odom_msg.twist.twist.linear.x = linear_x
            odom_msg.twist.twist.angular.z = angular_z

            self.pub_odometry.publish(odom_msg)
            
            # Crear y publicar mensaje Twist
            twist_msg = Twist()
            twist_msg.linear.x = linear_x
            twist_msg.angular.z = angular_z
            self.pub_cmd_vel.publish(twist_msg)

            #rospy.loginfo(f"Odometría -> x: {self.x:.2f}, y: {self.y:.2f}, θ: {self.theta:.2f} rad")
            rospy.loginfo(f"Published cmd_vel: linear={linear_x:.3f}, angular={angular_z:.3f}")

        except ValueError:
            rospy.logwarn("Wheel names not found in JointState message")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = ForwardKinematicsNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Direct kinematics node interrupted")
