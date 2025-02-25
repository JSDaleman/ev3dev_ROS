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
from nav_msgs.msg import Odometry

class DifferentialInverseKinematics():
    
    def __init__(self, wheel_radius=0.028 , robot_track=0.12):

        self.wheel_radius = wheel_radius# m
        self.robot_track = robot_track # m

    def compute_wheel_velocities(self, linear_x, angular_z):

        """
        Calcula las velocidades de las ruedas izquierda y derecha
        a partir de la velocidad lineal y angular del robot.

        v_l = (v - ωL/2) / R
        v_r = (v + ωL/2) / R
        """
        v_left = (linear_x - (angular_z * self.robot_track / 2)) / self.wheel_radius
        v_right = (linear_x + (angular_z * self.robot_track / 2)) / self.wheel_radius
        v_left = round(v_left, 3)
        v_right = round(v_right, 3)
        return v_left, v_right


class InverseKinematicsNode:
    def __init__(self):
        rospy.init_node("inverse_kinematics_node", anonymous=True)
        self.kinematics = DifferentialInverseKinematics()

        self.lego_id = rospy.get_param('/robot_id', 1)
        rospy.set_param('/robot_name', self.lego_id)
        self.robot_name = f"LegoEV3{self.lego_id:02d}"

        # Publicador para enviar velocidades de las ruedas
        self.publish_topic_velocities = f"{self.robot_name}/wheels_vel_kinematic"
        self.pub_wheel_velocities = rospy.Publisher(self.publish_topic_velocities, JointState, queue_size=10)

        # Suscriptor que recibe comandos de velocidad
        self.subscription_topic_cmd_vel = f"{self.robot_name}/cmd_vel"
        self.subscription_topic_odom = f"{self.robot_name}/odom"
        self.sub_cmd_vel = rospy.Subscriber(self.subscription_topic_cmd_vel, Twist, self.cmd_vel_callback)
        self.odom_sub = rospy.Subscriber(self.subscription_topic_odom, Odometry, self.odom_callback)

        rospy.loginfo("Inverse kinematics node initialized")

    def robot_vel_received(self, msg):
        # Acceder a las velocidades lineales y angulares
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        linear_z = msg.linear.z
        angular_x = msg.angular.x
        angular_y = msg.angular.y
        angular_z = msg.angular.z

        # Log de los datos recibidos
        #rospy.loginfo(f"Linear velocity - x: {linear_x}, y: {linear_y}, z: {linear_z}")
        #rospy.loginfo(f"Angular velocity - x: {angular_x}, y: {angular_y}, z: {angular_z}")

        # Crear una respuesta (puedes modificar esto según tus necesidades)
        response = f"Response: Linear velocities (x: {linear_x}, y: {linear_y}, z: {linear_z}), " \
                f"Angular velocities (x: {angular_x}, y: {angular_y}, z: {angular_z})"

        # Publicar la respuesta en un tópico
        #self.pub_message.publish(response)
    
        # Log de la publicación
        rospy.loginfo(f"Published: {response}")

    def cmd_vel_callback(self, msg):
        """ Callback que convierte cmd_vel en velocidades de rueda y publica el resultado """
        v_left, v_right = self.kinematics.compute_wheel_velocities(msg.linear.x, msg.angular.z)

        # Crear mensaje JointState con las velocidades de las ruedas
        wheel_msg = JointState()
        wheel_msg.name = ["wheel_left", "wheel_right"]
        wheel_msg.velocity = [v_left, v_right]

        # Publicar en el tópico
        self.pub_wheel_velocities.publish(wheel_msg)

        rospy.loginfo(f"Calculated speeds [rad/s] left: {v_left:.3f} right: {v_right:.3f}")

    def odom_callback(self, msg):
        linear_x = msg.twist.twist.linear.x
        angular_z = msg.twist.twist.angular.z

        v_left, v_right = self.kinematics.compute_wheel_velocities(linear_x, angular_z)
        
        # Crear y publicar mensaje JointState
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = ["wheel_left", "wheel_right"]
        joint_msg.velocity = [v_left,  v_right]
        #self.joint_pub.publish(joint_msg)
        
        rospy.loginfo(f"Published wheels_vel[rad/s]: left:{v_left:.3f}, right:{ v_right:.3f}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = InverseKinematicsNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Inverse kinematics node interrupted")
    