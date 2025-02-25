#!/usr/bin/env python3

__author__ = "Juan Sebastian Daleman Martinez"
__copyright__ = "Copyright 2025, Ev3 ROS atravez de MQTT"
__license__ = "MIT"
__version__ = "0.0.4"
__maintainer__ = "Juan Sebastian Daleman Martinez"
__email__ = "jdaleman@unal.edu.co"
__status__ = "Development"

import sys
import rospy
from PyQt5.QtWidgets import QLineEdit, QLabel
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from gui.controls.units import ConverterUnits


class ControlButtons():
    def __init__(self):
        self.publisher = None

    def set_publisher_buttons_commands(self, publisher):
        self.publisher = publisher

class DifferentialControlButtons(ControlButtons):

    def __init__(self, Robot_ID=1):
        super().__init__()
        self.right_speed_entry: QLineEdit = None
        self.left_speed_entry: QLineEdit = None
        self.angle_value_label: QLabel = None

        self.lego_id = rospy.get_param('/robot_id', 1)
        rospy.set_param('/robot_name', self.lego_id)
        self.robot_name = f"LegoEV3{self.lego_id:02d}"

        self.convert_units = ConverterUnits()

        self.publish_topic_drive_control = f"{self.robot_name}/drive_control"
        self.publish_topic_vel_control = f"{self.robot_name}/vel_control"
        self.publish_topic_command_control = f"{self.robot_name}/command_control"

        self.pub_drive = rospy.Publisher(self.publish_topic_drive_control, JointState, queue_size=10)
        self.pub_vel = rospy.Publisher(self.publish_topic_vel_control, Twist, queue_size=10)
        self.pub_command = rospy.Publisher(self.publish_topic_command_control, String, queue_size=10)

        self.subscription_topic_gyro = f"{self.robot_name}/gyro_sensor"
        self.sub_gyro = rospy.Subscriber(self.subscription_topic_gyro, String, self.gui_angle_received)

    def set_right_speed_entry(self, right_speed_entry: QLineEdit):
        self.right_speed_entry = right_speed_entry
    
    def set_left_speed_entry(self, left_speed_entry: QLineEdit):
        self.left_speed_entry = left_speed_entry

    def set_angle_value_label(self, label: QLabel):
        self.angle_value_label = label

    def gui_angle_received(self, msg):
        angle = msg.data
        self.angle_value_label.setText(str(angle))
    
    def send_angle_message(self):
        self._send_command_message("angle", "angle button")

    def send_forward_message(self):
        self._send_movement_message(1, 1, "Forward button")

    def send_left_message(self):
        self._send_movement_message(-1, 1, "Left button")

    def send_right_message(self):
        self._send_movement_message(1, -1, "Right button")

    def send_back_message(self):
        self._send_movement_message(-1, -1, "Back button")

    def send_stop_message(self):
        self._send_command_message("stop", "Stop button")

        speed = self.convert_units.rpm_to_rad_per_sec(0)

        msg = JointState()
        
        # Nombres de las ruedas
        msg.name = ["wheel_left", "wheel_right"]

        # Smulación de velocidades
        msg.velocity = [1 * speed, 1 * speed]  # Velocidades arbitrarias

        # Publicar mensaje
        rospy.loginfo(f"Publish speeds[rad/s]: left:{msg.velocity[0]}, right:{msg.velocity[1]} Action Stop button")
        self.pub_drive.publish(msg)

    def send_up_message(self):
        self._send_command_message("arm_up", "Up button")

    def send_down_message(self):
        self._send_command_message("arm_down", "Down button")

    def send_quit_message(self):
        self._send_command_message("quit", "Quit button")

    def send_exit(self):
        self._send_command_message("shutdown", "Exit key")
        sys.exit()

    def _send_movement_message(self, factor_left, factor_right, action_name):
        """ Función para enviar mensajes de movimiento con los valores de los inputs. """
        try:
            value_right = int(self.right_speed_entry.text())
            value_left = int(self.left_speed_entry.text())
            speed = min(max(min(value_right, value_left), 0), 600) #rpm
            self.left_speed_entry.setText(str(speed))
            self.right_speed_entry.setText(str(speed))

            speed = self.convert_units.rpm_to_rad_per_sec(speed)

            msg = JointState()
        
            # Nombres de las ruedas
            msg.name = ["wheel_left", "wheel_right"]

            # Simulación de velocidades
            msg.velocity = [factor_left * speed, factor_right * speed]  # Velocidades arbitrarias

            # Publicar mensaje
            rospy.loginfo(f"Publish speeds[rad/s]: left:{msg.velocity[0]}, right:{msg.velocity[1]} Action {action_name}")
            self.pub_drive.publish(msg)
            
        except ValueError:
            rospy.loginfo("Please enter a valid number")

    def _send_command_message(self, msg_command, action_name):
        rospy.loginfo(f"Action {action_name}")
        self.pub_command.publish(msg_command)
