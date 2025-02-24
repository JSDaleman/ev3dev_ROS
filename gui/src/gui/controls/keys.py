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
from PyQt5.QtWidgets import QLineEdit, QWidget, QLabel
from PyQt5.QtGui import QKeyEvent
from PyQt5.QtCore import Qt
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from gui.controls.units import ConverterUnits


class ControlKeys(QWidget):
    def __init__(self):
        super().__init__()
        self.publisher = None

    def set_publisher_buttons_commands(self, publisher):
        self.publisher = publisher

class DifferentialControlKeys(ControlKeys):
    def __init__(self):
        super().__init__()
        self.right_speed_entry: QLineEdit = None
        self.left_speed_entry: QLineEdit = None
        self.angle_value_label: QLabel = None
       #self.setFocusPolicy(Qt.StrongFocus)

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

        self.key_map = {
            Qt.Key_Up: self.control_key_up,
            Qt.Key_Left: self.control_key_left,
            Qt.Key_Right: self.control_key_right,
            Qt.Key_Down: self.control_key_down,
            Qt.Key_A: lambda: self._send_command_message("Angle", "angle key"),
            Qt.Key_Space: lambda: self._send_command_message("Stop", "Stop key"),
            Qt.Key_Q: lambda: self._send_command_message("Quit", "Quit key"),
            Qt.Key_E: self.control_key_e
        }

    def set_arm(self):

        self.key_map[Qt.Key_U] = lambda: self._send_command_message("arm_up", "Up key")
        self.key_map[Qt.Key_J] = lambda: self._send_command_message("arm_down", "Down key")

    def set_right_speed_entry(self, right_speed_entry: QLineEdit):
        self.right_speed_entry = right_speed_entry

    def set_left_speed_entry(self, left_speed_entry: QLineEdit):
        self.left_speed_entry = left_speed_entry

    def set_angle_value_label(self, label: QLabel):
        self.angle_value_label = label

    def gui_angle_received(self, msg):
        angle = msg.data
        self.angle_value_label.setText(str(angle))

    def control_key_up(self):
        self._send_movement_message(1, 1, "Forward key")

    def control_key_left(self):
        self._send_movement_message(-1, 1, "Left key")

    def control_key_right(self):
        self._send_movement_message(1, -1, "Right key")

    def control_key_down(self):
        self._send_movement_message(-1, -1, "Back key")

    def control_key_e(self):
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
        rospy.loginfo(f"Actiona {action_name}")
        self.pub_command.publish(msg_command + " ")

        if msg_command == "Stop":

            speed = self.convert_units.rpm_to_rad_per_sec(0)

            msg = JointState()
        
            # Nombres de las ruedas
            msg.name = ["wheel_left", "wheel_right"]

            # Simulación de velocidades
            msg.velocity = [1 * speed, 1 * speed]  # Velocidades arbitrarias

            # Publicar mensaje
            rospy.loginfo(f"Publish speeds[rad/s]: left:{msg.velocity[0]}, right:{msg.velocity[1]} Action {action_name}")
            self.pub_drive.publish(msg)

    def keyPressEvent(self, event: QKeyEvent):
        """ Captura eventos de teclado y ejecuta la acción correspondiente. """
        if event.key() in self.key_map:
            self.key_map[event.key()]()
            event.accept()

