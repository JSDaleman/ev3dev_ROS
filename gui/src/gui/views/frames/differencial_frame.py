#!/usr/bin/env python3

__author__ = "Juan Sebastian Daleman Martinez"
__copyright__ = "Copyright 2025, Ev3 ROS atravez de MQTT"
__credits__ = ["David Fisher"]
__license__ = "MIT"
__version__ = "0.0.4"
__maintainer__ = "Juan Sebastian Daleman Martinez"
__email__ = "jdaleman@unal.edu.co"
__status__ = "Development"

from PyQt5.QtCore import Qt, QCoreApplication
from PyQt5.QtGui import QKeyEvent
import gui.views.frames.frame as frame
import gui.controls.buttons as buttons_controls
import gui.controls.keys as keys_controls


class DifferencialFrame(frame.BaseFrame):

    def __init__(self, parent, padding=20):
        super().__init__(parent, padding)

        self.buttons_controls = buttons_controls.DifferentialControlButtons()
        self.keys_controls = keys_controls.DifferentialControlKeys()
        self.setFocusPolicy(Qt.StrongFocus)

        self.create_widgets()

        self.style_buttons()
        self.style_labels()
        self.style_entries()

        #self.keys_controls.set_keys_control()

    def create_widgets(self):

        self._create_title()        
        self._create_vel_wheel_left()
        self._create_vel_wheel_right()
        self._crate_angle()
        self._create_buttons()

    def _create_title(self):
        """ Creación del label con titulo o texto para el usuario"""
        self.title_label = self.create_label("Control diff robot VW [rpm]", 0, 0, 1, 4)

    def _create_vel_wheel_left(self):
        """ Creación de cuadro de texto para obtener valor de velocidad 
            del motor izquierdo y etiqueta descriptiva para el usuario """
        self.left_speed_label = self.create_label("Left", 1, 0)
        self.left_speed_entry = self.create_entry(2, 0, text="20")
        self.buttons_controls.set_left_speed_entry(self.left_speed_entry)
        self.keys_controls.set_left_speed_entry(self.left_speed_entry)

    def _create_vel_wheel_right(self):
        """ Creación de cuadro de texto para obtener valor de velocidad 
            del motor derecho y etiqueta descriptiva para el usuario """
        self.right_speed_label = self.create_label("Right", 1, 2)
        self.right_speed_entry = self.create_entry(2, 2, text="20")
        self.buttons_controls.set_right_speed_entry(self.right_speed_entry)
        self.keys_controls.set_right_speed_entry(self.right_speed_entry)

    def _crate_angle(self):
        """ Creación de label para presentar el valor de ángulo recibido 
            y declaración en el objeto delegate del label que recibe dicho mensaje """
        self.angle_label = self.create_label("Angle", 1, 3)
        self.angle_value_label = self.create_label("0", 2, 3)
        self.buttons_controls.set_angle_value_label(self.angle_value_label)
        self.keys_controls.set_angle_value_label(self.angle_value_label)

    def _create_buttons(self):
        """ Creación de los botones """
        self.angle_button = self.create_button("Update angle", 3, 3, self.buttons_controls.send_angle_message)
        self.forward_button = self.create_button("Forward", 3, 1, self.buttons_controls.send_forward_message)
        self.left_button = self.create_button("Left", 4, 0, self.buttons_controls.send_left_message)
        self.stop_button = self.create_button("Stop", 4, 1, self.buttons_controls.send_stop_message)
        self.right_button = self.create_button("Right", 4, 2, self.buttons_controls.send_right_message)
        self.back_button = self.create_button("Back", 5, 1, self.buttons_controls.send_back_message)
        self.q_button = self.create_button("Quit", 6, 2, self.buttons_controls.send_quit_message)
        self.e_button = self.create_button("Exit", 6, 0, self.buttons_controls.send_exit)

    def keyPressEvent(self, event):
        QCoreApplication.postEvent(self.keys_controls, QKeyEvent(event))


class DifferencialFrameWithArm(DifferencialFrame):

    def __init__(self, parent, padding=20):
        super().__init__(parent, padding)

        self.keys_controls.set_arm()

    def _create_buttons(self):
        """ Creación de los botones """
        self.angle_button = self.create_button("Update angle", 3, 3, self.buttons_controls.send_angle_message)
        self.forward_button = self.create_button("Forward", 3, 1, self.buttons_controls.send_forward_message)
        self.left_button = self.create_button("Left", 4, 0, self.buttons_controls.send_left_message)
        self.stop_button = self.create_button("Stop", 4, 1, self.buttons_controls.send_stop_message)
        self.right_button = self.create_button("Right", 4, 2, self.buttons_controls.send_right_message)
        self.back_button = self.create_button("Back", 5, 1, self.buttons_controls.send_back_message)
        self.up_button = self.create_button("Up", 6, 0, self.buttons_controls.send_up_message)
        self.down_button = self.create_button("Down", 7, 0, self.buttons_controls.send_down_message)
        self.q_button = self.create_button("Quit", 6, 2, self.buttons_controls.send_quit_message)
        self.e_button = self.create_button("Exit", 7, 2, self.buttons_controls.send_exit)
        
