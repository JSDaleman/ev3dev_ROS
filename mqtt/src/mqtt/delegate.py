#!/usr/bin/env python3

__author__ = "Juan Sebastian Daleman Martinez"
__copyright__ = "Copyright 2025, Ev3 ROS atravez de MQTT"
__credits__ = ["David Fisher"]
__license__ = "MIT"
__version__ = "0.0.3"
__maintainer__ = "Juan Sebastian Daleman Martinez"
__email__ = "jdaleman@unal.edu.co"
__status__ = "Development"

import rospy

class PcROSDelegate(object):

    """Clase para procesar mensajes MQTT recibidos y publicar datos en ROS."""

    def __init__(self):

        # Publicador para los datos del girosensor
        self.gyro_publisher = None

    def set_gyro_publisher(self, publisher):
        """Asigna el publicador para los datos del girosensor."""
        self.gyro_publisher = publisher

    def get_gyro_publisher(self):
        """retorn el publicador para los datos del girosensor."""
        return self.gyro_publisher

    def print_message(self, message):
        """Procesa mensajes de tipo 'print_message'."""
        rospy.loginfo(f"Message received: {message}")

    def Angle(self, angle):
        """Procesa mensajes de tipo 'Angle' y los publica."""
        if self.gyro_publisher:
            self.gyro_publisher.publish(str(angle))
            rospy.loginfo(f"Angle received and published: {angle}")
        else:
            rospy.logwarn("Gyro publisher is not set. Angle data was not published.")