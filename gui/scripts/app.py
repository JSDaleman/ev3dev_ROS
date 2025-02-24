#!/usr/bin/env python3

__author__ = "Juan Sebastian Daleman Martine"
__copyright__ = "Copyright 2025, Ev3 ROS a través de MQTT"
__credits__ = ["David Fisher"]
__license__ = "MIT"
__version__ = "0.0.2"
__maintainer__ = "Juan Sebastian Daleman Martine"
__email__ = "jdaleman@unal.edu.co"
__status__ = "Development"

# Importación de las librerías necesarias
import sys
import rospy
from PyQt5.QtWidgets import QApplication
from gui.gui import GUI

# Ejecución del script como principal o como módulo
if __name__ == '__main__':
    try:
        # Inicialización de la GUI con PyQt5
        app = QApplication(sys.argv)
        ros_gui = GUI()
        ros_gui.show()
        
        # Ejecutar la aplicación
        sys.exit(app.exec_())

    except rospy.ROSInterruptException:
        rospy.logwarn("Node GUI interrupted.")
