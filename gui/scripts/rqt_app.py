#!/usr/bin/env python

__author__ = "Juan Sebastian Daleman Martinez"
__copyright__ = "Copyright 2025, Ev3 ROS atravez de MQTT"
__license__ = "MIT"
__version__ = "0.0.2"
__maintainer__ = "Juan Sebastian Daleman Martinez"
__email__ = "jdaleman@unal.edu.co"
__status__ = "Development"

import sys
import rospy
from rqt_gui.main import Main
from gui.rqt_plugin import ControlDifferentialRobotPlugin

if __name__ == '__main__':
    try:
        plugin = 'gui' #Nombre del paquete
        main = Main(filename=plugin)
        sys.exit(main.main(standalone=plugin))

    except rospy.ROSInterruptException:
        rospy.logwarn("Plugin GUI interrupted.")