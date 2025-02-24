#!/usr/bin/env python3

__author__ = "Juan Sebastian Daleman Martinez"
__copyright__ = "Copyright 2025, Ev3 ROS atravez de MQTT"
__credits__ = ["ChoKasem"]
__license__ = "MIT"
__version__ = "0.0.1"
__maintainer__ = "Juan Sebastian Daleman Martinez"
__email__ = "jdaleman@unal.edu.co"
__status__ = "Development"

import rospy
from argparse import ArgumentParser
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from gui.gui import GUIRQT
from rqt_gui_py.plugin import Plugin

class ControlDifferentialRobotPlugin(Plugin):

    def __init__(self, context):
        super(ControlDifferentialRobotPlugin, self).__init__(context)
        
        # Give QObjects reasonable names
        self.setObjectName('ControlDifferentialRobotPlugin')

        # Process standalone plugin command-line arguments
        
        parser = ArgumentParser()
        # Add argument(s) to the parser.

        parser.add_argument("-q", 
                            "--quiet", 
                            action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())

        if not args.quiet:
            #print('arguments: ', args)
            #print('unknowns: ', unknowns)
            pass

        # Crear el widget contenedor
        self.widget = QWidget()
        self.widget.setGeometry(100, 100, 400, 300)
        self.widget.setMinimumSize(300, 200)
        self.layout = QVBoxLayout(self.widget)

        # Instanciar GUI y agregarla al layout
        self.gui = GUIRQT(self.widget)
        self.layout.addWidget(self.gui)
        self.widget.setLayout(self.layout)
        
        self.widget.setObjectName('ControlDifferentialRobot')
        if context.serial_number() > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Agregar widget al contexto de RQt
        context.add_widget(self.widget)

    def shutdown_plugin(self):
        rospy.loginfo("Closing plugin RQt...")
        if hasattr(self, "gui"):
            self.gui.close()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog