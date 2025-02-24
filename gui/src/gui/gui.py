#!/usr/bin/env python3

__author__ = "Juan Sebastian Daleman Martinez"
__copyright__ = "Copyright 2025, Ev3 ROS atravez de MQTT"
__credits__ = ["David Fisher", "FCruzV10"]
__license__ = "MIT"
__version__ = "0.0.4"
__maintainer__ = "Juan Sebastian Daleman Martinez"
__email__ = "jdaleman@unal.edu.co"
__status__ = "Development"

import sys
import rospy
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QMessageBox
import gui.views.frames.differencial_frame as differencial_frame
import gui.styles.styles as styles


class GUI(QMainWindow):

    def __init__(self):
        super().__init__()

        # Inicializar nodo de ROS
        rospy.init_node("gui_node", anonymous=True)

        # Creación de ventana principal del GUI y título
        self.setWindowTitle("Control EV3 with MQTT messages")
        self.setGeometry(100, 100, 400, 300)
        self.setMinimumSize(300, 200)

        # Aplicar estilo al MainWindow
        self.style_main_window = styles.StyleMainWindow()
        self.style_main_window.apply_style(self)

        # Widget central
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Layout principal
        layout = QVBoxLayout()
        central_widget.setLayout(layout)

        # Creación de frame diferencial
        self.diff_frame = differencial_frame.DifferencialFrame(self)
        layout.addWidget(self.diff_frame)

    def resizeEvent(self, event):
        """Evento que se ejecuta cuando la ventana cambia de tamaño"""
        super().resizeEvent(event)
        self.update_font_size()  # Actualizar fuente al cambiar tamaño0000

    def update_font_size(self):
        """Calcula el tamaño de fuente dinámico basado en el ancho de la ventana"""
        nuevo_tamano = max(min(self.width(), self.height()) // 30, 12)  # Ajuste proporcional al ancho

        # Aplicar el nuevo tamaño de fuente a todos los widgets
        for widget in self.findChildren(QWidget):  
            if hasattr(widget, "setFont"):  # Verifica que el widget soporta setFont
                fuente = widget.font()
                fuente.setPointSize(nuevo_tamano)
                widget.setFont(fuente)

    def closeEvent(self, event):
        """ Controla el cierre de la ventana (X) """
        # Cierre del cliente MQTT al finalizar
        rospy.loginfo("Closing GUI...")
        rospy.signal_shutdown("GUI closure")  # Apagar nodo ROS
        event.accept()

class GUIRQT(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)

         # Inicializar nodo de ROS solo si aún no está inicializado
        if not rospy.core.is_initialized():
            rospy.init_node("gui_node", anonymous=True)

        # Creación de ventana principal del GUI y título
        self.setWindowTitle("Control EV3 with MQTT messages")
        self.setGeometry(100, 100, 400, 300)
        self.setMinimumSize(300, 200)

        # Aplicar estilo al MainWindow
        self.style_main_window = styles.StyleMainWindow()
        self.style_main_window.apply_style(self)

        # Layout principal
        layout = QVBoxLayout()
        self.setLayout(layout)

        # Creación de frame diferencial
        self.diff_frame = differencial_frame.DifferencialFrame(self)
        layout.addWidget(self.diff_frame)

    def resizeEvent(self, event):
        """Evento que se ejecuta cuando la ventana cambia de tamaño"""
        super().resizeEvent(event)
        self.update_font_size()  # Actualizar fuente al cambiar tamaño0000

    def update_font_size(self):
        """Calcula el tamaño de fuente dinámico basado en el ancho de la ventana"""
        nuevo_tamano = max(min(self.width(), self.height()) // 30, 12)  # Ajuste proporcional al ancho

        # Aplicar el nuevo tamaño de fuente a todos los widgets
        for widget in self.findChildren(QWidget):  
            if hasattr(widget, "setFont"):  # Verifica que el widget soporta setFont
                fuente = widget.font()
                fuente.setPointSize(nuevo_tamano)
                widget.setFont(fuente)

    def closeEvent(self, event):
        """ Controla el cierre de la ventana (X) """
        # Cierre del cliente MQTT al finalizar
        rospy.loginfo("Closing GUI...")
        rospy.signal_shutdown("GUI closure") # Apagar nodo ROS
        event.accept()

if __name__ == '__main__':

    app = QApplication(sys.argv)
    window = GUI()
    window.show()

    try:
        rospy.loginfo("Start GUI...")
        sys.exit(app.exec_())  # Iniciar el loop de eventos de PyQt5

    except rospy.ROSInterruptException:
        rospy.logwarn("Node GUI interrupted")
