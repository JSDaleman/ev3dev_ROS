#!/usr/bin/env python3

__author__ = "Juan Sebastian Daleman Martinez"
__copyright__ = "Copyright 2025, Ev3 ROS atravez de MQTT"
__license__ = "MIT"
__version__ = "0.0.4"
__maintainer__ = "Juan Sebastian Daleman Martinez"
__email__ = "jdaleman@unal.edu.co"
__status__ = "Development"

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QFrame, QMainWindow
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt
import sys

# Paletas de colores
# https://colorhunt.co/palette/f3f3e0133e87608bc1cbdceb
# https://colorhunt.co/palette/001f3f3a6d8c6a9ab0ead8b1

class BaseStyle:
    """Clase base para aplicar estilos personalizados a los widgets en PyQt5."""
    
    def apply_style(self, widget, style):
        widget.setStyleSheet(style)

class StyleMainWindow(BaseStyle):
    """Clase para aplicar estilos personalizados a la main window."""
    def apply_style(self, main_window: QMainWindow):
        style = """
            QMainWindow {
                background-color: #CBDCEB;
                border-radius: 5px;
                border: 1px solid #6A9AB0;
            }
        """
        super().apply_style(main_window, style)

class StyleWidget(BaseStyle):
    """Clase para aplicar estilos personalizados a la main window."""
    def apply_style(self, widget: QWidget):
        style = """
            QMainWindow {
                background-color: #CBDCEB;
                border-radius: 5px;
                border: 1px solid #6A9AB0;
            }
        """
        super().apply_style(widget, style)

class StyleFrame(BaseStyle):
    """Clase para aplicar estilos personalizados a los frames."""
    
    def apply_style(self, frame: QFrame):
        style = """
            QFrame {
                background-color: #CBDCEB;
                border: none;
                border-radius: 5px;
                /* border: 1px solid #133E87; */
            }
        """
        super().apply_style(frame, style)


class StyleButton(BaseStyle):
    """Clase para aplicar estilos personalizados a los botones."""

    def apply_style(self, button: QPushButton):
        button.setFont(QFont("Roboto", 12))
        style = """
            QPushButton {
                color: #EAD8B1;
                background-color: #6A9AB0;
                border: none;
                border-radius: 5px;
                padding: 5px;
            }
            QPushButton:hover {
                background-color: #3E5879;
            }
            QPushButton:pressed {
                background-color: #213555;
            }
        """
        super().apply_style(button, style)


class StyleLabel(BaseStyle):
    """Clase para aplicar estilos personalizados a las etiquetas."""

    def apply_style(self, label: QLabel):
        label.setFont(QFont("Roboto", 12, QFont.Bold))
        label.setAlignment(Qt.AlignCenter)
        style = """
            QLabel {
                color: #3E5879;
                background-color: #CBDCEB;
                padding: 5px;
                border: none;
                border-radius: 5px;
                text-align: center;
            }
        """
        super().apply_style(label, style)


class StyleEntry(BaseStyle):
    """Clase para aplicar estilos personalizados a los campos de entrada."""

    def apply_style(self, entry: QLineEdit):
        entry.setFont(QFont("Roboto", 12))
        entry.setAlignment(Qt.AlignCenter)
        style = """
            QLineEdit {
                color: #3E5879;
                background-color: #F3F3E0;
                border: 1px solid #133E87;
                padding: 5px;
                border-radius: 5px;
            }
            QLineEdit:focus {
                border: 2px solid #2980B9;
            }
            QLineEdit:disabled {
                color: #7F8C8D;
            }
        """
        super().apply_style(entry, style)


def main():
    # Crear la aplicación PyQt5
    app = QApplication(sys.argv)
    window = QWidget()
    window.setWindowTitle("Aplicación con Estilos Personalizados")
    window.setGeometry(100, 100, 400, 400)

    # Instanciar los estilos
    button_style = StyleButton()
    frame_style = StyleFrame()
    label_style = StyleLabel()
    entry_style = StyleEntry()

    # Crear el layout
    layout = QVBoxLayout()

    # Crear un frame y aplicar estilo
    frame = QFrame()
    frame_style.apply_style(frame)
    layout.addWidget(frame)

    # Crear un label y aplicar estilo
    label = QLabel("Etiqueta con estilo personalizado")
    label_style.apply_style(label)
    layout.addWidget(label)

    # Crear un entry y aplicar estilo
    entry = QLineEdit()
    entry_style.apply_style(entry)
    layout.addWidget(entry)

    # Crear un botón y aplicar estilo
    button = QPushButton("Haz clic aquí")
    button_style.apply_style(button)
    layout.addWidget(button)

    # Configurar la ventana
    window.setLayout(layout)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
