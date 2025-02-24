#!/usr/bin/env python3

__author__ = "Juan Sebastian Daleman Martinez"
__copyright__ = "Copyright 2025, Ev3 ROS atravez de MQTT"
__license__ = "MIT"
__version__ = "0.0.4"
__maintainer__ = "Juan Sebastian Daleman Martinez"
__email__ = "jdaleman@unal.edu.co"
__status__ = "Development"

from PyQt5.QtWidgets import QFrame, QLabel, QLineEdit, QPushButton, QLayout, QGridLayout, QWidget,  QSizePolicy
import gui.styles.styles as styles


class BaseFrame(QFrame):

    def __init__(self, parent: QWidget, padding=20):
        super().__init__(parent)

        self.layout = QGridLayout()
        self.setLayout(self.layout)
        self.layout.setContentsMargins(padding, padding, padding, padding)
        self.layout.setSizeConstraint(QLayout.SetMinimumSize)

        # Aplicar estilo al Frame
        self.style_frame = styles.StyleFrame()
        self.style_frame.apply_style(self)

    def create_label(self, text, row, col, row_span=1, col_span=1):
        label = QLabel(text)
        label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.layout.addWidget(label, row, col, row_span, col_span)
        return label

    def create_entry(self, row, col, text="", width=8, row_span=1, col_span=1):
        entry = QLineEdit()
        entry.setMaxLength(width)
        entry.setText(text)
        entry.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.layout.addWidget(entry, row, col, row_span, col_span)
        return entry

    def create_button(self, text, row, col, command, row_span=1, col_span=1):
        button = QPushButton(text)
        button.clicked.connect(command)
        button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.layout.addWidget(button, row, col, row_span, col_span)
        return button

    def get_widgets_by_type(self, widget_type):
        """Devuelve una lista de widgets de un tipo específico dentro del layout."""
        return [self.layout.itemAt(i).widget() for i in range(self.layout.count())
                if isinstance(self.layout.itemAt(i).widget(), widget_type)]

    def get_buttons(self):
        return self.get_widgets_by_type(QPushButton)

    def get_entries(self):
        return self.get_widgets_by_type(QLineEdit)

    def get_labels(self):
        return self.get_widgets_by_type(QLabel)

    def style_buttons(self):
        style_button = styles.StyleButton()
        for button in self.get_buttons():
            style_button.apply_style(button)

    def style_labels(self):
        style_label = styles.StyleLabel()
        for label in self.get_labels():
            style_label.apply_style(label)

    def style_entries(self):
        style_entry = styles.StyleEntry()
        for entry in self.get_entries():
            style_entry.apply_style(entry)

    

