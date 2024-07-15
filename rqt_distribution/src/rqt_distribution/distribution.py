#!/usr/bin/env python3


from rqt_gui_py.plugin import Plugin
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel
from rqt_gui_py.plugin import Plugin

class Distribution(Plugin):
    def __init__(self, context):
        super(Distribution, self).__init__(context)
        self.setObjectName('Distribution')

        # GUI initialization code
        self._widget = MyWidget()
        print("MyWidget instance created")

        self._widget.show()


class MyWidget(QWidget):
    def __init__(self):
        super(MyWidget, self).__init__()

        print("MyWidget constructor called...")

        self.setStyleSheet("background-color: white;")

        # Create a layout
        layout = QVBoxLayout(self)

        # Create a button
        self.button = QPushButton('Click Me!', self)
        self.button.clicked.connect(self.on_button_click)

        # Create a label for text display
        self.label = QLabel('Hello, World!', self)

        # Add the button and label to the layout
        layout.addWidget(self.button)
        layout.addWidget(self.label)

         # Set the layout for the widget
        self.setLayout(layout)

    def on_button_click(self):
        self.label.setText('Button Clicked!')