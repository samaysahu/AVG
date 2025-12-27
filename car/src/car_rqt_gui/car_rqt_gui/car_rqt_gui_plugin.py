import os
import rclpy
from rclpy.node import Node
from rqt_gui_py.plugin import Plugin
from PyQt5.QtWidgets import QWidget, QPushButton, QVBoxLayout, QLabel

class CarRqtGuiPlugin(Plugin):

    def __init__(self):
        super().__init__()
        self.setObjectName('CarRqtGuiPlugin')
        self._widget = None
        self._node = None # ROS2 node for publishing/subscribing

    def init_module(self, context):
        # Initialize ROS2 node
        if not rclpy.ok():
            rclpy.init()
        self._node = Node('car_rqt_gui_node')
        self._node.get_logger().info("CarRqtGuiPlugin ROS2 node initialized.")

        # Create QWidget
        self._widget = QWidget()
        # Create a vertical layout
        layout = QVBoxLayout(self._widget)

        # Add a label for instructions/status
        self.status_label = QLabel("Car Control Panel", self._widget)
        layout.addWidget(self.status_label)

        # Add four buttons
        self.button1 = QPushButton("Room 1", self._widget)
        self.button1.clicked.connect(self._on_button1_clicked)
        layout.addWidget(self.button1)

        self.button2 = QPushButton("Room 2", self._widget)
        self.button2.clicked.connect(self._on_button2_clicked)
        layout.addWidget(self.button2)

        self.button3 = QPushButton("Room 3", self._widget)
        self.button3.clicked.connect(self._on_button3_clicked)
        layout.addWidget(self.button3)

        self.button4 = QPushButton("Room 4", self._widget)
        self.button4.clicked.connect(self._on_button4_clicked)
        layout.addWidget(self.button4)

        # Add widget to the user interface
        context.add_widget(self._widget)

    def _on_button1_clicked(self):
        self.status_label.setText("Room 1 clicked!")
        self._node.get_logger().info("Room 1 clicked!")
        # TODO: Add ROS2 communication here, e.g., publish a Twist message

    def _on_button2_clicked(self):
        self.status_label.setText("Room 2 clicked!")
        self._node.get_logger().info("Room 2 clicked!")
        # TODO: Add ROS2 communication here

    def _on_button3_clicked(self):
        self.status_label.setText("Room 3 clicked!")
        self._node.get_logger().info("Room 3 clicked!")
        # TODO: Add ROS2 communication here

    def _on_button4_clicked(self):
        self.status_label.setText("Room 4 clicked!")
        self._node.get_logger().info("Room 4 clicked!")
        # TODO: Add ROS2 communication here

    def shutdown_module(self):
        # Destroy the ROS2 node
        if self._node:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self._node.get_logger().info("CarRqtGuiPlugin ROS2 node shut down.")

    def save_settings(self, plugin_settings, instance_settings):
        # TODO: Save any plugin-specific settings here
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO: Restore any plugin-specific settings here
        pass
