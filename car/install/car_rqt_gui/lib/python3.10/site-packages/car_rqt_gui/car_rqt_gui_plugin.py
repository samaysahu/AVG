import os
import rclpy
from rclpy.node import Node
from rqt_gui_py.plugin import Plugin
from PyQt5.QtWidgets import QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QFrame, QScrollArea, QTextEdit
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QPalette, QColor

from std_msgs.msg import String  # Import String message type
from nav2_msgs.action import NavigateToPose # Import NavigateToPose action
from rclpy.action import ActionClient # Import ActionClient

class CarRqtGuiPlugin(Plugin):

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('CarRqtGuiPlugin')
        
        # Initialize ROS2 node first
        if not rclpy.ok():
            rclpy.init()
        self._node = Node('car_rqt_gui_node')
        self._node.get_logger().info("CarRqtGuiPlugin ROS2 node initialized.")

        # Create a publisher for room navigation commands
        self._room_publisher = self._node.create_publisher(String, 'navigate_room', 10)
        self._node.get_logger().info("Room navigation command publisher created.")

        # Create an action client for NavigateToPose to send cancel requests
        self._action_client = ActionClient(self._node, NavigateToPose, 'navigate_to_pose')
        self._node.get_logger().info("NavigateToPose action client created.")
        
        # Create main widget
        self._widget = QWidget()
        self._widget.setMinimumSize(900, 600)
        
        # Set dark theme styling
        self._widget.setStyleSheet("""
            QWidget {
                background-color: #1e1e1e;
                color: #ffffff;
                font-family: Arial, sans-serif;
            }
            QLabel {
                color: #ffffff;
            }
        """)
        
        # Create main horizontal layout
        main_layout = QHBoxLayout(self._widget)
        main_layout.setSpacing(15)
        main_layout.setContentsMargins(15, 15, 15, 15)

        # Left side - Large Activity Log
        self.log_frame = self._create_log_display()
        main_layout.addWidget(self.log_frame, stretch=3)

        # Right side - Control panel
        control_panel = self._create_control_panel()
        main_layout.addWidget(control_panel, stretch=1)

        # Add widget to the user interface
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
        
        # Add initial log message
        self._add_log("System initialized and ready")

    def _create_log_display(self):
        """Create a large log display area"""
        frame = QFrame()
        frame.setStyleSheet("""
            QFrame {
                background-color: #2d2d2d;
                border: 2px solid #3d3d3d;
                border-radius: 8px;
            }
        """)
        
        layout = QVBoxLayout(frame)
        layout.setSpacing(10)
        layout.setContentsMargins(15, 15, 15, 15)
        
        # Title
        title = QLabel("Activity Log")
        title.setFont(QFont("Arial", 18, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #4CAF50; border: none; margin-bottom: 10px;")
        layout.addWidget(title)
        
        # Log text area (using QTextEdit for better scrolling and text handling)
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setFont(QFont("Courier", 11))
        self.log_display.setStyleSheet("""
            QTextEdit {
                color: #00ff00;
                background-color: #1a1a1a;
                border: 1px solid #3d3d3d;
                border-radius: 5px;
                padding: 10px;
            }
        """)
        layout.addWidget(self.log_display)
        
        return frame

    def _create_control_panel(self):
        """Create the control panel with room buttons"""
        panel = QFrame()
        panel.setStyleSheet("""
            QFrame {
                background-color: #2d2d2d;
                border: 2px solid #3d3d3d;
                border-radius: 8px;
            }
        """)
        
        layout = QVBoxLayout(panel)
        layout.setSpacing(15)
        layout.setContentsMargins(20, 20, 20, 20)
        
        # Title
        title = QLabel("Robot Navigation")
        title.setFont(QFont("Arial", 18, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #4CAF50; border: none; margin-bottom: 20px;")
        layout.addWidget(title)
        
        # Subtitle
        subtitle = QLabel("Select Destination:")
        subtitle.setFont(QFont("Arial", 12, QFont.Bold))
        subtitle.setStyleSheet("color: #aaaaaa; border: none; margin-bottom: 10px;")
        layout.addWidget(subtitle)
        
        # Room buttons
        button_style = """
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                border-radius: 8px;
                padding: 12px;
                font-size: 16px;
                font-weight: bold;
                min-height: 45px;
                max-height: 45px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
        """
        
        # Room 1 button
        self.button1 = QPushButton("🏠 Room 1")
        self.button1.setStyleSheet(button_style)
        self.button1.setFont(QFont("Arial", 16, QFont.Bold))
        self.button1.clicked.connect(self._on_button1_clicked)
        layout.addWidget(self.button1)
        
        # Room 2 button
        self.button2 = QPushButton("🏠 Room 2")
        self.button2.setStyleSheet(button_style.replace("#4CAF50", "#2196F3").replace("#45a049", "#1976D2").replace("#3d8b40", "#1565C0"))
        self.button2.setFont(QFont("Arial", 16, QFont.Bold))
        self.button2.clicked.connect(self._on_button2_clicked)
        layout.addWidget(self.button2)
        
        # Room 3 button
        self.button3 = QPushButton("🏠 Room 3")
        self.button3.setStyleSheet(button_style.replace("#4CAF50", "#FF9800").replace("#45a049", "#F57C00").replace("#3d8b40", "#E65100"))
        self.button3.setFont(QFont("Arial", 16, QFont.Bold))
        self.button3.clicked.connect(self._on_button3_clicked)
        layout.addWidget(self.button3)
        
        # Room 4 button
        self.button4 = QPushButton("🏠 Room 4")
        self.button4.setStyleSheet(button_style.replace("#4CAF50", "#9C27B0").replace("#45a049", "#7B1FA2").replace("#3d8b40", "#6A1B9A"))
        self.button4.setFont(QFont("Arial", 16, QFont.Bold))
        self.button4.clicked.connect(self._on_button4_clicked)
        layout.addWidget(self.button4)

        # Home button
        self.button_home = QPushButton("🏡 Home")
        self.button_home.setStyleSheet(button_style.replace("#4CAF50", "#607D8B").replace("#45a049", "#455A64").replace("#3d8b40", "#37474F"))
        self.button_home.setFont(QFont("Arial", 16, QFont.Bold))
        self.button_home.clicked.connect(self._on_button_home_clicked)
        layout.addWidget(self.button_home)
        
        # Add stretch to push buttons to top
        layout.addStretch()
        
        # Emergency stop button at bottom
        stop_button = QPushButton("⚠️ EMERGENCY STOP")
        stop_button.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                border: none;
                border-radius: 8px;
                padding: 12px;
                font-size: 15px;
                font-weight: bold;
                min-height: 45px;
                max-height: 45px;
            }
            QPushButton:hover {
                background-color: #da190b;
            }
            QPushButton:pressed {
                background-color: #c41c0b;
            }
        """)
        stop_button.setFont(QFont("Arial", 15, QFont.Bold))
        stop_button.clicked.connect(self._on_emergency_stop)
        layout.addWidget(stop_button)
        
        return panel

    def _publish_room_command(self, room_key):
        msg = String()
        msg.data = room_key
        self._room_publisher.publish(msg)
        self._node.get_logger().info(f"Published command to navigate to room: {room_key}")

    def _on_button1_clicked(self):
        self._add_log("🚀 Navigation command: Moving to Room 1...")
        self._publish_room_command('1')

    def _on_button2_clicked(self):
        self._add_log("🚀 Navigation command: Moving to Room 2...")
        self._publish_room_command('2')

    def _on_button3_clicked(self):
        self._add_log("🚀 Navigation command: Moving to Room 3...")
        self._publish_room_command('3')

    def _on_button4_clicked(self):
        self._add_log("🚀 Navigation command: Moving to Room 4...")
        self._publish_room_command('4')

    def _on_button_home_clicked(self):
        self._add_log("🚀 Navigation command: Moving to Home...")
        self._publish_room_command('0')

    def _on_emergency_stop(self):
        self._add_log("⚠️ EMERGENCY STOP ACTIVATED! Attempting to cancel navigation goal.")
        self._node.get_logger().warn("Emergency stop activated!")
        
        # Publish 'stop' command to the room_waypoint node
        self._publish_room_command('stop')

    def _add_log(self, message):
        """Add a message to the log display"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        # Add timestamped message
        log_message = f"[{timestamp}] {message}"
        
        # Append to the text edit (automatically scrolls to bottom)
        self.log_display.append(log_message)
        
        # Auto-scroll to bottom
        scrollbar = self.log_display.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def shutdown_plugin(self):
        # Destroy the ROS2 node
        if self._node:
            self._node.destroy_node()
        self._node.get_logger().info("CarRqtGuiPlugin ROS2 node shut down.")

    def save_settings(self, plugin_settings, instance_settings):
        # TODO: Save any plugin-specific settings here
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO: Restore any plugin-specific settings here
        pass
