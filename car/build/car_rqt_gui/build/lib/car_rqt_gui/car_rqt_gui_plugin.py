import os
import rclpy
from rclpy.node import Node
from rqt_gui_py.plugin import Plugin
from PyQt5.QtWidgets import QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QFrame
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QPalette, QColor

class CarRqtGuiPlugin(Plugin):

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('CarRqtGuiPlugin')
        
        # Initialize ROS2 node first
        if not rclpy.ok():
            rclpy.init()
        self._node = Node('car_rqt_gui_node')
        self._node.get_logger().info("CarRqtGuiPlugin ROS2 node initialized.")

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

        # Left side - RViz placeholder
        self.rviz_frame = self._create_rviz_placeholder()
        main_layout.addWidget(self.rviz_frame, stretch=3)

        # Right side - Control panel
        control_panel = self._create_control_panel()
        main_layout.addWidget(control_panel, stretch=1)

        # Add widget to the user interface
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def _create_rviz_placeholder(self):
        """Create a placeholder for RViz visualization"""
        frame = QFrame()
        frame.setStyleSheet("""
            QFrame {
                background-color: #2d2d2d;
                border: 2px solid #3d3d3d;
                border-radius: 8px;
            }
        """)
        
        layout = QVBoxLayout(frame)
        layout.setAlignment(Qt.AlignCenter)
        
        # Title
        title = QLabel("RViz Visualization")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #4CAF50; border: none;")
        layout.addWidget(title)
        
        # Placeholder text
        placeholder = QLabel("RViz view will appear here\n\n🤖")
        placeholder.setFont(QFont("Arial", 14))
        placeholder.setAlignment(Qt.AlignCenter)
        placeholder.setStyleSheet("color: #888888; border: none;")
        layout.addWidget(placeholder)
        
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
        self.button1 = QPushButton("Room 1")
        self.button1.setStyleSheet(button_style)
        self.button1.setFont(QFont("Arial", 16, QFont.Bold))
        self.button1.clicked.connect(self._on_button1_clicked)
        layout.addWidget(self.button1)
        
        # Room 2 button
        self.button2 = QPushButton("Room 2")
        self.button2.setStyleSheet(button_style.replace("#4CAF50", "#2196F3").replace("#45a049", "#1976D2").replace("#3d8b40", "#1565C0"))
        self.button2.setFont(QFont("Arial", 16, QFont.Bold))
        self.button2.clicked.connect(self._on_button2_clicked)
        layout.addWidget(self.button2)
        
        # Room 3 button
        self.button3 = QPushButton("Room 3")
        self.button3.setStyleSheet(button_style.replace("#4CAF50", "#FF9800").replace("#45a049", "#F57C00").replace("#3d8b40", "#E65100"))
        self.button3.setFont(QFont("Arial", 16, QFont.Bold))
        self.button3.clicked.connect(self._on_button3_clicked)
        layout.addWidget(self.button3)
        
        # Room 4 button
        self.button4 = QPushButton("Room 4")
        self.button4.setStyleSheet(button_style.replace("#4CAF50", "#9C27B0").replace("#45a049", "#7B1FA2").replace("#3d8b40", "#6A1B9A"))
        self.button4.setFont(QFont("Arial", 16, QFont.Bold))
        self.button4.clicked.connect(self._on_button4_clicked)
        layout.addWidget(self.button4)
        
        # Log display section
        log_label = QLabel("Activity Log:")
        log_label.setFont(QFont("Arial", 12, QFont.Bold))
        log_label.setStyleSheet("color: #aaaaaa; border: none; margin-top: 20px; margin-bottom: 8px;")
        layout.addWidget(log_label)
        
        # Log text area
        self.log_display = QLabel("System ready...\nWaiting for commands...")
        self.log_display.setFont(QFont("Courier", 11))
        self.log_display.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.log_display.setWordWrap(True)
        self.log_display.setStyleSheet("""
            QLabel {
                color: #00ff00;
                background-color: #1a1a1a;
                border: 1px solid #3d3d3d;
                border-radius: 5px;
                padding: 10px;
                min-height: 180px;
                max-height: 180px;
            }
        """)
        self.log_display.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.log_display)
        
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

    def _on_button1_clicked(self):
        self._add_log("Moving to Room 1...")
        self._node.get_logger().info("Room 1 navigation requested")
        # TODO: Add ROS2 navigation goal here

    def _on_button2_clicked(self):
        self._add_log("Moving to Room 2...")
        self._node.get_logger().info("Room 2 navigation requested")
        # TODO: Add ROS2 navigation goal here

    def _on_button3_clicked(self):
        self._add_log("Moving to Room 3...")
        self._node.get_logger().info("Room 3 navigation requested")
        # TODO: Add ROS2 navigation goal here

    def _on_button4_clicked(self):
        self._add_log("Moving to Room 4...")
        self._node.get_logger().info("Room 4 navigation requested")
        # TODO: Add ROS2 navigation goal here

    def _on_emergency_stop(self):
        self._add_log("⚠️ EMERGENCY STOP!")
        self._node.get_logger().warn("Emergency stop activated!")
        # TODO: Add emergency stop command here

    def _add_log(self, message):
        """Add a message to the log display"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S")
        current_logs = self.log_display.text()
        
        # Keep only last 7 log entries (more space now)
        log_lines = current_logs.split('\n')
        if len(log_lines) >= 7:
            log_lines = log_lines[-6:]  # Keep last 6, add 1 new = 7 total
        
        log_lines.append(f"[{timestamp}] {message}")
        self.log_display.setText('\n'.join(log_lines))

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