import sys
import rclpy
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel
from car_rqt_gui.car_rqt_gui_plugin import CarRqtGuiPlugin

def main():
    # Initialize QApplication
    app = QApplication(sys.argv)

    # Initialize ROS2 (needed for the plugin's internal node)
    if not rclpy.ok():
        rclpy.init()

    # Create a dummy context for the plugin
    # In standalone mode, we just show the widget directly
    class DummyContext:
        def add_widget(self, widget):
            widget.show()

    # Instantiate the plugin
    # The __init__ method now expects a context
    plugin = CarRqtGuiPlugin()

    # The plugin's init_module is where the widget is actually set up and added
    # We need to call it manually for standalone testing
    plugin.init_module(DummyContext())

    # Start the Qt event loop
    sys.exit(app.exec_())

    # Shutdown ROS2
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
