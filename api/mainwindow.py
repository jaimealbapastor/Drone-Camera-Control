import sys
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QAction,
    QStackedWidget,
    QVBoxLayout,
    QHBoxLayout,
    QWidget,
    QLabel,
    QDesktopWidget,
    QGroupBox,
    QPushButton,
)
from connected_devices_widget import ConnectedDevicesScreen
from camera_widget_timer import CameraScreen


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Drone Control Interface")

        # Set the window size to 16:9 aspect ratio
        h = 900
        w = h * 16 // 9
        screen_geometry = QDesktopWidget().screenGeometry()
        x_centered = (screen_geometry.width() - w) // 2
        y_centered = (screen_geometry.height() - h) // 2

        # Set the geometry of the window
        self.setGeometry(x_centered, y_centered, w, h)

        self.central_widget = QStackedWidget()
        self.setCentralWidget(self.central_widget)

        # Create menu bar
        self.menu_bar = self.menuBar()
        self.create_menus()

        # Create screens
        self.create_camera_screen()
        self.create_live_data_screen()
        self.create_parameters_screen()
        self.create_connected_devices_screen()
        
    def closeEvent(self, event):
        self.cameras_screen.cleanup()
        event.accept() # let the window close
        # event.ignore()

    def create_menus(self):
        # Create menu actions
        camera_action = QAction("Camera", self)
        camera_action.triggered.connect(self.show_camera_screen)

        live_data_action = QAction("Live Data", self)
        live_data_action.triggered.connect(self.show_live_data_screen)

        parameters_action = QAction("Parameters", self)
        parameters_action.triggered.connect(self.show_parameters_screen)

        connected_devices_action = QAction("Connected Devices", self)
        connected_devices_action.triggered.connect(self.show_connected_devices_screen)

        # Add actions to menu
        for action in [
            camera_action,
            live_data_action,
            parameters_action,
            connected_devices_action,
        ]:
            self.menu_bar.addAction(action)

    # Show screens

    def show_camera_screen(self):
        self.central_widget.setCurrentIndex(0)

    def show_live_data_screen(self):
        self.central_widget.setCurrentIndex(1)

    def show_parameters_screen(self):
        self.central_widget.setCurrentIndex(2)

    def show_connected_devices_screen(self):
        self.central_widget.setCurrentIndex(3)

    # Create screens

    def create_camera_screen(self):

        # https://stackoverflow.com/questions/57076105/how-to-display-a-cv2-video-inside-a-python-gui
        self.cameras_screen = CameraScreen()
        self.central_widget.addWidget(self.cameras_screen)

    def create_live_data_screen(self):
        live_data_screen = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Live Data Screen"))
        # Add live data widgets and functionality here
        live_data_screen.setLayout(layout)
        self.central_widget.addWidget(live_data_screen)

    def create_parameters_screen(self):
        parameters_screen = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Parameters Screen"))
        # Add parameter widgets and functionality here
        parameters_screen.setLayout(layout)
        self.central_widget.addWidget(parameters_screen)

    def create_connected_devices_screen(self):
        # connected_devices_screen = QWidget()
        connected_devices_screen = ConnectedDevicesScreen()
        self.central_widget.addWidget(connected_devices_screen)
