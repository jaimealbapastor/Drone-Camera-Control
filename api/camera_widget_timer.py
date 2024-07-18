import cv2
import time
import json
import numpy as np

from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QLabel,
    QGroupBox,
    QHBoxLayout,
    QPushButton,
)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QObject, QThread, pyqtSignal, QTimer


class CameraLabel(QLabel):
    def __init__(self, index=0):
        super().__init__()
        self.index = index

    def display_frame(self, img: QImage):
        self.setPixmap(QPixmap.fromImage(img))


class CameraWorker(QObject):
    updated_frame = pyqtSignal(tuple)
    finished = pyqtSignal()

    def __init__(self, indexes: list[int], fps=30):
        super().__init__()
        self.cam_indexes = indexes
        self.caps = {}
        self.last_frame_times = {i: 0 for i in self.cam_indexes}

        self.cam_settings = None

        self.is_on = False
        self.force_stop = False

        self.fps = fps
        self.delay = 1 / self.fps

    def initialize_cameras(self):
        for cam_i in self.cam_indexes:
            if cam_i in self.caps and self.caps[cam_i].isOpened():
                continue

            print(f"Opening camera {cam_i} ... ", end="")
            pass
            try:
                cap = cv2.VideoCapture(cam_i)
                # cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
                if cap and cap.isOpened():
                    self.caps[cam_i] = cap
                    print("Done")
                else:
                    print("Fail")

            except Exception as e:
                print("Fail")
                print(e)

    def emit_updated_frame(self, index):
        cap = self.caps[index]
        try:
            if cap and cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    q_img = self.rectify_cam_frame(frame, index)
                    self.updated_frame.emit((index, q_img))
        except Exception as e:
            print(f"Failed to capture frame of camera {index}")
            print(e)

    def rectify_cam_frame(self, frame, index) -> QImage:
        
        settings = self.cam_settings.get(index, {})
        f_type = np.float32

        frame = frame.astype(f_type)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if "flip" in settings:
            frame = cv2.flip(frame, settings["flip"])

        if "brightness" in settings:
            matrix = np.ones(frame.shape, dtype=f_type) * settings["brightness"]
            frame = cv2.add(frame, matrix)

        if "contrast" in settings:
            frame = cv2.multiply(frame, settings["contrast"])

        if "bin_threshold" in settings:
            th = settings["bin_threshold"]
            _, frame = cv2.threshold(frame, th, 255, cv2.THRESH_BINARY)

        frame = np.clip(frame, 0, 255).astype(np.uint8)

        if "bin_adapt_threshold" in settings:
            th = settings["bin_adapt_threshold"][0]
            block_size = settings["bin_adapt_threshold"][1]
            frame = cv2.adaptiveThreshold(
                frame,
                255,
                cv2.ADAPTIVE_THRESH_MEAN_C,
                cv2.THRESH_BINARY,
                th,
                block_size,
            )
            
        text = f"Camera {index} - {frame.shape[1]}x{frame.shape[0]} - fps: {self.fps}"
        fontscale = 2
        fontface = cv2.FONT_HERSHEY_PLAIN
        fontcolor = (255, 255, 255)
        fontthickness = 2
        cv2.putText(frame, text, (10, 30), fontface, fontscale, fontcolor, fontthickness)

        q_img = QImage(
            frame.data,
            frame.shape[1],
            frame.shape[0],
            QImage.Format_Grayscale8,
        )
        return q_img

    def run(self):
        self.initialize_cameras()
        self.is_on = True
        while not self.force_stop:
            if self.is_on:
                for cam_i in self.cam_indexes:
                    if time.time() - self.last_frame_times[cam_i] > self.delay:
                        self.emit_updated_frame(cam_i)
                        self.last_frame_times[cam_i] = time.time()

        self.finished.emit()

    def cleanup(self):
        self.is_on = False
        self.force_stop = True
        for i, cap in self.caps.items():
            if cap and cap.isOpened():
                cap.release()
        self.finished.emit()

    def play(self):
        self.is_on = True

    def pause(self):
        self.is_on = False

    def set_cam_settings(self, cam_settings):
        self.cam_settings = cam_settings


class CameraScreen(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout()
        self.camera_layout = QHBoxLayout()

        self.cam_labels = {}
        self.thread = None
        self.create_cam_label(0)
        self.create_cam_label(1)
        self.create_cam_label(2)
        self.layout.addLayout(self.camera_layout)

        settings_groupbox = QGroupBox("Settings")
        settings_layout = QHBoxLayout()

        self.play_button = QPushButton("Play")
        self.play_button.clicked.connect(self.play_all)

        self.pause_button = QPushButton("Pause")
        self.pause_button.clicked.connect(self.pause_all)

        self.reload_settings_button = QPushButton("Reload Settings")
        self.reload_settings_button.clicked.connect(
            lambda: self.worker.set_cam_settings(self.load_settings())
        )

        settings_layout.addWidget(self.play_button)
        settings_layout.addWidget(self.pause_button)
        settings_layout.addWidget(self.reload_settings_button)

        settings_groupbox.setLayout(settings_layout)
        self.layout.addWidget(settings_groupbox)
        self.setLayout(self.layout)

    def create_cam_label(self, index=0):
        camera = CameraLabel(index)
        self.camera_layout.addWidget(camera)
        self.cam_labels[index] = camera
        return camera

    def update_cam_label(self, frame: tuple[int, QImage]):
        i, img = frame
        self.cam_labels[i].display_frame(img)

    def create_thread(self):
        self.thread = QThread()
        self.thread.daemon = True
        self.worker = CameraWorker(list(self.cam_labels.keys()), 10)
        self.worker.cam_settings = self.load_settings()
        self.worker.moveToThread(self.thread)

        self.thread.started.connect(self.worker.run)
        self.worker.updated_frame.connect(self.update_cam_label)
        self.worker.finished.connect(self.worker.cleanup)
        self.worker.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)

        return self.thread

    def play_all(self):
        self.play_button.setDisabled(True)
        if self.thread is None:
            self.pause_button.setDisabled(True)
            self.create_thread()
            self.thread.start()

        self.pause_button.setDisabled(False)
        self.worker.play()

    def pause_all(self):
        self.play_button.setDisabled(False)
        self.pause_button.setDisabled(True)
        self.worker.pause()

    def load_settings(self) -> dict:
        with open("cam_settings.json", "r") as file:
            cam_settings = json.load(file)
            cam_settings = {int(key): val for key, val in cam_settings.items()}
        return cam_settings

    def cleanup(self):
        if self.thread is None:
            return

        self.worker.cleanup()
        self.thread.quit()
        self.thread.wait()
        self.worker.deleteLater()
        self.thread.deleteLater()
