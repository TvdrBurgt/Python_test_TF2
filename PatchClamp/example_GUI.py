# -*- coding: utf-8 -*-
"""
Created on Mon Apr 12 10:37:51 2021

@author: tvdrb
"""

import sys
import logging

from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QWidget, QGridLayout, QPushButton, QGroupBox
import pyqtgraph.exporters
import pyqtgraph as pg

from example_backend import CameraThread, AutoPatchThread


class PatchClampUI(QWidget):
    def __init__(self):
        super().__init__()

        # =====================================================================
        # ---------------------------- Start of GUI ---------------------------
        # =====================================================================
        # ----------------------- General widget settings ---------------------
        self.setWindowTitle("Automatic Patchclamp")

        # ------------------ (Dis)onnect hardware container ------------------
        hardwareContainer = QGroupBox()
        hardwareLayout = QGridLayout()
        
        # Button for connecting camera
        self.connect_camera_button = QPushButton(text="Camera", clicked=self.toggle_connect_camera, checkable=True)
        
        # Button for connecting micromanipulator
        self.connect_micromanipulator_button = QPushButton("Micromanipulator", clicked=self.toggle_connect_micromanipulator, checkable=True)
        
        # Button for connecting objective motor
        self.connect_objective_button = QPushButton("Objective", clicked=self.toggle_connect_objective, checkable=True)
        
        hardwareLayout.addWidget(self.connect_camera_button, 0, 0, 1, 1)
        hardwareLayout.addWidget(self.connect_micromanipulator_button, 1, 0, 1, 1)
        hardwareLayout.addWidget(self.connect_objective_button, 2, 0, 1, 1)
        hardwareContainer.setLayout(hardwareLayout)
        
        # ------------------------ Snapshot container ------------------------
        snapshotContainer = QGroupBox()
        snapshotContainer.setMinimumSize(1200, 600)
        snapshotLayout = QGridLayout()

        # Display to project live camera view
        self.liveWidget = pg.ImageView()
        self.canvaslive = self.liveWidget.getImageItem()
        self.canvaslive.setAutoDownsample(True)

        self.liveWidget.ui.roiBtn.hide()
        self.liveWidget.ui.menuBtn.hide()
        self.liveWidget.ui.histogram.hide()
        snapshotLayout.addWidget(self.liveWidget, 0, 0, 1, 2)

        # Display to project snapshots
        self.snapshotWidget = pg.ImageView()
        self.canvassnap = self.snapshotWidget.getImageItem()
        self.canvassnap.setAutoDownsample(True)

        self.snapshotWidget.ui.roiBtn.hide()
        self.snapshotWidget.ui.menuBtn.hide()
        self.snapshotWidget.ui.histogram.hide()
        snapshotLayout.addWidget(self.snapshotWidget, 0, 2, 1, 2)

        # Button for starting camera acquisition. This button is a property of
        # the container because we need its checkable state in a function.
        self.request_pause_button = QPushButton(text="Pause live", clicked=self.toggle_live, checkable=True)
        snapshotLayout.addWidget(self.request_pause_button, 1, 0, 1, 1)

        # Button for making a snapshot
        request_camera_image_button = QPushButton(text="Snap image", clicked=self.request_snap)
        snapshotLayout.addWidget(request_camera_image_button, 1, 1, 1, 1)

        # Button for autofocus
        request_autofocus_button = QPushButton(text="Autofocus", clicked=self.request_autofocus)
        snapshotLayout.addWidget(request_autofocus_button, 1, 2, 1, 1)

        # Button for pipette detection
        request_detection_button = QPushButton(text="Detect pipette", clicked=self.request_detect)
        snapshotLayout.addWidget(request_detection_button, 1, 3, 1, 1)

        snapshotContainer.setLayout(snapshotLayout)

        # -------------------------- Adding to master -------------------------
        master = QGridLayout()
        master.addWidget(hardwareContainer, 0, 0, 1, 1)
        master.addWidget(snapshotContainer, 0, 1, 1, 1)

        self.setLayout(master)

        # =====================================================================
        # ---------------------------- End of GUI -----------------------------
        # =====================================================================

    def toggle_connect_camera(self):
        # make this a toggle function, function should initiate and connect or
        # disconnect and delete.
        if self.connect_camera_button.isChecked():
            self.camerathread = CameraThread()
            self.camerathread.livesignal.connect(self.update_canvaslive)
            self.camerathread.snapsignal.connect(self.update_canvassnap)
            self.camerathread.start()
        else:
            self.camerathread.__del__()
        
    def toggle_connect_micromanipulator(self):
        # make this a toggle function, function should initiate and connect or
        # disconnect and delete.
        if self.connect_micromanipulator_button.isChecked():
            pass
        else:
            pass
    
    def toggle_connect_objective(self):
        # make this a toggle function, function should initiate and connect or
        # disconnect and delete.
        if self.connect_objective_button.isChecked():
            pass
        else:
            pass
    
    def toggle_live(self, state):
        # Request to pause or continue emitting frames from the camera thread
        if self.request_pause_button.isChecked():
            self.camerathread.livesignal.disconnect()
        else:
            self.camerathread.livesignal.connect(self.update_canvaslive)

    def request_snap(self):
        # Request a snapshot from the camera thread
        self.camerathread.snap()

    def request_autofocus(self):
        self.autopatchthread = AutoPatchThread("request autofocus", self.camerathread)
        self.autopatchthread.start()

    def request_detect(self):
        self.autopatchthread = AutoPatchThread("request detect", self.camerathread)
        self.autopatchthread.start()

    def update_canvaslive(self, image):
        # Update the live canvas with every new camera frame
        self.canvaslive.setImage(image)

    def update_canvassnap(self, image):
        # Update the snap canvas with the newest camera frame
        self.canvassnap.setImage(image)
        
    def closeEvent(self, event):
        if hasattr(self, 'camerathread'):
            self.camerathread.__del__()
            logging.info('Camera thread stopped')
        else:
            pass
        if hasattr(self, 'autopatchthread'):
            self.autopatchthread.__del__()
            logging.info('Autopatch stopped')
        else:
            pass
        QtWidgets.QApplication.quit()
        event.accept()
        logging.info('Widget closed successfully')


if __name__ == "__main__":
    def start_logger():
        logging.basicConfig(
            level=logging.ERROR,
            format="%(asctime)s [%(levelname)s] %(message)s",
            handlers=[
                # logging.FileHandler("autopatch.log"),
                logging.StreamHandler()
            ]
        )
    
    def run_app():
        app = QtWidgets.QApplication(sys.argv)
        pg.setConfigOptions(
            imageAxisOrder="row-major"
        )  # Transposes image in pg.ImageView()
        mainwin = PatchClampUI()
        mainwin.show()
        app.exec_()
        
    start_logger()
    run_app()
