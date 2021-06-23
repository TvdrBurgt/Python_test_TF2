# -*- coding: utf-8 -*-
"""
Created on Mon Apr 12 10:37:51 2021

@author: tvdrb
"""

import os
import sys
import numpy as np
import logging

from PyQt5.QtCore import Qt
from PyQt5 import QtWidgets
from PyQt5.QtGui import QPen
from PyQt5.QtWidgets import QWidget, QGridLayout, QPushButton, QGroupBox
import pyqtgraph.exporters
import pyqtgraph as pg

# Change path so the Widget can be run either independently or as part of Tupolev.
if __name__ == "__main__":
    os.chdir(os.getcwd() + '\\..')
from PatchClamp.patchclamp_backend import AutoPatchThread
from PatchClamp.camera import CameraThread
from PatchClamp.micromanipulator import ScientificaPatchStar


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
        self.connect_camera_button = QPushButton(text="Camera", clicked=self.toggle_connect_camera)
        self.connect_camera_button.setCheckable(True)
        
        # Button for connecting micromanipulator
        self.connect_micromanipulator_button = QPushButton("Micromanipulator", clicked=self.toggle_connect_micromanipulator)
        self.connect_micromanipulator_button.setCheckable(True)
        
        # Button for connecting objective motor
        self.connect_objective_button = QPushButton("Objective", clicked=self.toggle_connect_objective)
        self.connect_objective_button.setCheckable(True)
        
        # Emergency stop to stop everything and disconnect all
        emergency_button = QPushButton(text="STOP", clicked=self.emergency_stop)
        
        hardwareLayout.addWidget(self.connect_camera_button, 0, 0, 1, 1)
        hardwareLayout.addWidget(self.connect_micromanipulator_button, 1, 0, 1, 1)
        hardwareLayout.addWidget(self.connect_objective_button, 2, 0, 1, 1)
        hardwareLayout.addWidget(emergency_button, 3, 0, 1, 1)
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

        # Display to project snapshots
        self.snapshotWidget = pg.ImageView()
        self.canvassnap = self.snapshotWidget.getImageItem()
        self.canvassnap.setAutoDownsample(True)
        self.snapshotWidget.ui.roiBtn.hide()
        self.snapshotWidget.ui.menuBtn.hide()
        self.snapshotWidget.ui.histogram.hide()

        # Button for freezing camera view
        self.request_pause_button = QPushButton(text="Pause live", clicked=self.toggle_live)
        self.request_pause_button.setCheckable(True)

        # Button for making a snapshot
        request_camera_image_button = QPushButton(text="Snap image", clicked=self.request_snap)

        # Button for autofocus
        request_autofocus_button = QPushButton(text="Autofocus", clicked=self.request_autofocus)

        # Button for pipette detection
        self.request_detect_button = QPushButton(text="Detect pipette", clicked=self.request_detect)
        self.request_detect_button.setCheckable(True)
        
        # Button for calibration
        request_calibrate_button = QPushButton(text="Calibrate", clicked=self.request_calibrate)
        
        snapshotLayout.addWidget(self.liveWidget, 0, 0, 1, 5)
        snapshotLayout.addWidget(self.snapshotWidget, 0, 5, 1, 5)
        snapshotLayout.addWidget(self.request_pause_button, 1, 0, 1, 2)
        snapshotLayout.addWidget(request_camera_image_button, 1, 2, 1, 2)
        snapshotLayout.addWidget(request_autofocus_button, 1, 4, 1, 2)
        snapshotLayout.addWidget(self.request_detect_button, 1, 6, 1, 2)
        snapshotLayout.addWidget(request_calibrate_button, 1, 8, 1, 2)
        snapshotContainer.setLayout(snapshotLayout)

        # -------------------------- Adding to master -------------------------
        master = QGridLayout()
        master.addWidget(hardwareContainer, 0, 0, 1, 1)
        master.addWidget(snapshotContainer, 0, 1, 1, 1)

        self.setLayout(master)

        # =====================================================================
        # ---------------------------- End of GUI -----------------------------
        # =====================================================================
        
        # Fire up backend and establish feedback communication
        self.autopatch = AutoPatchThread()
        self.autopatch.sketches.connect(self.update_canvassnap)
        # self.autopatch.drawings.connect(self.draw)
        self.autopatch.crosshair.connect(self.draw_crosshair)
        self.autopatch.drawsignal.connect(self.draw_lines)
        
    def toggle_connect_camera(self):
        if self.connect_camera_button.isChecked():
            # Create instance and pass on to backend
            self.camerathread = CameraThread()
            self.autopatch.camera_instance = self.camerathread
            
            # Connect canvasses to camera signals
            self.camerathread.livesignal.connect(self.update_canvaslive)
            self.camerathread.snapsignal.connect(self.update_canvassnap)
            
            # Start camera thread
            self.camerathread.start()
        else:
            # Delete camera thread
            self.camerathread.__del__()
        
    def toggle_connect_micromanipulator(self):
        # make this a toggle function, function should initiate and connect or
        # disconnect and delete.
        if self.connect_micromanipulator_button.isChecked():
            self.micromanipulatorinstance = ScientificaPatchStar(address='COM16', baud=38400)
            self.autopatch.micromanipulator_handle = self.micromanipulatorinstance
        else:
            # Make my own close function in the micromanipulator class
            self.micromanipulatorinstance.close()
    
    def toggle_connect_objective(self):
        # make this a toggle function, function should initiate and connect or
        # disconnect and delete.
        if self.connect_objective_button.isChecked():
            pass
        else:
            pass
    
    def toggle_live(self, state):
        logging.info('Toggle live')
        # Request to pause or continue emitting frames from the camera thread
        if self.request_pause_button.isChecked():
            self.camerathread.livesignal.disconnect()
        else:
            self.camerathread.livesignal.connect(self.update_canvaslive)
        
    def request_snap(self):
        logging.info('Snap button pushed')
        self.camerathread.snap()
        
    def request_autofocus(self):
        logging.info('Request autofocus button pushed')
        self.autopatch.request("autofocus")
        
    def request_detect(self):
        logging.info('Request detection button pushed')
        if self.request_detect_button.isChecked():
            self.autopatch.request("detect")
        else:
            self.snapshotWidget.getView().removeItem(self.crosshairs[-1])
            self.snapshotWidget.getView().removeItem(self.crosshairs[-2])
        
    def request_calibrate(self):
        logging.info('Request calibrate button pushed')
        self.autopatch.request("calibrate")
        
    def update_canvaslive(self, image):
        self.canvaslive.setImage(image)
        
    def update_canvassnap(self, image):
        self.canvassnap.setImage(image)
        
    def draw_crosshair(self, position):
        logging.info('Display crosshair')
        vertical = pg.ROI(pos=(position[0],position[1]-15), size=[1,30], pen=QPen(Qt.yellow, 0))
        horizontal = pg.ROI(pos=(position[0]-15,position[1]), size=[30,1], pen=QPen(Qt.yellow, 0))
        
        if hasattr(self, 'crosshairs'):
            self.crosshairs.append(vertical)
            self.crosshairs.append(horizontal)
        else:
            self.crosshairs = [vertical, horizontal]
        self.snapshotWidget.getView().addItem(self.crosshairs[-1])
        self.snapshotWidget.getView().addItem(self.crosshairs[-2])
    
    def draw_lines(self, data):
        logging.info('Display calibration line')
        position = data[0]
        slope = data[1]
        dx = slope[0]
        dy = slope[1]
        axis = pg.ROI(pos=(position[0],position[1]), size=[np.sign(dx)*200*np.linalg.norm(slope),np.sign(dy)], pen=QPen(Qt.green, 0), movable=False)
        axis.rotate(np.rad2deg(np.arctan(dy/dx)))
        
        if hasattr(self, 'lines'):
            self.lines.append(axis)
        else:
            self.lines = [axis]
        self.snapshotWidget.getView().addItem(self.lines[-1])
        
    # def draw(self, description, position, geometry):
    #     logging.info('Draw something')
    #     [x, y, z] = position
    #     [dx, dy, dz] = geometry
        
    #     if description == "cross":
    #         vertical = pg.ROI(pos=(x,y-15), size=[1,30], pen=QPen(Qt.yellow, 0))
    #         horizontal = pg.ROI(pos=(x-15,y), size=[30,1], pen=QPen(Qt.yellow, 0))
    #     elif description == "line":
    #         # line = pg.ROI(pos=(x,y), size=[dx,dy], pen=QPen(Qt.green, 0))
    #         line = pg.ROI(pos=(x,y), size=[np.sign(dx)*200*np.linalg.norm(geometry),np.sign(dy)], pen=QPen(Qt.green, 0))
    #         line.rotate(np.rad2deg(np.arctan(dy/dx)))
    #     else:
    #         pass
        
        
    def emergency_stop(self):
        logging.info('Emergency stop pressed')
        # Disconnect all signals and slots, here or in class?
        if hasattr(self, 'camerathread'):
            self.camerathread.__del__()
            logging.info('Camera thread stopped successfully')
        if hasattr(self, 'autopatch'):
            self.autopatch.__del__()
            logging.info('Autopatch stopped successfully')
        
    def closeEvent(self, event):
        """ Close event
        This method is called when the GUI is shut down. First we need to stop
        the threads that are steill running, then we accept the close event
        and quit the widget.
        """
        self.emergency_stop()
        event.accept()
        QtWidgets.QApplication.quit()
        self.close()
        logging.info('Widget closed successfully')


if __name__ == "__main__":
    def start_logger():
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s] %(message)s",
            handlers=[
                # logging.FileHandler("autopatch.log"),   # uncomment to write to .log
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
