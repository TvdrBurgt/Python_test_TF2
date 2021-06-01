# -*- coding: utf-8 -*-
"""
Created on Tue Jun  1 10:54:02 2021

@author: tvdrb
"""


import os
import sys

from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QWidget, QGridLayout, QPushButton, QGroupBox
import pyqtgraph as pg

# Change path so the Widget can be run either independently or as part of Tupolev.
if __name__ == "__main__":
    os.chdir(os.getcwd() + '\\..')
from PatchClamp.patchclamp_backend import CameraThread, AutoPatchThread


class PatchClampUI(QWidget):
    
    def __init__(self, camera_handle = None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        # =====================================================================
        # --------------------------- Start of GUI ----------------------------
        # =====================================================================
        # ---------------------- General widget settings ----------------------
        self.setWindowTitle("Automatic Patchclamp")
        
        # -------------------------- Camera section ---------------------------
        displayContainer = QGroupBox()
        displayContainer.setMinimumSize(1200, 600)
        displayLayout = QGridLayout()
        
        # Display to project live camera output
        self.liveWidget = pg.ImageView()
        self.canvaslive = self.liveWidget.getImageItem()
        self.canvaslive.setAutoDownsample(True)
        self.liveWidget.ui.roiBtn.hide()
        self.liveWidget.ui.menuBtn.hide()
        self.liveWidget.ui.histogram.hide()
        displayLayout.addWidget(self.liveWidget, 0, 0, 1, 2)
        
        # Display to project snapshots
        self.snapshotWidget = pg.ImageView()
        self.canvassnap = self.snapshotWidget.getImageItem()
        self.canvassnap.setAutoDownsample(True)
        self.snapshotWidget.ui.roiBtn.hide()
        self.snapshotWidget.ui.menuBtn.hide()
        self.snapshotWidget.ui.histogram.hide()
        displayLayout.addWidget(self.snapshotWidget, 0, 2, 1, 2)
        
        # Button for starting camera acquisition. This button is a property of
        # the container because we need its checkable state in a function.
        self.toggle_live_button = QPushButton("Pause live")
        self.toggle_live_button.setCheckable(True)
        self.toggle_live_button.clicked.connect(self.toggle_live)
        displayLayout.addWidget(self.toggle_live_button, 1, 0, 1, 1)
        
        # Button for making a snapshot
        request_camera_image_button = QPushButton("Snap image")
        request_camera_image_button.clicked.connect(self.request_snap)
        displayLayout.addWidget(request_camera_image_button, 1, 1, 1, 1)
        
        # Button for automatic focussing a pipette
        request_autofocus_button = QPushButton("Autofocus pipette")
        # request_autofocus_button.clicked.connect(self.request_autofocus)
        displayLayout.addWidget(request_autofocus_button, 1, 2, 1, 1)
        
        # Button for detecting pipette tip
        request_pipette_coords_button = QPushButton("Detect pipette tip")
        # request_pipette_coords_button.clicked.connect(self.request_pipette_coords)
        displayLayout.addWidget(request_pipette_coords_button, 1, 3, 1, 1)
        
        displayContainer.setLayout(displayLayout)
        
        # -------------------------- Adding to master -------------------------
        master = QGridLayout()
        master.addWidget(displayContainer, 0, 0, 1, 1)

        self.setLayout(master)
        # ---------------------------- End of GUI -----------------------------
        
        # =====================================================================
        # ------------------------- Activate backend --------------------------
        # =====================================================================
        
        # Initiate backend threads
        self.camerathread = CameraThread()
        self.autopatchthread = AutoPatchThread(self.camerathread)
        
        # Connect our GUI display to the signal slots
        self.camerathread.snapsignal.connect(self.update_canvassnap)
        self.camerathread.livesignal.connect(self.update_canvaslive)

        # Start threads
        self.camerathread.start()
        
    def update_canvaslive(self, image):
        # Update the live canvas with every new camera frame
        self.canvaslive.setImage(image)
        
    def update_canvassnap(self, image):
        # Update the snap canvas with the newest camera frame
        self.canvassnap.setImage(image)
    
    def toggle_live(self):
        # Request to pause or continue emitting frames from the camera thread
        if self.toggle_live_button.isChecked():
            self.camerathread.stop_canvasupdates()
        else:
            self.camerathread.start_canvasupdates()
        
    def request_snap(self):
        # Request a snapshot from the camera thread
        self.camerathread.snap()
        
    def request_autofocus(self):
        print("Autofocus pipette")
        # Starting the autopatch thread with the pipette autofocus functions
        self.autopatchthread.started.connect(self.autopatchthread.autofocus_pipette)
        
    def request_pipette_coords(self):
        print("Detect pipette tip")
        # Starting the autopatch thread with the pipette autofocus functions
        self.autopatchthread.started.connect(self.autopatchthread.detect_pipette_tip)

    def closeEvent(self, event):
        """ Close event
        This method is called when the GUI is shut down. First we need to stop
        the threads that are steill running, then we accept the close event
        and quit the application.
        """
        self.camerathread.__del__()
        self.autopatchthread.__del__()
        event.accept()
        QtWidgets.QApplication.quit()
        
        
if __name__ == "__main__":

    def run_app():
        app = QtWidgets.QApplication(sys.argv)
        pg.setConfigOptions(
            imageAxisOrder="row-major"
        )  # Transposes image in pg.ImageView()
        mainwin = PatchClampUI()
        mainwin.show()
        app.exec_()

    run_app()
