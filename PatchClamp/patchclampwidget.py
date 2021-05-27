# -*- coding: utf-8 -*-
"""
Created on Mon Apr 12 10:37:51 2021

@author: tvdrb
"""

import os
import sys

from PyQt5 import QtWidgets
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QWidget, QGridLayout, QPushButton, QGroupBox
import pyqtgraph.exporters
import pyqtgraph as pg

# Ensure that the Widget can be run either independently or as part of Tupolev.
if __name__ == "__main__":
    os.chdir(os.getcwd() + '\\..')
from PatchClamp.autopatch import AutomaticPatcher


class PatchClampUI(QWidget):
    
    def __init__(self, camera_handle = None, motor_handle = None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        #======================================================================
        #---------------------------- Start of GUI ----------------------------
        #======================================================================
        #----------------------- General widget settings ----------------------
        self.setWindowTitle("Automatic Patchclamp")
        
        # -------------------------- Save directory ---------------------------
        
        #---------------------------- Snapshot view ---------------------------
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
        
        # Button for staring live camera view
        request_camera_image_button = QPushButton("Live")
        # request_camera_image_button.clicked.connect(self.live)
        snapshotLayout.addWidget(request_camera_image_button, 1, 0, 1, 1)
        
        # Button for making a snapshot
        request_camera_image_button = QPushButton("Snap")
        request_camera_image_button.clicked.connect(self.snap)
        snapshotLayout.addWidget(request_camera_image_button, 1, 1, 1, 1)
        
        # Button for automatic focussing a pipette
        request_autofocus_button = QPushButton("Autofocus pipette")
        request_autofocus_button.clicked.connect(self.autofocus)
        snapshotLayout.addWidget(request_autofocus_button, 1, 2, 1, 1)
        
        # Button for detecting pipette tip
        request_pipette_coordinates_button = QPushButton("Detect pipette tip")
        # request_pipette_coordinates_button.clicked.connect(self.localize_pipette)
        snapshotLayout.addWidget(request_pipette_coordinates_button, 1, 3, 1, 1)
        
        snapshotContainer.setLayout(snapshotLayout)
        
        # --------------------------- Add to master ---------------------------
        master = QGridLayout()
        master.addWidget(snapshotContainer, 0, 0, 1, 1)
        
        self.setLayout(master)
        
        # =====================================================================
        # ---------------------------- End of GUI -----------------------------
        # =====================================================================
        
        # Initiate backend
        self.autopatch_instance = AutomaticPatcher()
        
        # Connect signals to slots
        self.autopatch_instance.livesignal.connect(lambda I: self.update_canvas(I, canvasnumber=0))
        self.autopatch_instance.snapsignal.connect(lambda I: self.update_canvas(I, canvasnumber=1))
        
    
    def update_canvas(self, image, canvasnumber):
        if canvasnumber == 0:
            # Update the live canvas
            self.canvaslive.setImage(image)
        else:
            # Update the snap canvas
            self.canvassnap.setImage(image)
        
    def autofocus(self):
        # Create a thread
        self.thread = QThread()
        # Move backend to a thread
        self.autopatch_instance.moveToThread(self.thread)
        # Run pipette autofocus when thread is started
        self.thread.started.connect(self.autopatch_instance.autofocus_pipette)
        # Start thread
        self.thread.start()
        
    def snap(self):
        # snap image and update the view
        self.autopatch_instance.snap_image()
    
    def closeEvent(self, event):
        """ Interupts widget processes
        
        On closing the application we have to make sure that the console
        gets freed.
        """
        QtWidgets.QApplication.quit()
        event.accept()


if __name__ == "__main__":
    def run_app():
        app = QtWidgets.QApplication(sys.argv)
        pg.setConfigOptions(imageAxisOrder='row-major') #Transposes image in pg.ImageView()
        mainwin = PatchClampUI()
        mainwin.show()
        app.exec_()
    run_app()
    