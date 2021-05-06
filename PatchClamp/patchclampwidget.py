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
        
        #---------------------------- Snapshot view ---------------------------
        snapshotContainer = QGroupBox()
        snapshotContainer.setMinimumSize(600, 600)
        snapshotLayout = QGridLayout()
        
        # Display to project snapshots
        self.snapshotWidget = pg.ImageView()
        self.canvas = self.snapshotWidget.getImageItem()
        self.canvas.setAutoDownsample(True)
        
        self.snapshotWidget.ui.roiBtn.hide()
        self.snapshotWidget.ui.menuBtn.hide()
        self.snapshotWidget.ui.histogram.hide()
        snapshotLayout.addWidget(self.snapshotWidget, 0, 0, 1, 3)
        
        # Button for automatic focussing a pipette
        request_autofocus_button = QPushButton("Autofocus pipette")
        request_autofocus_button.clicked.connect(self.autofocus)
        snapshotLayout.addWidget(request_autofocus_button, 1, 0, 1, 1)
        
        # Button for making a snapshot
        request_camera_image_button = QPushButton("Snap image")
        request_camera_image_button.clicked.connect(self.snap_shot)
        snapshotLayout.addWidget(request_camera_image_button, 1, 1, 1, 1)
        
        # Button for detecting pipette tip
        request_pipette_coordinates_button = QPushButton("Detect pipette tip")
        # request_pipette_coordinates_button.clicked.connect(self.localize_pipette)
        snapshotLayout.addWidget(request_pipette_coordinates_button, 1, 2, 1, 1)
        
        snapshotContainer.setLayout(snapshotLayout)
        
        #-------------------------- Adding to master --------------------------
        master = QGridLayout()
        master.addWidget(snapshotContainer, 0, 0, 1, 1)
        
        self.setLayout(master)
        
        
        #======================================================================
        #---------------------------- End of GUI ------------------------------
        #======================================================================
        
        # Initiate backend
        self.autopatch_instance = AutomaticPatcher()
        
        # Move backend to a thread
        self.thread = QThread()
        self.autopatch_instance.moveToThread(self.thread)
        
        # Connect our GUI display to the newimage slot
        self.autopatch_instance.newimage.connect(self.update_graph)
        
        # 
        self.thread.started.connect(self.autopatch_instance.run)
        self.thread.start()
        
        
    def autofocus(self):
        self.autopatch_instance.autofocus_pipette
        
    def snap_shot(self):
        # snap image and update the view
        self.autopatch_instance.snap_image()
        
    def update_graph(self, image):
        # Display the newly snapped image
        self.canvas.setImage(image)
        
    
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
    