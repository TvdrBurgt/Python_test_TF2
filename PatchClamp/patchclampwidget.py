# -*- coding: utf-8 -*-
"""
Created on Mon Apr 12 10:37:51 2021

@author: tvdrb
"""

import os
import sys
import numpy as np

from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QWidget, QGridLayout, QPushButton, QGroupBox
import pyqtgraph.exporters
import pyqtgraph as pg

from PatchClamp.autopatch import AutomaticPatcher


# Ensure that the Widget can be run either independently or as part of Tupolev.
if __name__ == "__main__":
    os.chdir(os.getcwd() + '\\..')


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
        self.view = self.snapshotWidget.getImageItem() #setLevels
        self.view.setAutoDownsample(True)
        
        self.snapshotWidget.ui.roiBtn.hide()
        self.snapshotWidget.ui.menuBtn.hide()
        self.snapshotWidget.ui.histogram.hide()
        snapshotLayout.addWidget(self.snapshotWidget, 0, 0, 1, 2)
        
        # Button for making a snapshot
        request_camera_image_button = QPushButton("Snap image")
        request_camera_image_button.clicked.connect(self.snapshot)
        snapshotLayout.addWidget(request_camera_image_button, 1, 0, 1, 1)
        
        # Button for detecting pipette tip
        request_camera_image_button = QPushButton("Detect pipette tip")
        request_camera_image_button.clicked.connect(self.localize_pipette)
        snapshotLayout.addWidget(request_camera_image_button, 1, 1, 1, 1)
        
        snapshotContainer.setLayout(snapshotLayout)
        
        #-------------------------- Adding to master --------------------------
        master = QGridLayout()
        master.addWidget(snapshotContainer, 0, 0, 1, 1)
        
        self.setLayout(master)
        
        
        #======================================================================
        #---------------------------- End of GUI ------------------------------
        #======================================================================
    
    def snapshot(self):
        autopatch_instance = AutomaticPatcher()
        image = autopatch_instance.snap_image()
        
        # Update display
        self.view.setImage(image)
        
    
    def localize_pipette(self):
        pass
    
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
        mainwin = PatchClampUI()
        mainwin.show()
        app.exec_()
    run_app()
    