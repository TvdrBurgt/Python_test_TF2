# -*- coding: utf-8 -*-
"""
Created on Mon Apr 12 10:37:51 2021

@author: tvdrb
"""

from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QGridLayout, QPushButton, QGroupBox
import pyqtgraph.exporters
import pyqtgraph as pg

import sys
import os

# Ensure that the Widget can be run either independently or as part of Tupolev.
if __name__ == "__main__":
    os.chdir(os.getcwd() + '\\..')
from HamamatsuCam.HamamatsuUI import CameraUI


class AutomaticPatchclampUI(QWidget):
    
    # initialize signals
    sig_request_SnapImg = pyqtSignal()
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        #----------------------------------------------------------------------
        #----------------------------------GUI---------------------------------
        #----------------------------------------------------------------------
        self.setWindowTitle("Automatic Patchclamp")
        
        #-----------------------------Snapshot view----------------------------
        snapshotContainer = QGroupBox()
        snapshotContainer.setMinimumSize(600, 600)
        snapshotLayout = QGridLayout()
        
        #add a pyqt graphic and hide roi, menu, histogram
        self.snapshotWidget = pg.ImageView()
        self.snapshotWidget.ui.roiBtn.hide()
        self.snapshotWidget.ui.menuBtn.hide()
        self.snapshotWidget.ui.histogram.hide()
        snapshotLayout.addWidget(self.snapshotWidget, 0, 0, 1, 1)
        
        #add a snapshot button
        request_camera_image_button = QPushButton("Snap image")
        # request_camera_image_button.clicked.connect(lambda: self.request_SnapImg      # is lambda necessary?
        request_camera_image_button.clicked.connect(self.request_SnapImg)
        snapshotLayout.addWidget(request_camera_image_button, 1, 0, 1, 1)
        
        #update figure when new snapshot is made
        CameraUI().output_signal_SnapImg.connect(self.updateimage)
        
        snapshotContainer.setLayout(snapshotLayout)
        
        #---------------------------Adding to master---------------------------
        master = QGridLayout()
        master.addWidget(snapshotContainer, 0, 0, 1, 1)
        
        self.setLayout(master)
        
        
        #----------------------------------------------------------------------
        #-----------------------------End of GUI-------------------------------
        #----------------------------------------------------------------------
        
        
    def updateimage(self,image):
        # pipetterecognition(image)
        print('Refresh the image figure and do image analysis etc...')
        self.snapshotWidget.getImageItem().setImage(image)
        
    def request_SnapImg(self):
        # self.sig_request_SnapImg.emit()
        print('Asking Camera for a snap')
        CameraUI().SnapImg()
    
    def closeEvent(self, event):
        """On closing the application we have to make sure that the console
        gets freed."""
        QtWidgets.QApplication.quit()
        event.accept()


if __name__ == "__main__":
    def run_app():
        app = QtWidgets.QApplication(sys.argv)
        mainwin = AutomaticPatchclampUI()
        mainwin.show()
        app.exec_()
    run_app()
    