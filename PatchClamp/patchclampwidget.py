# -*- coding: utf-8 -*-
"""
Created on Mon Apr 12 10:37:51 2021

@author: tvdrb
"""

import os
import sys

import threading

from PyQt5 import QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QGridLayout, QPushButton, QGroupBox
from PyQt5.QtGui import QPen
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
        self.view = self.snapshotWidget.getImageItem()
        self.view.setAutoDownsample(True)
        
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
        request_pipette_coordinates_button.clicked.connect(self.localize_pipette)
        snapshotLayout.addWidget(request_pipette_coordinates_button, 1, 2, 1, 1)
        
        snapshotContainer.setLayout(snapshotLayout)
        
        #-------------------------- Adding to master --------------------------
        master = QGridLayout()
        master.addWidget(snapshotContainer, 0, 0, 1, 1)
        
        self.setLayout(master)
        
        
        #======================================================================
        #---------------------------- End of GUI ------------------------------
        #======================================================================
        
        # Fire up backend
        self.autopatch_instance = AutomaticPatcher(imageview_handle=self.view)
        
        
    def autofocus(self):
        self.autopatch_instance.autofocus_pipette()
        
        # Update display
        self.view.setImage(self.snap)
        
    def snap_shot(self):
        # snap image and update the view
        self.autopatch_instance.snap_image()
        
    
    def localize_pipette(self):
        x, y = self.autopatch_instance.detect_pipette_tip()
        
        # Draw a crosshair at pipette tip coordinates
        pen = QPen(Qt.red, 0.1)
        pen.setWidthF(5)
        r = MyCrosshairOverlay(pos=(x, y), size=25, pen=pen, movable=False)
        self.snapshotWidget.getView().addItem(r)
        
        pass
    
    def run_in_thread(self, fn, *args, **kwargs):
        thread = threading.Thread(target=fn, args=args, kwargs=kwargs)
        thread.start()
        
        return thread
    
    def closeEvent(self, event):
        """ Interupts widget processes
        
        On closing the application we have to make sure that the console
        gets freed.
        """
        QtWidgets.QApplication.quit()
        event.accept()


class MyCrosshairOverlay(pg.CrosshairROI):
    def __init__(self, pos=None, size=None, **kargs):
        self._shape = None
        pg.ROI.__init__(self, pos, size, **kargs)
        self.sigRegionChanged.connect(self.invalidate)
        self.aspectLocked = True


if __name__ == "__main__":
    def run_app():
        app = QtWidgets.QApplication(sys.argv)
        pg.setConfigOptions(imageAxisOrder='row-major') #Transposes image in pg.ImageView()
        mainwin = PatchClampUI()
        mainwin.show()
        app.exec_()
    run_app()
    