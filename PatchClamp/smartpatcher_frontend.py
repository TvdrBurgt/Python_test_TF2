# -*- coding: utf-8 -*-
"""
Created on Fri Aug  6 15:15:38 2021

@author: tvdrb
"""


import os
import sys
import numpy as np
import logging

from PyQt5.QtCore import Qt
from PyQt5 import QtWidgets
from PyQt5.QtGui import QPen
from PyQt5.QtWidgets import QWidget, QGridLayout, QPushButton, QDoubleSpinBox, QGroupBox, QLabel
import pyqtgraph.exporters
import pyqtgraph as pg




class PatchClampUI(QWidget):
    def __init__(self):
        super().__init__()

        # =====================================================================
        # ---------------------------- Start of GUI ---------------------------
        # =====================================================================
        # ---------------------- General widget settings ---------------------
        self.setWindowTitle("Automatic Patchclamp")
        
        
        # ------------------------- Hardware container ------------------------
        hardwareContainer = QGroupBox()
        hardwareLayout = QGridLayout()
        
        # Button to (dis)connect camera
        self.connect_camera_button = QPushButton(text="Camera", clicked=self.mockfunction)
        self.connect_camera_button.setCheckable(True)
        
        # Button to (dis)connect objective motor
        self.connect_objectivemotor_button = QPushButton(text="Objective motor", clicked=self.mockfunction)
        self.connect_objectivemotor_button.setCheckable(True)
        
        # Button to (dis)connect micromanipulator
        self.connect_micromanipulator_button = QPushButton(text="Micromanipulator", clicked=self.mockfunction)
        self.connect_micromanipulator_button.setCheckable(True)
        
        # Button to (dis)connect amplifier
        self.connect_amplifier_button = QPushButton(text="Amplifier", clicked=self.mockfunction)
        self.connect_amplifier_button.setCheckable(True)
        
        # Button to (dis)connect pressure controller
        self.connect_pressurecontroller_button = QPushButton(text="Pressure controller", clicked=self.mockfunction)
        self.connect_pressurecontroller_button.setCheckable(True)
        
        # Button to stop all hardware in motion
        self.STOP_button = QPushButton(text="Emergency STOP", clicked=self.mockfunction)
        self.STOP_button.setCheckable(True)
        
        hardwareLayout.addWidget(self.connect_camera_button, 0, 0, 1, 1)
        hardwareLayout.addWidget(self.connect_objectivemotor_button, 1, 0, 1, 1)
        hardwareLayout.addWidget(self.connect_micromanipulator_button, 2, 0, 1, 1)
        hardwareLayout.addWidget(self.connect_amplifier_button, 3, 0, 1, 1)
        hardwareLayout.addWidget(self.connect_pressurecontroller_button, 4, 0, 1, 1)
        hardwareLayout.addWidget(self.STOP_button, 5, 0, 1, 1)
        hardwareContainer.setLayout(hardwareLayout)
        
        # ------------------------ Camera view display ------------------------
        viewContainer = QGroupBox()
        viewContainer.setMinimumSize(600, 600)
        viewLayout = QGridLayout()
        
        # Display to project live camera view
        viewWidget = pg.ImageView()
        self.view = viewWidget.getImageItem()
        # self.view.setAutoDownsample(True)
        viewWidget.ui.roiBtn.hide()
        viewWidget.ui.menuBtn.hide()
        viewWidget.ui.histogram.hide()
        
        # Button for pausing camera view
        self.request_pause_button = QPushButton(text="Pause live", clicked=self.mockfunction)
        self.request_pause_button.setCheckable(True)
        
        viewLayout.addWidget(viewWidget, 0, 0, 1, 1)
        viewLayout.addWidget(self.request_pause_button, 1, 0, 1, 1)
        viewContainer.setLayout(viewLayout)
        
        # ------------------------- Live data display ------------------------
        liveContainer = QGroupBox()
        liveContainer.setMinimumSize(400,600)
        liveLayout = QGridLayout()
        
        
        
        # --------------------- Algorithm control buttons ---------------------
        algorithmContainer = QGroupBox()
        algorithmLayout = QGridLayout()
        
        # Button for hard calibration in XY
        request_hardcalibrationxy_button = QPushButton(text="Calibrate XY", clicked=self.mockfunction)
        
        # Button for hard calibration in XYZ
        request_hardcalibrationxyz_button = QPushButton(text="Calibrate XYZ", clicked=self.mockfunction)
        
        # Button for target selection
        request_selecttarget_button = QPushButton(text="Select target", clicked=self.mockfunction)
        
        # Button for pipette tip detection in XY
        request_detecttip_button = QPushButton(text="Detect tip", clicked=self.mockfunction)
        
        # Button for pipette tip autofocus
        request_autofocustip = QPushButton(text="Autofocus tip", clicked=self.mockfunction)
        
        # Button for gigaseal formation
        request_gigaseal_button = QPushButton(text="Gigaseal", clicked=self.mockfunction)
        
        # Button for break-in
        request_breakin_button = QPushButton(text="Break-in", clicked=self.mockfunction)
        
        # Button for ZAP
        request_zap_button = QPushButton(text="ZAP", clicked=self.mockfunction)
        
        # Button to set pressure
        self.set_pressure_button = QDoubleSpinBox(self)
        self.set_pressure_button.setMinimum(-200)
        self.set_pressure_button.setMaximum(200)
        self.set_pressure_button.setDecimals(0)
        self.set_pressure_button.setValue(0)
        self.set_pressure_button.setSingleStep(1)
            
        # Button to release pressure instantaneous
        request_releasepressure_button = QPushButton(text="Release pressure", clicked=self.mockfunction)
        
        # Button to send pressure to pressure controller
        request_applypressure_button = QPushButton(text="Apply pressure", clicked=self.mockfunction)
        
        algorithmLayout.addWidget(request_hardcalibrationxy_button, 0, 0, 1, 1)
        algorithmLayout.addWidget(request_hardcalibrationxyz_button, 1, 0, 1, 1)
        algorithmLayout.addWidget(request_selecttarget_button, 0, 1, 2, 1)
        algorithmLayout.addWidget(request_detecttip_button, 0, 2, 2, 1)
        algorithmLayout.addWidget(request_autofocustip, 0, 3, 2, 1)
        algorithmLayout.addWidget(request_gigaseal_button, 0, 4, 2, 1)
        algorithmLayout.addWidget(request_breakin_button, 0, 5, 2, 1)
        algorithmLayout.addWidget(request_zap_button, 0, 6, 2, 1)
        algorithmLayout.addWidget(QLabel("Pressure (in mBar):"), 0, 7, 1, 1)
        algorithmLayout.addWidget(self.set_pressure_button, 0, 8, 1, 1)
        algorithmLayout.addWidget(request_releasepressure_button, 1, 7, 1, 1)
        algorithmLayout.addWidget(request_applypressure_button, 1, 8, 1, 1)
        algorithmContainer.setLayout(algorithmLayout)
        
        # -------------------------- Adding to master -------------------------
        master = QGridLayout()
        master.addWidget(hardwareContainer, 0, 0, 1, 1)
        master.addWidget(viewContainer, 0, 1, 1, 1)
        master.addWidget(liveContainer, 0, 2, 1, 1)
        master.addWidget(algorithmContainer, 1, 0, 1, 3)
        

        self.setLayout(master)
        
        
        
        # =====================================================================
        # ---------------------------- End of GUI -----------------------------
        # =====================================================================
        
        
        
        
    def mockfunction(self):
        print("Button pushed")
        
        
    def closeEvent(self, event):
        """ Close event
        This method is called when the GUI is shut down. First we need to stop
        the threads that are still running, then we accept the close event
        and quit the widget.
        """
        
        QtWidgets.QApplication.quit() # minimally required to return kernel on my laptop
        # event.accept()
        
        
        
        
        

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
