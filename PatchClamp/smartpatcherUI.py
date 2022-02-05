# -*- coding: utf-8 -*-
"""
Created on Sat Feb  5 16:34:10 2022

@author: tvdrb
"""

import sys
import numpy as np
import logging
import matplotlib.pyplot as plt

from skimage import io

from PyQt5 import QtWidgets
from PyQt5.QtGui import QPen, QColor, QFont, QPalette
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QGridLayout, QPushButton, QDoubleSpinBox, QGroupBox, QLabel, QStackedWidget, QComboBox, QTabWidget
import pyqtgraph.exporters
import pyqtgraph as pg

sys.path.append('../')
from NIDAQ.constants import MeasurementConstants
from PatchClamp.manualpatcher_frontend import PatchclampSealTestUI
from PatchClamp.smartpatcher_backend import SmartPatcher
from PatchClamp.camerathread import CameraThread
from PatchClamp.sealtestthread import SealTestThread
from PatchClamp.pressurethread import PressureThread
from PatchClamp.objective import PIMotor
from PatchClamp.micromanipulator import ScientificaPatchStar
from PatchClamp.stage import LudlStage











class PatchClampUI(QWidget):
    def __init__(self):
        super().__init__()
        """
        =======================================================================
        ----------------------------- Start of GUI ----------------------------
        =======================================================================
        """
        """
        # ---------------------- General widget settings ---------------------
        """
        self.setWindowTitle("Patchclamp")
        
        """
        -------------------------- Hardware container -------------------------
        """
        hardwareContainer = QGroupBox("Hardware devices")
        hardwareLayout = QGridLayout()
        
        # Button to (dis)connect camera
        self.connect_camerathread_button = QPushButton(text="Camera", clicked=self.mockfunction)
        self.connect_camerathread_button.setCheckable(True)
        
        # Button to (dis)connect objective motor
        self.connect_objectivemotor_button = QPushButton(text="Objective motor", clicked=self.mockfunction)
        self.connect_objectivemotor_button.setCheckable(True)
        
        # Button to (dis)connect micromanipulator
        self.connect_micromanipulator_button = QPushButton(text="Micromanipulator", clicked=self.mockfunction)
        self.connect_micromanipulator_button.setCheckable(True)
        
        # Button to (dis)connect XYstage
        self.connect_XYstage_button = QPushButton(text="XY stage", clicked=self.mockfunction)
        self.connect_XYstage_button.setCheckable(True)
        
        # Button to (dis)connect NIDAQ for sealtest
        self.connect_sealtestthread_button = QPushButton(text="NIDAQ/Sealtest", clicked=self.mockfunction)
        self.connect_sealtestthread_button.setCheckable(True)
        
        # Button to (dis)connect pressure controller
        self.connect_pressurecontroller_button = QPushButton(text="Pressure controller", clicked=self.mockfunction)
        self.connect_pressurecontroller_button.setCheckable(True)
        
        hardwareLayout.addWidget(self.connect_camerathread_button, 0, 0, 1, 1)
        hardwareLayout.addWidget(self.connect_objectivemotor_button, 1, 0, 1, 1)
        hardwareLayout.addWidget(self.connect_micromanipulator_button, 2, 0, 1, 1)
        hardwareLayout.addWidget(self.connect_XYstage_button, 3, 0, 1, 1)
        hardwareLayout.addWidget(self.connect_sealtestthread_button, 4, 0, 1, 1)
        hardwareLayout.addWidget(self.connect_pressurecontroller_button, 5, 0, 1, 1)
        hardwareContainer.setLayout(hardwareLayout)
        
        """
        --------------------------- Stage movements ---------------------------
        """
        stagemoveContainer = QGroupBox("Stage move")
        stagemoveLayout = QGridLayout()
        
        request_stage_up = QPushButton(text="up", clicked=self.mockfunction)
        request_stage_down = QPushButton(text="down", clicked=self.mockfunction)
        request_stage_right = QPushButton(text="right", clicked=self.mockfunction)
        request_stage_left = QPushButton(text="left", clicked=self.mockfunction)
        
        stagemoveLayout.addWidget(request_stage_up, 0, 1, 1, 1)
        stagemoveLayout.addWidget(request_stage_down, 2, 1, 1, 1)
        stagemoveLayout.addWidget(request_stage_right, 1, 2, 1, 1)
        stagemoveLayout.addWidget(request_stage_left, 1, 0, 1, 1)
        stagemoveContainer.setLayout(stagemoveLayout)
        
        """
        ------------------------- Calibration options -------------------------
        """
        calibrationContainer = QGroupBox("Calibration options")
        calibrationLayout = QGridLayout()
        
        # Calibration algorithms
        request_calibrate_xy = QPushButton(text="Calibrate XY", clicked=self.mockfunction)
        request_calibrate_pixelsize = QPushButton(text="Calibrate pixelsize", clicked=self.mockfunction)
        
        # Calibration statistics
        rotation_angles_label = QLabel("Rotation axes (α,β,γ):")
        self.rotation_angles_value_label = QLabel("-")
        self.rotation_angles_value_label.setFont(QFont("Times", weight=QFont.Bold))
        rotation_angles_units_label = QLabel("degree")
        pixelsize_label = QLabel("Pixelsize:")
        self.pixelsize_unit_label = QLabel("-")
        self.pixelsize_unit_label.setFont(QFont("Times", weight=QFont.Bold))
        pixelsize_units_label = QLabel("nm")
        
        calibrationLayout.addWidget(request_calibrate_xy, 0, 0, 1, 3)
        calibrationLayout.addWidget(request_calibrate_pixelsize, 1, 0, 1, 3)
        calibrationLayout.addWidget(rotation_angles_label, 2, 0, 1, 1)
        calibrationLayout.addWidget(self.rotation_angles_value_label, 2, 1, 1, 1)
        calibrationLayout.addWidget(rotation_angles_units_label, 2, 2, 1, 1)
        calibrationLayout.addWidget(pixelsize_label, 3, 0, 1, 1)
        calibrationLayout.addWidget(self.pixelsize_unit_label, 3, 1, 1, 1)
        calibrationLayout.addWidget(pixelsize_units_label, 3, 2, 1, 1)
        calibrationContainer.setLayout(calibrationLayout)
        
        """
        ------------------------- Camera view display -------------------------
        """
        liveContainer = QGroupBox("Field-of-view")
        liveContainer.setMinimumWidth(700)
        liveLayout = QGridLayout()
        
        # Display to project live camera view
        liveWidget = pg.ImageView()
        liveWidget.ui.roiBtn.hide()
        liveWidget.ui.menuBtn.hide()
        liveWidget.ui.histogram.hide()
        self.liveView = liveWidget.getView()
        self.liveImageItem = liveWidget.getImageItem()
        self.liveImageItem.setAutoDownsample(True)
        
        # Button for pausing camera view
        self.request_pause_button = QPushButton(text="Pause live", clicked=self.mockfunction)
        self.request_pause_button.setCheckable(True)
        
        # Button for clearing phantoms
        request_clear_drawings = QPushButton(text="Clear phantoms", clicked=self.mockfunction)
        
        # Button for reseting the field of view
        request_reset_fov = QPushButton(text="Reset view", clicked=self.mockfunction)
        
        # Button for snapshotting
        self.request_snap_button = QPushButton(text="Take snap", clicked=self.mockfunction)
        
        liveLayout.addWidget(liveWidget, 0, 0, 1, 4)
        liveLayout.addWidget(self.request_pause_button, 1, 0, 1, 1)
        liveLayout.addWidget(request_clear_drawings, 1, 1, 1, 1)
        liveLayout.addWidget(request_reset_fov, 1, 2, 1, 1)
        liveLayout.addWidget(self.request_snap_button, 1, 3, 1, 1)
        liveContainer.setLayout(liveLayout)
        
        
        """
        ---------------------- Autopatch control buttons ----------------------
        """
        autopatchContainer = QGroupBox("Autoptach algorithms")
        autopatchLayout = QGridLayout()
        
        # Buttons for autopatch algorithms
        request_prechecks = QPushButton(text="Pre-checks", clicked=self.mockfunction)
        request_selecttarget = QPushButton(text="Select target", clicked=self.mockfunction)
        request_start_autopatch = QPushButton(text="Start autopatch", clicked=self.mockfunction)
        request_manual_calibration = QPushButton(text="Correct tip location", clicked=self.mockfunction)
        request_start_approach = QPushButton(text="Start from target approach", clicked=self.mockfunction)
        request_start_gigaseal = QPushButton(text="Start from gigaseal", clicked=self.mockfunction)
        request_start_breakin = QPushButton(text="Start from break-in", clicked=self.mockfunction)
        request_stop = QPushButton(text="STOP!", clicked=self.mockfunction)
        
        # Labels for autopatch status and warning
        autopatch_status_label = QLabel("Autopatch status:")
        autopatch_warning_label = QLabel("Warning:")
        self.autopatch_status = QLabel("Status")
        self.autopatch_warning = QLabel("Warning")
        self.autopatch_status.setFont(QFont("Times", weight=QFont.Bold))
        self.autopatch_warning.setFont(QFont("Times", weight=QFont.Bold))
        
        autopatchLayout.addWidget(request_prechecks, 0, 0, 1, 2)
        autopatchLayout.addWidget(request_selecttarget, 1, 0, 1, 2)
        autopatchLayout.addWidget(request_start_autopatch, 2, 0, 1, 2)
        autopatchLayout.addWidget(request_manual_calibration, 3, 0, 1, 2)
        autopatchLayout.addWidget(request_start_approach, 4, 0, 1, 2)
        autopatchLayout.addWidget(request_start_gigaseal, 5, 0, 1, 2)
        autopatchLayout.addWidget(request_start_breakin, 6, 0, 1, 2)
        autopatchLayout.addWidget(request_stop, 7, 0, 1, 2)
        autopatchLayout.addWidget(autopatch_status_label, 8, 0, 1, 1)
        autopatchLayout.addWidget(self.autopatch_status, 8, 1, 1, 1)
        autopatchLayout.addWidget(autopatch_warning_label, 9, 0, 1, 1)
        autopatchLayout.addWidget(self.autopatch_warning, 9, 1, 1, 1)
        autopatchContainer.setLayout(autopatchLayout)
        
        """
        ----------------------- Pressure control layout -----------------------
        """
        pressurecontrolContainer = QGroupBox(title="Pressure control")
        pressurecontrolLayout = QGridLayout()
        
        # Labels set and read pressure
        pressure_set_label = QLabel("Request pressure:")
        pressure_read_label = QLabel("Readout pressure:")
        pressure_units_label1 = QLabel("mBar")
        pressure_units_label2 = QLabel("mBar")
        
        # Spinbox to set pressure
        self.set_pressure_spinbox = QDoubleSpinBox()
        self.set_pressure_spinbox.setMinimum(-300)
        self.set_pressure_spinbox.setMaximum(200)
        self.set_pressure_spinbox.setDecimals(0)
        self.set_pressure_spinbox.setValue(0)
        self.set_pressure_spinbox.setSingleStep(10)
        
        # Label pressure readout value
        self.pressure_value_Label = QLabel("-")
        self.pressure_value_Label.setFont(QFont("Times", weight=QFont.Bold))
        
        # Label pressure status
        pressure_statustext_label = QLabel("Pressure status:")
        self.pressure_status_label = QLabel("Disconnected")
        self.pressure_status_label.setFont(QFont("Times", weight=QFont.Bold))
        
        # Buttons for relatively valued pressures
        low_positive_button = QPushButton(text="Low +", clicked=self.mockfunction)
        low_negative_button = QPushButton(text="Low -", clicked=self.mockfunction)
        high_positive_button = QPushButton(text="High +", clicked=self.mockfunction)
        high_negative_button = QPushButton(text="High -", clicked=self.mockfunction)
        stop_regulating_button = QPushButton(text="Set once", clicked=self.mockfunction)
        atmoshpere_button = QPushButton(text="Atm", clicked=self.mockfunction)
        
        # Buttons for operating modes
        confirm_button = QPushButton(text="Set as pressure", clicked=self.mockfunction)
        spike_button = QPushButton(text="Set as pulse", clicked=self.mockfunction)
        self.toggle_lcd_button = QPushButton(text="LCD off", clicked=self.mockfunction)
        self.toggle_lcd_button.setCheckable(True)
        
        pressurecontrolLayout.addWidget(pressure_set_label, 0, 0, 1, 2)
        pressurecontrolLayout.addWidget(self.set_pressure_spinbox, 0, 2, 1, 1)
        pressurecontrolLayout.addWidget(pressure_units_label1, 0, 3, 1, 1)
        pressurecontrolLayout.addWidget(pressure_read_label, 1, 0, 1, 2)
        pressurecontrolLayout.addWidget(self.pressure_value_Label, 1, 2, 1, 1)
        pressurecontrolLayout.addWidget(pressure_units_label2, 1, 3, 1, 1)
        pressurecontrolLayout.addWidget(pressure_statustext_label, 2, 0, 1, 2)
        pressurecontrolLayout.addWidget(self.pressure_status_label, 2, 2, 1, 2)
        pressurecontrolLayout.addWidget(low_positive_button, 3, 0, 1, 1)
        pressurecontrolLayout.addWidget(low_negative_button, 3, 1, 1, 1)
        pressurecontrolLayout.addWidget(high_positive_button, 4, 0, 1, 1)
        pressurecontrolLayout.addWidget(high_negative_button, 4, 1, 1, 1)
        pressurecontrolLayout.addWidget(stop_regulating_button, 5, 0, 1, 1)
        pressurecontrolLayout.addWidget(atmoshpere_button, 5, 1, 1, 1)
        pressurecontrolLayout.addWidget(confirm_button, 3, 2, 1, 2)
        pressurecontrolLayout.addWidget(spike_button, 4, 2, 1, 2)
        pressurecontrolLayout.addWidget(self.toggle_lcd_button, 5, 2, 1, 2)
        pressurecontrolContainer.setLayout(pressurecontrolLayout)
        
        """
        -------------------------- Electrophysiology --------------------------
        """
        electrophysiologyContainer = QGroupBox("Electrophysiology")
        electrophysiologyLayout = QGridLayout()
        
        # Labels for electrophysiology
        resistance_label = QLabel("Resistance: ")
        self.resistance_value_label = QLabel("-")
        self.resistance_value_label.setFont(QFont("Times", weight=QFont.Bold))
        resistance_unit_label = QLabel("MΩ")
        capacitance_label = QLabel("Capacitance: ")
        self.capacitance_value_label = QLabel("-")
        self.capacitance_value_label.setFont(QFont("Times", weight=QFont.Bold))
        capacitance_unit_label = QLabel("F/μm?")
        ratio_label = QLabel("Ratio: ")
        self.ratio_value_label = QLabel("-")
        membranevoltage_label = QLabel("Membrane potential:")
        self.membranevoltage_value_label = QLabel("-")
        membranevoltage_unit_label = QLabel("V")
        
        request_zap_button = QPushButton(text="ZAP!", clicked=self.mockfunction)
        
        electrophysiologyLayout.addWidget(resistance_label, 0, 0, 1, 1)
        electrophysiologyLayout.addWidget(self.resistance_value_label, 0, 1, 1, 1)
        electrophysiologyLayout.addWidget(resistance_unit_label, 0, 2, 1, 1)
        electrophysiologyLayout.addWidget(capacitance_label, 1, 0, 1, 1)
        electrophysiologyLayout.addWidget(self.capacitance_value_label, 1, 1, 1, 1)
        electrophysiologyLayout.addWidget(capacitance_unit_label, 1, 2, 1, 1)
        electrophysiologyLayout.addWidget(ratio_label, 2, 0, 1, 1)
        electrophysiologyLayout.addWidget(self.ratio_value_label, 2, 1, 1, 1)
        electrophysiologyLayout.addWidget(membranevoltage_label, 3, 0, 1, 1)
        electrophysiologyLayout.addWidget(self.membranevoltage_value_label, 3, 1, 1, 1)
        electrophysiologyLayout.addWidget(membranevoltage_unit_label, 3, 2, 1, 1)
        electrophysiologyLayout.addWidget(request_zap_button, 4, 0, 1, 1)
        electrophysiologyContainer.setLayout(electrophysiologyLayout)
        
        """
        --------------------------- Sensor display ----------------------------
        """
        sensorContainer = QGroupBox()
        sensorLayout = QGridLayout()
        
        # Algorithm plot
        algorithmPlot = pg.PlotWidget()
        algorithmPlot.setTitle("Sharpness graph")
        algorithmPlot.setLabel("left", text="a.u.")
        algorithmPlot.setLabel("bottom", text="a.u.")
        self.algorithmPlot = algorithmPlot.plot(pen=(1,3))
        
        # Current plot
        currentPlot = pg.PlotWidget()
        currentPlot.setTitle("Current")
        currentPlot.setLabel("left", units="A")
        currentPlot.setLabel("bottom", text="20 ms")
        self.currentPlot = currentPlot.plot(pen=(2,3))
        
        # Resistance plot
        resistancePlot = pg.PlotWidget()
        resistancePlot.setTitle("Resistance")
        resistancePlot.setLabel("left", units="Ω")
        resistancePlot.setLabel("bottom", text="20 ms")
        self.resistancePlot = resistancePlot.plot(pen=(2,3))
        
        # Reset plot button
        request_resetplots_button = QPushButton(text="Reset all plots", clicked=self.mockfunction)
        
        # Create plot tabs
        self.displayWidget = QTabWidget()
        self.displayWidget.addTab(algorithmPlot, "Algorithm")
        self.displayWidget.addTab(resistancePlot, "Resistance")
        self.displayWidget.addTab(currentPlot, "Current")
        
        sensorLayout.addWidget(self.displayWidget, 0, 0, 1, 1)
        sensorLayout.addWidget(request_resetplots_button, 1, 0, 1, 1)
        sensorContainer.setLayout(sensorLayout)
        
        """
        ---------------------- Add widgets and set Layout ---------------------
        """
        # layout = QGridLayout()
        # layout.addWidget(hardwareContainer, 0, 0, 1, 1)
        # layout.addWidget(stagemoveContainer, 1, 0, 1, 1)
        # layout.addWidget(calibrationContainer, 2, 0, 1, 1)
        # layout.addWidget(liveContainer, 0, 1, 3, 3)
        # layout.addWidget(autopatchContainer, 0, 4, 2, 1)
        # layout.addWidget(pressurecontrolContainer, 0, 5, 1, 1)
        # layout.addWidget(electrophysiologyContainer, 1, 5, 1, 1)
        # layout.addWidget(sensorContainer, 2, 4, 1, 2)
        # self.setLayout(layout)
        
        
        layout = QGridLayout()
        layout.addWidget(hardwareContainer, 0, 0, 2, 1)
        layout.addWidget(stagemoveContainer, 2, 0, 1, 1)
        layout.addWidget(calibrationContainer, 3, 0, 1, 1)
        
        layout.addWidget(liveContainer, 0, 1, 4, 4)
        
        layout.addWidget(autopatchContainer, 0, 5, 2, 1)
        layout.addWidget(pressurecontrolContainer, 0, 6, 1, 1)
        layout.addWidget(electrophysiologyContainer, 1, 6, 1, 1)
        layout.addWidget(sensorContainer, 2, 5, 2, 2)
        self.setLayout(layout)
        
        """
        --------------------------- Stack old GUI's ---------------------------
        """
        
        """
        =======================================================================
        ----------------------------- End of GUI ------------------------------
        =======================================================================
        """
        
        
        
        
    def mockfunction(self):
        pass
        
    
    
    
    
    def closeEvent(self, event):
        """ Close event
        This method is called when the GUI is shut down. First we need to stop
        the threads that are still running, then we disconnect all hardware to
        be reused in the main widget, only then we accept the close event.
        and quit the widget.
        """
        
        event.accept()
        
        # Frees the console by quitting the application entirely
        QtWidgets.QApplication.quit() # remove when part of Tupolev!!
        


if __name__ == "__main__":
    
    # Create a palette to switch to dark mode colors
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, Qt.black)
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    
    def run_app():
        app = QtWidgets.QApplication(sys.argv)
        app.setStyle("Fusion")
        app.setPalette(palette)
        pg.setConfigOptions(imageAxisOrder="row-major")
        mainwin = PatchClampUI()
        mainwin.show()
        app.exec_()
        
    run_app()
