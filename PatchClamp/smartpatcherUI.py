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
from PatchClamp.smartpatcher import SmartPatcher
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
        self.connect_camerathread_button = QPushButton(text="Camera", clicked=self.connect_camerathread)
        self.connect_camerathread_button.setCheckable(True)
        
        # Button to (dis)connect objective motor
        self.connect_objectivemotor_button = QPushButton(text="Objective motor", clicked=self.connect_objectivemotor)
        self.connect_objectivemotor_button.setCheckable(True)
        
        # Button to (dis)connect micromanipulator
        self.connect_micromanipulator_button = QPushButton(text="Micromanipulator", clicked=self.connect_micromanipulator)
        self.connect_micromanipulator_button.setCheckable(True)
        
        # Button to (dis)connect XYstage
        self.connect_XYstage_button = QPushButton(text="XY stage", clicked=self.connect_XYstage)
        self.connect_XYstage_button.setCheckable(True)
        
        # Button to (dis)connect NIDAQ for sealtest
        self.connect_sealtestthread_button = QPushButton(text="NIDAQ/Sealtest", clicked=self.connect_sealtestthread)
        self.connect_sealtestthread_button.setCheckable(True)
        
        # Button to (dis)connect pressure controller
        self.connect_pressurecontroller_button = QPushButton(text="Pressure controller", clicked=self.connect_pressurethread)
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
        
        # Stage translation
        request_stage_up_button = QPushButton(text="up", clicked=self.request_stage_up)
        request_stage_down_button = QPushButton(text="down", clicked=self.request_stage_down)
        request_stage_right_button = QPushButton(text="right", clicked=self.request_stage_right)
        request_stage_left_button = QPushButton(text="left", clicked=self.request_stage_left)
        
        # Button for moving target to center
        request_target2center_button = QPushButton(text="Target to center", clicked=self.request_target2center)
        
        stagemoveLayout.addWidget(request_stage_up_button, 0, 1, 1, 1)
        stagemoveLayout.addWidget(request_stage_down_button, 2, 1, 1, 1)
        stagemoveLayout.addWidget(request_stage_right_button, 1, 2, 1, 1)
        stagemoveLayout.addWidget(request_stage_left_button, 1, 0, 1, 1)
        stagemoveLayout.addWidget(request_target2center_button, 3, 0, 1, 3)
        stagemoveContainer.setLayout(stagemoveLayout)
        
        """
        ------------------------- Calibration options -------------------------
        """
        calibrationContainer = QGroupBox("Calibration options")
        calibrationLayout = QGridLayout()
        
        # Calibration algorithms
        request_calibrate_xy_button = QPushButton(text="Calibrate XY", clicked=self.request_calibrate_xy)
        request_calibrate_pixelsize_button = QPushButton(text="Calibrate pixelsize", clicked=self.request_calibrate_pixelsize)
        
        # Calibration statistics
        rotation_angles_label = QLabel("Rotation axes (α,β,γ):")
        self.rotation_angles_value_label = QLabel("-")
        self.rotation_angles_value_label.setFont(QFont("Times", weight=QFont.Bold))
        rotation_angles_units_label = QLabel("degrees")
        pixelsize_label = QLabel("Pixelsize (hxw):")
        self.pixelsize_unit_label = QLabel("-")
        self.pixelsize_unit_label.setFont(QFont("Times", weight=QFont.Bold))
        pixelsize_units_label = QLabel("nm")
        
        calibrationLayout.addWidget(request_calibrate_xy_button, 0, 0, 1, 4)
        calibrationLayout.addWidget(request_calibrate_pixelsize_button, 1, 0, 1, 4)
        calibrationLayout.addWidget(rotation_angles_label, 2, 0, 1, 1)
        calibrationLayout.addWidget(self.rotation_angles_value_label, 2, 1, 1, 2)
        calibrationLayout.addWidget(rotation_angles_units_label, 2, 3, 1, 1)
        calibrationLayout.addWidget(pixelsize_label, 3, 0, 1, 1)
        calibrationLayout.addWidget(self.pixelsize_unit_label, 3, 1, 1, 2)
        calibrationLayout.addWidget(pixelsize_units_label, 3, 3, 1, 1)
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
        self.request_pause_button = QPushButton(text="Pause live", clicked=self.toggle_pauselive)
        self.request_pause_button.setCheckable(True)
        
        # Button for clearing phantoms
        request_clear_drawings = QPushButton(text="Clear phantoms", clicked=self.clearROIs)
        
        # Button for reseting the field of view
        request_reset_fov = QPushButton(text="Reset view", clicked=self.reset_imageview)
        
        # Button for snapshotting
        self.request_snap_button = QPushButton(text="Take snap", clicked=self.request_snap)
        
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
        self.pressure_status_label = QLabel("-")
        self.pressure_status_label.setFont(QFont("Times", weight=QFont.Bold))
        
        # Buttons for relatively valued pressures
        low_positive_button = QPushButton(text="Low +", clicked=self.request_low_positive)
        low_negative_button = QPushButton(text="Low -", clicked=self.request_low_negative)
        high_positive_button = QPushButton(text="High +", clicked=self.request_high_positive)
        high_negative_button = QPushButton(text="High -", clicked=self.request_high_negative)
        stop_regulating_button = QPushButton(text="Set once", clicked=self.request_stop_regulating)
        atmoshpere_button = QPushButton(text="ATM", clicked=self.request_atmosphere)
        
        # Buttons for operating modes
        apply_pressure_button = QPushButton(text="Apply pressure", clicked=self.request_apply_pressure)
        spike_pulse_button = QPushButton(text="Apply pulse", clicked=self.request_apply_pulse)
        self.toggle_lcd_button = QPushButton(text="LCD off", clicked=self.toggle_lcd)
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
        pressurecontrolLayout.addWidget(apply_pressure_button, 3, 2, 1, 2)
        pressurecontrolLayout.addWidget(spike_pulse_button, 4, 2, 1, 2)
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
        self.algorithmPlot = algorithmPlot.plot(pen=(1,4))
        
        # Current plot
        currentPlot = pg.PlotWidget()
        currentPlot.setTitle("Current")
        currentPlot.setLabel("left", units="A")
        currentPlot.setLabel("bottom", text="20 ms")
        self.currentPlot = currentPlot.plot(pen=(2,4))
        
        # Resistance plot
        resistancePlot = pg.PlotWidget()
        resistancePlot.setTitle("Resistance")
        resistancePlot.setLabel("left", units="Ω")
        resistancePlot.setLabel("bottom", text="20 ms")
        self.resistancePlot = resistancePlot.plot(pen=(3,4))
        
        # Pressure plot
        pressurePlot = pg.PlotWidget()
        pressurePlot.setTitle("Pressure")
        pressurePlot.setLabel("left", units="mBar")
        pressurePlot.setLabel("bottom", text="time", units="s")
        self.pressurePlot = pressurePlot.plot(pen=(4,4))
        
        # Reset plot button
        request_resetplots_button = QPushButton(text="Reset all plots", clicked=self.reset_plots)
        
        # Create plot tabs
        self.displayWidget = QTabWidget()
        self.displayWidget.addTab(algorithmPlot, "Algorithm")
        self.displayWidget.addTab(resistancePlot, "Resistance")
        self.displayWidget.addTab(currentPlot, "Current")
        self.displayWidget.addTab(pressurePlot, "Pressure")
        
        sensorLayout.addWidget(self.displayWidget, 0, 0, 1, 1)
        sensorLayout.addWidget(request_resetplots_button, 1, 0, 1, 1)
        sensorContainer.setLayout(sensorLayout)
        
        """
        ---------------------- Add widgets and set Layout ---------------------
        """
        layout = QGridLayout()
        layout.addWidget(hardwareContainer, 0, 0, 2, 1)
        layout.addWidget(stagemoveContainer, 2, 0, 2, 1)
        layout.addWidget(calibrationContainer, 4, 0, 1, 1)
        layout.addWidget(liveContainer, 0, 1, 5, 5)
        layout.addWidget(autopatchContainer, 0, 6, 2, 1)
        layout.addWidget(pressurecontrolContainer, 0, 7, 1, 1)
        layout.addWidget(electrophysiologyContainer, 1, 7, 1, 1)
        layout.addWidget(sensorContainer, 2, 6, 3, 2)
        self.setLayout(layout)
        
        """
        --------------------------- Stack old GUI's ---------------------------
        """
        """
        =======================================================================
        ------------------------ Start up roi manager -------------------------
        =======================================================================
        """
        self.roimanager = ROIManagerGUI(offset=len(self.liveView.addedItems))
        
        """
        =======================================================================
        -------------- Start up backend and connect signals/slots -------------
        =======================================================================
        """
        self.backend = SmartPatcher()
        # self.backend.worker.draw.connect(self.draw_roi)
        # self.backend.worker.graph.connect(self.update_algorithmplot)
        # self.backend.worker.state.connect(self.update_autopatch_status)
        self.backend.worker.finished.connect(self.update_constants_from_backend)
        
        """
        =======================================================================
        --------------------- Update constant from backend --------------------
        =======================================================================
        """
        self.update_constants_from_backend()
        
        """
        =======================================================================
        ----------------------------- End of GUI ------------------------------
        =======================================================================
        """
    
    
    
    
    
    
    def mockfunction(self):
        self.backend.request(name='mockworker')
    
    
    
    
    def connect_camerathread(self):
        """
        We initiate the camera by creating the CameraThread object, then we
        connect signals with slots to govern inter-thread communication, i.e.
        to recieve images to display. Afterwards we move the camera thread to
        the backend where the thread is immediately started.
        """
        if self.connect_camerathread_button.isChecked():
            try:
                camerathread = CameraThread(camerahandle=None)
                self.signal_camera_live = camerathread.livesignal
                self.signal_camera_live.connect(self.update_live)
                self.backend.camerathread = camerathread
            except:
                self.connect_camerathread_button.setChecked(False)
                raise
        else:
            del self.backend.camerathread
            del self.signal_camera_live
    
    def connect_objectivemotor(self):
        """
        We initiate the objective motor by creating the PIMotor object. This
        takes time in which the GUI freezes, this issue can be solved by
        passing the objective motor instance from Tupolev/Fiumicino as an
        objective_motor_handle. Deleting the object disconnects the objective
        motor.
        """
        if self.connect_objectivemotor_button.isChecked():
            try:
                objectivemotor = PIMotor(objective_motor_handle=None)
                self.backend.objectivemotor = objectivemotor
            except:
                self.connect_objectivemotor_button.setChecked(False)
                raise
        else:
            del self.backend.objectivemotor
    
    def connect_micromanipulator(self):
        """
        We initiate the micromanipulator by creating the ScientificaPatchStar
        serial object. Creating the object object automatically opens the USB
        port that the device is connected to, similarly, deleting the object
        stops micromanipulator movement and closes the port.
        """
        if self.connect_micromanipulator_button.isChecked():
            try:
                micromanipulator = ScientificaPatchStar(address='COM16', baud=38400)
                self.backend.micromanipulator = micromanipulator
            except:
                self.connect_micromanipulator_button.setChecked(False)
                raise
        else:
            del self.backend.micromanipulator
    
    def connect_XYstage(self):
        """
        We initiate the XYstage (sample stage) by creating the LudlStage object
        Creating the object does not automatically open the USB port to which
        the device is connected to. Deleting the object stops stage movement.
        """
        if self.connect_XYstage_button.isChecked():
            try:
                ludlStage = LudlStage('COM12')
                self.backend.XYstage = ludlStage
            except:
                self.connect_XYstage_button.setChecked(False)
                raise
        else:
            del self.backend.XYstage
    
    def connect_sealtestthread(self):
        """
        We initiate the sealtest thread by creating the SealTestThread object,
        then we connect the measurement signal to update the graph for voltage
        and current. Afterwards we move the sealtest thread to the backend
        where the thread is immediately started.
        """
        if self.connect_sealtestthread_button.isChecked():
            try:
                sealtestthread = SealTestThread()
                self.signal_sealtest = sealtestthread.measurement
                self.signal_sealtest.connect(self.update_current_voltage)
                self.backend.sealtestthread = sealtestthread
            except:
                self.connect_sealtestthread_button.setChecked(False)
                raise
        else:
            del self.backend.sealtestthread
            del self.signal_sealtest
    
    def connect_pressurethread(self):
        """
        We initiate the pressure thread by creating the PressureThread object,
        then we connect the measurement signal to update the pressure evolution
        graph. Afterwards we move the sealtest thread to the backend
        where the thread is immediately started.
        """
        if self.connect_pressurecontroller_button.isChecked():
            try:
                pressurethread = PressureThread(pressurecontroller_handle=None)
                self.signal_pressure = pressurethread.measurement
                self.signal_pressure.connect(self.update_pressure)
                self.backend.pressurethread = pressurethread
            except:
                self.connect_sealtestthread_button.setChecked(False)
                raise
        else:
            del self.backend.pressurethread
            del self.signal_pressure
    
    
    def request_stage_up(self):
        step_height = self.backend.image_size[0]*self.backend.pixel_size
        self.backend.XYstage.moveRel(xRel=step_height, yRel=0)
    
    def request_stage_down(self):
        step_height = self.backend.image_size[0]*self.backend.pixel_size
        self.backend.XYstage.moveRel(xRel=-step_height, yRel=0)
    
    def request_stage_right(self):
        step_width = self.backend.image_size[1]*self.backend.pixel_size
        self.backend.XYstage.moveRel(xRel=0, yRel=-step_width)
    
    def request_stage_left(self):
        step_width = self.backend.image_size[1]*self.backend.pixel_size
        self.backend.XYstage.moveRel(xRel=0, yRel=step_width)
    
    def request_target2center(self):
        self.backend.request(name='target2center')
    
    
    def request_calibrate_xy(self):
        self.backend.request(name='hardcalibration', mode='XY')
    
    def request_calibrate_pixelsize(self):
        self.backend.request(name='hardcalibration', mode='pixelsize')
    
    
    def toggle_pauselive(self):
        if hasattr(self, 'signal_camera_live'):
            try:
                self.signal_camera_live.disconnect()
            except TypeError:
                pass
            if self.request_pause_button.isChecked():
                self.request_pause_button.setChecked(True)
            else:
                self.signal_camera_live.connect(self.update_live)
                self.request_pause_button.setChecked(False)
        else:
            self.request_pause_button.setChecked(False)
    
    def clearROIs(self):
        self.roimanager.removeallROIs()
        for roi in self.liveView.addedItems[self.roimanager.offset::]:
            self.liveView.removeItem(roi)
    
    def reset_imageview(self):
        self.liveView.autoRange()
        
    def request_snap(self):
        if self.backend.camerathread is not None:
            I = self.backend.camerathread.snap()
            io.imsave(self.backend.save_directory+'.tif', I, check_contrast=False)
            self.toggle_pauselive()
            self.update_live(I)
        else:
            raise AttributeError('no camera connected')
    
    
    def request_low_positive(self):
        pressure = 30
        self.backend.pressurethread.set_pressure_stop_waveform(pressure)
        self.pressure_status_label.setText("Regulating")
    
    def request_low_negative(self):
        pressure = -30
        self.backend.pressurethread.set_pressure_stop_waveform(pressure)
        self.pressure_status_label.setText("Regulating")
    
    def request_high_positive(self):
        pressure = 100
        self.backend.pressurethread.set_pressure_stop_waveform(pressure)
        self.pressure_status_label.setText("Regulating")
    
    def request_high_negative(self):
        pressure = -100
        self.backend.pressurethread.set_pressure_stop_waveform(pressure)
        self.pressure_status_label.setText("Regulating")
    
    def request_stop_regulating(self):
        pressure = self.set_pressure_spinbox.value()
        self.backend.pressurethread.set_pressure_hold_stop_waveform(pressure)
        self.pressure_status_label.setText("Not regulating")
    
    def request_atmosphere(self):
        pressure = 0
        self.backend.pressurethread.set_pressure_stop_waveform(pressure)
        self.pressure_status_label.setText("Not regulating")
    
    def request_apply_pressure(self):
        pressure = self.set_pressure_spinbox.value()
        self.backend.pressurethread.set_pressure_stop_waveform(pressure)
        self.pressure_status_label.setText("Regulating")
    
    def request_apply_pulse(self):
        pressure = self.set_pressure_spinbox.value()
        self.backend.pressurethread.set_pulse_stop_waveform(pressure)
        self.pressure_status_label.setText("Doing pulse")
    
    def toggle_lcd(self):
        if self.backend.pressurethread is not None:
            if self.toggle_lcd_button.isChecked():
                self.backend.pressurethread.pressurecontroller.LCDoff()
            else:
                self.backend.pressurethread.pressurecontroller.LCDon()
        else:
            self.toggle_lcd_button.setChecked(False)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    def update_live(self, image):
        """
        Image data is retrieved per image and directly displayed.
        """
        self.liveImageItem.setImage(image)
    
    def update_pressure(self, data):
        """
        Pressure data is retrieved as [pressure, timing] and processed in the
        backend. We retrieve the processed pressure as an array that we can
        display.
        """
        self.backend._pressure_append_(data[0], data[1])
        pressuredata = self.backend.pressure
        self.pressurePlot.setData(pressuredata[1], pressuredata[0])
        self.pressure_value_Label.setText("{:.1f}".format(pressuredata[0,-1]))
    
    def update_current_voltage(self, voltOut, curOut):
        """
        Voltage and current are retrieved, converted to their right units, and
        processed in the backend. We retrieve the voltage and current as an
        array that we can display and/or further process to calculate the
        resistance and capacitance from.
        """
        self.backend._voltage_append_(voltOut / 10)
        self.backend._current_append_(curOut / 1 / (100*10**6))
        # voltage = self.backend.voltage
        current = self.backend.current
        self.currentPlot.setData(current)
        self.update_resistance_capacitance(self.backend.current, self.backend.voltage)
    
    def update_resistance_capacitance(self, curOut, voltOut):
        """ Update the resistance and capacitance labels.
        http://scipy-lectures.org/intro/scipy/auto_examples/plot_curve_fit.html
        https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.curve_fit.html
        This function was taken from lhuismans and modified by TvdrBurgt.
        """
        constants = MeasurementConstants()
        sampPerCyc = int(constants.patchSealSampRate / constants.patchSealFreq)

        try:
            curOutCyc = curOut.reshape(int(curOut.size / sampPerCyc), sampPerCyc)
            curData = np.mean(curOutCyc, axis=0)
        except:
            curData = curOut

        voltData = voltOut
        try:
            # Computing resistance
            tres = np.mean(voltData)
            dV = np.mean(voltData[voltData > tres]) - np.mean(voltData[voltData < tres])  # Computing the voltage difference
            dIss = np.mean(curData[int(np.floor(0.15 * sampPerCyc)) : int(np.floor(sampPerCyc / 2)) - 2]) - np.mean(curData[int(np.floor(0.65 * sampPerCyc)) : sampPerCyc - 2])  # Computing the current distance
            membraneResistance = dV / (dIss * 1000000)  # Ohms law (MegaOhm)
            self.resistance_value_label.setText("{:.1f}".format(membraneResistance))
            R_to_append = membraneResistance*1e6

            estimated_size_resistance = 10000 / (membraneResistance * 1000000)  # The resistance of a typical patch of membrane, RM is 10000 Omega/{cm}^2
        except:
            self.resistance_value_label.setText("NaN")
            R_to_append = np.nan

        try:
            measured_vlotage = np.mean(voltData) * 1000
            self.membranevoltage_value_label.setText("{:.1f}".format(measured_vlotage))
        except:
            self.membranevoltage_value_label.setText("NaN")
        try:
            # Computing capacitance
            points = 10
            maxCur = np.amax(curData)
            maxCurIndex = np.where(curData == maxCur)[0][0]
            curFit = curData[int(maxCurIndex+1) : int(maxCurIndex+1+points-1)] - \
                0.5*(np.mean(curData[int(np.floor(0.15*sampPerCyc)) : int(np.floor(sampPerCyc/2)) - 2]) + \
                     np.mean(curData[int(np.floor(0.65*sampPerCyc)) : sampPerCyc-2]))
            timepoints = 1000*np.arange(3, points - 1 + 3) / constants.patchSealSampRate
            
            # Fitting the data to an exponential of the form y=a*exp(-b*x) where b = 1/tau and tau = RC
            # I(t)=I0*e^−t/τ, y=a*exp(-b*x), get log of both sides:log y = -bx + log a
            fit = np.polyfit(timepoints, curFit, 1)  # Converting the exponential to a linear function and fitting it
            
            # Extracting data
            current = fit[0]
            resistance = dV * 1000 / current / 2  # Getting the resistance
            tau = -1 / fit[1]
            capacitance = 1000 * tau / resistance
            self.capacitance_value_label.setText("{:.1f}".format(capacitance))
            C_to_append = capacitance

            estimated_size_capacitance = capacitance * (10 ** -12) * (10 ** 6)

            if estimated_size_capacitance > estimated_size_resistance:
                estimated_ratio = (estimated_size_capacitance / estimated_size_resistance)
            else:
                estimated_ratio = (estimated_size_resistance / estimated_size_capacitance)

            self.ratio_value_label.setText("{:.1f}".format(estimated_ratio))  # http://www.cnbc.cmu.edu/~bard/passive2/node5.html

        except:
            self.capacitance_value_label.setText("NaN")
            C_to_append = np.nan
            self.ratio_value_label.setText("NaN")
        
        # store resistance and capacitance values in backend
        self.backend._resistance_append_(R_to_append)
        self.backend._capacitance_append_(C_to_append)
    
    
    def reset_plots(self):
        del self.backend.current
        del self.backend.voltage
        del self.backend.pressure
        self.algorithmPlot.setData([0])
        self.currentPlot.setData([0])
        self.resistancePlot.setData([0])
        self.pressurePlot.setData([0])
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    def update_constants_from_backend(self):
        # self._image_size = [2048, 2048]             # dimension of FOV in pix
        # self._pipette_orientation = 0               # in radians
        # self._pipette_diameter = 16                 # in pixels (16=patchclamp, ??=cell-picking)
        # self._focus_offset = 30                     # in micron above coverslip
        
        # retrieve constants from backend
        alpha = self.backend.rotation_angles[0]*180/np.pi
        beta = self.backend.rotation_angles[1]*180/np.pi
        gamma = self.backend.rotation_angles[2]*180/np.pi
        pixelsize = self.backend.pixel_size
        
        # update labels
        self.rotation_angles_value_label.setText("({:.2f}".format(alpha)+", {:.2f}, ".format(beta)+"{:.2f})".format(gamma))
        self.pixelsize_unit_label.setText("{:.1f}".format(pixelsize)+"x{:.1f}".format(pixelsize))
    
    
    
    
    
    def unlock_pressure_control(self):
        # Set radout pressure to "-"
        # Set pressure status to (dis)connected
        # Unlock the pressure control widget
        # Unlock other pressure control dependent buttons
        pass
        
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    def closeEvent(self, event):
        """ Close event
        This method is called when the GUI is shut down. First we need to stop
        the threads that are still running, then we disconnect all hardware to
        be reused in the main widget, only then we accept the close event.
        and quit the widget.
        """
        try:
            del self.backend.camerathread
        except AttributeError:
            pass
        try:
            del self.backend.objectivemotor
        except AttributeError:
            pass
        try:
            del self.backend.micromanipulator
        except AttributeError:
            pass
        try:
            del self.backend.XYstage
        except AttributeError:
            pass
        try:
            del self.backend.sealtestthread
        except AttributeError:
            pass
        try:
            del self.backend.pressurethread
        except AttributeError:
            pass
        
        event.accept()
        
        # Frees the console by quitting the application entirely
        QtWidgets.QApplication.quit() # remove when part of Tupolev!!
        








class ROIManagerGUI:
    def __init__(self, offset):
        self.offset = offset
        self.ROInumber = 0
        self.ROIdictionary = {}
    
    def giveROIindex(self, name):
        if self.contains(name):
            index_array = [x+self.offset for x in self.ROIdictionary[name][:]]
        else:
            index_array = []
        return index_array
    
    def addROI(self, name):
        if name in self.ROIdictionary:
            self.ROIdictionary[name] = self.ROIdictionary[name] + [self.ROInumber]
            self.ROInumber += 1
        else:
            self.ROIdictionary[name] = [self.ROInumber]
            self.ROInumber += 1
        
    def removeROI(self, name, args=None):
        if args == 'all':
            self.ROInumber -= len(self.ROIdictionary[name])
            del self.ROIdictionary[name]
        elif type(args) == int:
            entries = len(self.ROIdictionary[name])
            if entries > args:
                self.ROInumber -= args
                self.ROIdictionary[name] = self.ROIdictionary[name][:-args]
            elif entries == args:
                self.ROInumber -= entries
                del self.ROIdictionary[name]
            else:
                raise ValueError('list out of range')
                
    def removeallROIs(self):
        name = list(self.ROIdictionary)
        for name in name:
            self.removeROI(name, args='all')
                
    def contains(self, name):
        if name in self.ROIdictionary:
            return True
        else:
            return False








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
