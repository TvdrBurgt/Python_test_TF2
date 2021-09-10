# -*- coding: utf-8 -*-
"""
Created on Fri Aug  6 15:15:38 2021

@author: tvdrb
"""


import os
import sys
import numpy as np
import logging

from PyQt5 import QtWidgets
from PyQt5.QtGui import QPen, QColor
from PyQt5.QtWidgets import QWidget, QGridLayout, QPushButton, QDoubleSpinBox, QGroupBox, QLabel
import pyqtgraph.exporters
import pyqtgraph as pg

import matplotlib.pyplot as plt

if __name__ == "__main__":
    os.chdir(os.getcwd() + '\\..')
    from PatchClamp.smartpatcher_backend import SmartPatcher
    from PatchClamp.camerathread import CameraThread
    from PatchClamp.micromanipulator import ScientificaPatchStar
    


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
        self.setWindowTitle("Automatic Patchclamp")
        
        """
        -------------------------- Hardware container -------------------------
        """
        hardwareContainer = QGroupBox()
        hardwareLayout = QGridLayout()
        
        # Button to (dis)connect camera
        self.connect_camera_button = QPushButton(text="Camera", clicked=self.connect_camera)
        self.connect_camera_button.setCheckable(True)
        
        # Button to (dis)connect objective motor
        self.connect_objectivemotor_button = QPushButton(text="Objective motor", clicked=self.mockfunction)
        self.connect_objectivemotor_button.setCheckable(True)
        
        # Button to (dis)connect micromanipulator
        self.connect_micromanipulator_button = QPushButton(text="Micromanipulator", clicked=self.connect_micromanipulator)
        self.connect_micromanipulator_button.setCheckable(True)
        
        # Button to (dis)connect amplifier
        self.connect_amplifier_button = QPushButton(text="Amplifier", clicked=self.mockfunction)
        self.connect_amplifier_button.setCheckable(True)
        
        # Button to (dis)connect pressure controller
        self.connect_pressurecontroller_button = QPushButton(text="Pressure controller", clicked=self.mockfunction)
        self.connect_pressurecontroller_button.setCheckable(True)
        
        # Button to stop all hardware in motion
        self.STOP_button = QPushButton(text="Emergency STOP", clicked=self.STOP)
        self.STOP_button.setCheckable(True)
        
        hardwareLayout.addWidget(self.connect_camera_button, 0, 0, 1, 1)
        hardwareLayout.addWidget(self.connect_objectivemotor_button, 1, 0, 1, 1)
        hardwareLayout.addWidget(self.connect_micromanipulator_button, 2, 0, 1, 1)
        hardwareLayout.addWidget(self.connect_amplifier_button, 3, 0, 1, 1)
        hardwareLayout.addWidget(self.connect_pressurecontroller_button, 4, 0, 1, 1)
        hardwareLayout.addWidget(self.STOP_button, 5, 0, 1, 1)
        hardwareContainer.setLayout(hardwareLayout)
        
        """
        ------------------------- Camera view display -------------------------
        """
        liveContainer = QGroupBox()
        liveContainer.setMinimumSize(1200, 600)
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
        request_clear_drawings = QPushButton(text="Clear phantoms", clicked=self.clearROIs)
        
        # Display to project snaphots on
        snapWidget = pg.ImageView()
        snapWidget.ui.roiBtn.hide()
        snapWidget.ui.menuBtn.hide()
        snapWidget.ui.histogram.hide()
        self.snapImageItem = snapWidget.getImageItem()
        self.snapImageItem.setAutoDownsample(True)
        
        # Button for snapshotting
        self.request_snap_button = QPushButton(text="Take snapshot", clicked=self.request_snap)
        
        liveLayout.addWidget(liveWidget, 0, 0, 1, 2)
        liveLayout.addWidget(snapWidget, 0, 2, 1, 2)
        liveLayout.addWidget(self.request_pause_button, 1, 0, 1, 1)
        liveLayout.addWidget(request_clear_drawings, 1, 1, 1, 1)
        liveLayout.addWidget(self.request_snap_button, 1, 2, 1, 2)
        liveContainer.setLayout(liveLayout)
        
        """
        -------------------------- Sensory output display -------------------------
        """
        sensorContainer = QGroupBox()
        sensorContainer.setMinimumSize(400,600)
        sensorLayout = QGridLayout()
        
        sensorWidget = pg.GraphicsLayoutWidget()
        self.algorithm = sensorWidget.addPlot(1, 0, 1, 1)
        self.current = sensorWidget.addPlot(2, 0, 1, 1)
        self.pressure = sensorWidget.addPlot(3, 0, 1, 1)
        
        sensorLayout.addWidget(sensorWidget)
        sensorContainer.setLayout(sensorLayout)
        
        """
        ---------------------- Algorithm control buttons ----------------------
        """
        algorithmContainer = QGroupBox()
        algorithmLayout = QGridLayout()
        
        request_hardcalibrationxy_button = QPushButton(text="Calibrate XY", clicked=self.request_hardcalibration_xy)
        request_hardcalibrationxyz_button = QPushButton(text="Calibrate XYZ", clicked=self.request_hardcalibration_xyz)
        request_selecttarget_button = QPushButton(text="Select target", clicked=self.request_selecttarget)
        request_confirmtarget_button = QPushButton(text="Confirm target", clicked=self.request_confirmtarget)
        request_detecttip_button = QPushButton(text="Detect tip", clicked=self.request_softcalibration)
        request_autofocustip = QPushButton(text="Autofocus tip", clicked=self.mockfunction)
        request_gigaseal_button = QPushButton(text="Gigaseal", clicked=self.mockfunction)
        request_breakin_button = QPushButton(text="Break-in", clicked=self.mockfunction)
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
        algorithmLayout.addWidget(request_selecttarget_button, 0, 1, 1, 1)
        algorithmLayout.addWidget(request_confirmtarget_button, 1, 1, 1, 1)
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
        
        """
        --------------------------- Adding to master --------------------------
        """
        master = QGridLayout()
        master.addWidget(hardwareContainer, 0, 0, 1, 1)
        master.addWidget(liveContainer, 0, 1, 1, 1)
        master.addWidget(sensorContainer, 0, 2, 1, 1)
        master.addWidget(algorithmContainer, 1, 0, 1, 3)

        self.setLayout(master)
        
        """
        =======================================================================
        ------------------------ Start up roi manager -------------------------
        =======================================================================
        """
        
        self.roimanager = ROIManagerGUI(offset=len(self.liveView.addedItems))
        
        """
        =======================================================================
        -------------- Start up backend and connect signals/slots--------------
        =======================================================================
        """
        
        self.backend = SmartPatcher()
        self.backend.worker.draw.connect(self.draw_roi)
        self.backend.worker.progress.connect(self.mocksignal)
        self.backend.worker.finished.connect(self.mocksignal)
        
        """
        =======================================================================
        ----------------------------- End of GUI ------------------------------
        =======================================================================
        """
        
    def mocksignal(self):
        print('Algorithm finished')
        logging.info('QThread isFinished: ' + str(self.backend.thread.isFinished()))
        logging.info('QThread isRunning: ' + str(self.backend.thread.isRunning()))
        
        
    def mockfunction(self):
        print("Button pushed")
        self.backend.request(name='mockworker')
        
        
    def connect_micromanipulator(self):
        """
        We initiate the micromanipulator by creating the ScientificaPatchStar
        serial object. Creating the object object automatically opens the USB
        port that the device is connected to, similarly, deleting the object
        stops micromanipulator movement and closes the port.
        """
        if self.connect_micromanipulator_button.isChecked():
            micromanipulator = ScientificaPatchStar(address='COM16', baud=38400)
            self.backend.micromanipulator = micromanipulator
        else:
            del self.backend.micromanipulator
        
        
    def connect_camera(self):
        """
        We initiate the camera by creating the CameraThread object, then we
        connect signals with slots to govern inter-thread communication, i.e.
        to recieve images to display. Afterwards we move the camera thread to
        the backend where the thread is immediately started.
        """
        if self.connect_camera_button.isChecked():
            camerathread = CameraThread(camerahandle=None)
            
            self.signal_camera_live = camerathread.livesignal
            self.signal_camera_snap = camerathread.snapsignal
            self.signal_camera_live.connect(self.update_live)
            self.signal_camera_snap.connect(self.update_snap)
            
            self.backend.camerathread = camerathread
        else:
            del self.backend.camerathread
            del self.signal_camera_live
            del self.signal_camera_snap
        
        
    def toggle_pauselive(self):
        if hasattr(self, 'signal_camera_live'):
            if self.request_pause_button.isChecked():
                self.signal_camera_live.disconnect()
            else:
                self.signal_camera_live.connect(self.update_live)
        else:
            self.request_pause_button.setChecked(False)
        
        
    def request_snap(self):
        if self.backend.camerathread != None:
            self.backend.camerathread.snap()
        else:
            I = plt.imread("PatchClamp/testimage.tif")
            self.update_snap(I)
            raise ValueError('no camera connected')
        
    
    def request_hardcalibration_xy(self):
        self.backend.request(name='hardcalibration', mode='XY')
        
    def request_hardcalibration_xyz(self):
        self.backend.request(name='hardcalibration', mode='XYZ')
    
    
    def request_softcalibration(self):
        self.backend.request(name='softcalibration')
    
    
    def request_selecttarget(self):
        """
        The user drags a circular ROI on top of the target cell. The ROI center
        is the target. We first check if a target already exists, if so, we
        recycle it. If the target ROI got removed then we place it back to its
        last known position.
        """
        coords = self.backend.target_coordinates
        if all(values is None for values in coords):
            coords = (0,0)
        else:
            coords = (coords[0]-60,coords[1]-60)
            
        if not self.roimanager.contains('target'):
            target = pg.CircleROI(pos=coords, radius=60, movable=True, pen=QPen(QColor(255,255,0), 0))
            target.setZValue(10)
            self.roimanager.addROI('target')
            self.liveView.addItem(target)
        else:
            idx = self.roimanager.giveROIindex('target')[-1]
            self.liveView.addedItems[idx].translatable = True
            self.liveView.addedItems[idx].setPen(QPen(QColor(255,255,0), 0))
            # self.backend.target_coordinates = np.array([None,None,None])
        
        
    def request_confirmtarget(self):
        """
        If a target is selected, we save the center coordinates of the ROI in
        the camera field of reference.
        """
        if self.roimanager.contains('target'):
            idx = self.roimanager.giveROIindex('target')[-1]
            x,y = self.liveView.addedItems[idx].state['pos'] + self.liveView.addedItems[idx].state['size'] / 2
            self.liveView.addedItems[idx].translatable = False
            self.liveView.addedItems[idx].setPen(QPen(QColor(193,245,240), 0))
            self.backend.target_coordinates = np.array([x,y,np.nan])
        
    
    def draw_roi(self, *args):
        label = args[0][0]
        if label == 'cross':
            xpos,ypos = args[0][1:3]
            vertical = pg.ROI(pos=(xpos,ypos-15), size=[1,30], pen=QPen(QColor(0,255,0), 0))
            horizontal = pg.ROI(pos=(xpos-15,ypos), size=[30,1], pen=QPen(QColor(0,255,0), 0))
            vertical.setZValue(10)
            horizontal.setZValue(10)
            self.roimanager.addROI('cross' + '_vertical')
            self.roimanager.addROI('cross' + '_horizontal')
            self.liveView.addItem(vertical)
            self.liveView.addItem(horizontal)
        elif label == 'line':
            xpos,ypos,orientation = args[0][1:4]
            horizontal = pg.ROI(pos=(xpos-15,ypos), angle=orientation, size=[500,1], pen=QPen(QColor(0,255,0), 0))
            horizontal.setZValue(10)
            self.roimanager.addROI('line')
            self.liveView.addItem(horizontal)
        elif label == 'image':
            img = args[0][1]
            self.update_snap(img)
        else:
            print(label + ' is not a known draw-label')
    
    
    def clearROIs(self):
        self.roimanager.removeallROIs()
        for roi in self.liveView.addedItems[self.roimanager.offset::]:
            self.liveView.removeItem(roi)
    
    
    def update_live(self, image):
        self.liveImageItem.setImage(image)
        
        
    def update_snap(self, image):
        self.snapImageItem.setImage(image)
    
    
    def STOP(self):
        if self.STOP_button.isChecked():
            self.backend.STOP = True
        else:
            self.backend.STOP = False
        
    
    
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
        
        


class ROIManagerGUI:
    def __init__(self, offset):
        self.offset = offset
        self.ROInumber = 0
        self.ROIdictionary = {}
    
    def giveROIindex(self, name):
        return [x+self.offset for x in self.ROIdictionary[name][:]]
    
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
            self.removeROI(name, 'all')
                
    def contains(self, name):
        if name in self.ROIdictionary:
            return True
        else:
            return False





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
