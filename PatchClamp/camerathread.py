# -*- coding: utf-8 -*-
"""
Created on Tue Aug 10 15:05:11 2021

@author: tvdrb
"""


import logging
import numpy as np
from copy import copy
from PyQt5.QtCore import pyqtSignal, pyqtSlot, QThread

from HamamatsuCam.HamamatsuActuator import CamActuator


class CameraThread(QThread):
    snapsignal = pyqtSignal(np.ndarray)
    livesignal = pyqtSignal(np.ndarray)
    
    def __init__(self, camerahandle=None):
        self.exposuretime = 0.02    # seconds
        self.GUIframerate = 25      # frames per second
        self.sleeptime = int(np.max([1/self.GUIframerate, self.exposuretime]))
        self.frame = np.random.rand(2048, 2048)
        
        # Camera attributes
        if camerahandle == None:
            self.camera = CamActuator()
            self.camera.initializeCamera()
        else:
            self.camera = camerahandle
        self.camera.hcam.setPropertyValue("exposure_time", self.exposuretime)
        
        # QThread attributes
        super().__init__()
        self.isrunning = False
        self.moveToThread(self)
        self.started.connect(self.live)
        
    def stop(self):
        self.isrunning = False
        self.quit()
        self.wait()
        
    def snap(self):
        QThread.msleep(self.sleeptime * 1000)
        snap = copy(self.frame)
        self.snapsignal.emit(snap)
        return snap
        
    @pyqtSlot()
    def live(self):
        logging.info('camera thread started')
        
        self.camera.isLiving = True
        self.camera.hcam.acquisition_mode = "run_till_abort"
        
        # Wait a second for camera acquisition to start
        self.camera.hcam.startAcquisition()
        QThread.msleep(1000)
        
        # Emit and get frames from the camera at a rate of 1/sleeptime
        self.isrunning = True
        while self.isrunning:
            QThread.msleep(self.sleeptime  * 1000)
            [frames, dims] = self.camera.hcam.getFrames()
            self.frame = np.resize(frames[-1].np_array, (dims[1], dims[0]))
            self.livesignal.emit(self.frame)
        
        self.camera.hcam.stopAcquisition()
        self.camera.isLiving = False
        self.camera.Exit()
        
        logging.info('camera thread stopped')
    

class CameraThread(QThread):
    snapsignal = pyqtSignal(np.ndarray)
    livesignal = pyqtSignal(np.ndarray)
    
    def __init__(self, camerahandle=None):
        self.exposuretime = 0.02    # seconds
        self.GUIframerate = 25      # frames per second
        self.sleeptime = int(np.max([1/self.GUIframerate, self.exposuretime]))
        self.frame = np.random.rand(2048, 2048)
        
        # QThread attributes
        super().__init__()
        self.isrunning = False
        self.moveToThread(self)
        self.started.connect(self.live)
        
    def stop(self):
        self.isrunning = False
        self.quit()
        self.wait()
        
    def snap(self):
        QThread.msleep(self.sleeptime * 1000)
        snap = copy(self.frame)
        self.snapsignal.emit(snap)
        return snap
        
    @pyqtSlot()
    def live(self):
        logging.info('camera thread started')
        
        self.isrunning = True
        while self.isrunning:
            QThread.msleep(self.sleeptime*1000)
            self.frame = np.random.rand(2048, 2048)
            self.livesignal.emit(self.frame)
            
        logging.info('camera thread stopped')
