# -*- coding: utf-8 -*-
"""
Created on Tue Aug 10 15:05:11 2021

@author: tvdrb
"""


import os
import logging
import numpy as np

from copy import copy

from PyQt5.QtCore import pyqtSignal, pyqtSlot, QThread

if __name__ == "__main__":
    os.chdir(os.getcwd() + '\\..')
    from HamamatsuCam.HamamatsuActuator import CamActuator


# class CameraThread(QThread):
#     snapsignal = pyqtSignal(np.ndarray)
#     livesignal = pyqtSignal(np.ndarray)
    
#     def __init__(self, camerahandle=None):
#         self.exposuretime = 0.02
#         self.GUIframerate = 25 #frames per second
#         if camerahandle == None:
#             self.camera = CamActuator()
#             self.camera.initializeCamera()
#         else:
#             self.camera = camerahandle
#         self.camera.hcam.setPropertyValue("exposure_time", self.exposuretime)
        
#         # QThread attributes
#         super().__init__()
#         self.isrunning = False
#         self.moveToThread(self)
#         self.started.connect(self.live)
        
#     def stop(self):
#         self.isrunning = False
#         self.camera.Exit()
#         self.quit()
#         self.wait()
        
#     def snap(self):
#         snap = copy(self.camera.Live_image)
#         self.snapsignal.emit(snap)
#         return snap
        
#     @pyqtSlot()
#     def live(self):
#         self.isrunning = True
#         self.camera.LIVE()
#         while self.isrunning:
#             QThread.msleep(int(1/self.GUIframerate * 1000))
#             self.livesignal.emit(self.camera.Live_image)
#         self.camera.StopLIVE()
    

class CameraThread(QThread):
    snapsignal = pyqtSignal(np.ndarray)
    livesignal = pyqtSignal(np.ndarray)
    
    def __init__(self):
        self.exposuretime = 0.02
        self.GUIframerate = 25 #frames per second
        self.Live_image = np.random.rand(2048, 2048)
        
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
        snap = copy(self.Live_image)
        self.snapsignal.emit(snap)
        return snap
        
    @pyqtSlot()
    def live(self):
        logging.info('camera thread started')
        
        self.isrunning = True
        while self.isrunning:
            QThread.msleep(int(self.exposuretime * 1000))
            self.Live_image = np.random.rand(2048, 2048)
            QThread.msleep(int(1/self.GUIframerate * 1000))
            self.livesignal.emit(self.Live_image)
            
        logging.info('camera thread stopped')
