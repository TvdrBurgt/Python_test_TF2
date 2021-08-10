# -*- coding: utf-8 -*-
"""
Created on Tue Aug 10 11:11:54 2021

@author: tvdrb
"""

import os
import ctypes
import logging
import numpy as np

from copy import copy

from PyQt5.QtCore import pyqtSignal, pyqtSlot, QThread, QMutex

if __name__ == "__main__":
    os.chdir(os.getcwd() + '\\..')
    from HamamatsuCam.HamamatsuDCAM import HamamatsuCameraMR, DCAMAPI_INIT


class CameraThread(QThread):
    snapsignal = pyqtSignal(np.ndarray)
    livesignal = pyqtSignal(np.ndarray)

    def __init__(self):
        # Class attributes
        self.exposuretime = 0.01
        self.frame = np.random.rand(2048, 2048)
        # self.initializeCamera()
        
        # QThread attributes
        super().__init__()
        self.isrunning = False
        self.mutex = QMutex()
        self.moveToThread(self)
        self.started.connect(self.acquire)

    def stop(self):
        """
        First we stop the camera thread by setting .isrunning to False, then
        we stop and disconnect the camera. The thread quits but waits for the
        process to finish first.
        """
        self.isrunning = False
        # self.dcam.dcamapi_uninit()
        self.quit()
        self.wait()
        
    def initializeCamera(self):
        # Can we make this path relative?
        dcamapi_path = r"M:\tnw\ist\do\projects\Neurophotonics\Brinkslab\People\Xin Meng\Code\Python_test\HamamatsuCam\19_12\dcamapi.dll"
        self.dcam = ctypes.WinDLL(dcamapi_path)

        paraminit = DCAMAPI_INIT(0, 0, 0, 0, None, None)
        paraminit.size = ctypes.sizeof(paraminit)
        self.dcam.dcamapi_init(ctypes.byref(paraminit))

        n_cameras = paraminit.iDeviceCount
        print("found:", n_cameras, "cameras")

        if n_cameras > 0:
            self.hcam = HamamatsuCameraMR(camera_id=0)
            # Enable defect correction
            self.hcam.setPropertyValue("defect_correct_mode", 2)
            # Set the readout speed to fast
            self.hcam.setPropertyValue("readout_speed", 2)
            # Set the binning
            self.hcam.setPropertyValue("binning", "1x1")
            # Set exposure time
            self.hcam.setPropertyValue("exposure_time", self.exposuretime)
    
    @pyqtSlot()
    def acquire(self):
        # Start acquisition and wait more than exposure time for camera start
        # self.hcam.startAcquisition()
        QThread.msleep(int(self.exposuretime*1000)) # Not sure if necessary with mutex lock
        logging.info("Camera acquisition started")
        
        self.isrunning = True
        while self.isrunning:
            # Mutex lock prevents external access of variables and executes lines in order
            self.mutex.lock()
            
            # Retrieve frames from the camera buffer
            # [frames, dims] = self.hcam.getFrames()
            
            # Resize last frame in the buffer to be displayed
            # self.frame = np.resize(frames[-1].np_array, (dims[1], dims[0]))
            self.frame = np.random.rand(2048, 2048)
            QThread.msleep(int(self.exposuretime*1000))
            
            # Emit a frame every time it is refreshed
            self.livesignal.emit(self.frame)
            
            # Mutex unlock so snap can access last frame
            self.mutex.unlock()
        
        # Stop camera acquisition when exiting the thread
        # self.hcam.stopAcquisition()
        logging.info("Camera acquisition stopped")
        

    def snap(self):
        # Wait exposure time so if micromanipulator was still moving it will
        # output a sharp image.
        QThread.msleep(int(self.exposuretime*1000))
        
        # Mutex lock to wait for camera thread to release a frame
        self.mutex.lock()
        
        # copy released frame
        snapshot = copy(self.frame)
        
        # Mutex unlock so the camera thread can continue again
        self.mutex.unlock()
        
        # Emit frame to display
        self.snapsignal.emit(snapshot)

        return snapshot