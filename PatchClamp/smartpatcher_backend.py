# -*- coding: utf-8 -*-
"""
Created on Sat Aug  7 18:21:18 2021

@author: tvdrb
"""


import os
import datetime
import logging
import numpy as np

from skimage import io

from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot, QThread


class SmartPatcher(QObject):
    
    def __init__(self, manipulator_handle=None, objective_handle=None):
        # Algorithm constants
        self.target_coordinates = np.array([None, None])
        
        # Hardware devices
        self.camerathread = None
        self.amplifierthread = None
        self.pressurethread = None
        self.micromanipulator = manipulator_handle
        self.objectivemotor = objective_handle
    
    @property
    def camerathread(self):
        return self._camerathread
    
    @camerathread.setter
    def camerathread(self, camerathread_handle):
        self._camerathread = camerathread_handle
        if self._camerathread != None:
            self._camerathread.start()
    
    @camerathread.deleter
    def camerathread(self):
        self._camerathread.stop()
        self._camerathread = None
    
    def set_amplifierthread(self, patchamplifier):
        self.amplifierthread = patchamplifier
    
    def set_pressurethread(self, pressurecontroller):
        self.pressurethread = pressurecontroller
        
        
    @property
    def micromanipulator(self):
        return self._micromanipulator
    
    @micromanipulator.setter
    def micromanipulator(self, micromanipulator_handle):
        self._micromanipulator = micromanipulator_handle
    
    @micromanipulator.deleter
    def micromanipulator(self):
        self._micromanipulator.stop()
        self._micromanipulator.close()
        self._micromanipulator = None
        
        
    def set_objectivemotor(self, objective):
        self.objectivemotor = objective
    
    
    @property
    def target_coordinates(self):
        return self._target_coordinates
    
    @target_coordinates.setter
    def target_coordinates(self, coords):
        logging.info('Set target coordinates: [x,y,z]=' + str(coords))
        if type(coords) is np.ndarray:
            if len(coords) == 2:
                self._target_coordinates = coords
            else:
                raise ValueError('length of target coordinates must be 2 or 3')
        else:
            raise ValueError('target coordinates should be a numpy.ndarray')
    
    @target_coordinates.deleter
    def target_coordinates(self):
        del self._target_coordinates
        
    
    