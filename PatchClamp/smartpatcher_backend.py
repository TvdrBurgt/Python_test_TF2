# -*- coding: utf-8 -*-
"""
Created on Sat Aug  7 18:21:18 2021

@author: tvdrb
"""


import logging
import numpy as np
from PyQt5.QtCore import QObject, QThread

from PatchClamp.workers import Worker


class SmartPatcher(QObject):
    
    def __init__(self):
        # Hardware constants
        self.pixel_size = 244.8                     # in nanometers
        self.pipette_orientation = 0                # in radians
        self.pipette_diameter = 16                  # in pixels
        
        # Algorithm constants
        self._R = np.eye(3)                         # rotation matrix
        self._operation_mode = 'Default'            # specifies worker mode
        self._pipette_coordinates_pair = np.array(  # [micromanipulator, camera]
            [[None,None,None], [None,None,None]])
        self._target_coordinates = np.array(        # [X, Y, Z] in pixels
            [None,None,None])
        
        # Hardware devices
        self._camerathread = None
        self._amplifierthread = None
        self._pressurethread = None
        self._micromanipulator = None
        self._objectivemotor = None
        
        # Worker thread
        self.worker = Worker(self)
        self.thread = QThread()
        self.worker.moveToThread(self.thread)
        self.worker.finished.connect(self.thread.quit)
        
        # Emergency stop
        self.STOP = False
    
    def request(self, name, mode='default'):
        if self.STOP == True:
            logging.info('Emergency stop active')
        else:
            logging.info('Reqeusted algorithm: ' + name)
            if self.thread.isRunning() == True:
                logging.info('Thread is still running')
            else:
                # disconnect the started method to prevent double execution
                try:
                    self.thread.started.disconnect()
                except:
                    logging.info('Thread is free to use')
                    
                # workers can use operation modes for extra variable input
                self.operation_mode = mode
                
                # update the worker with the latest parameter settings
                self.worker.parent = self
                
                # connected the started method to an executable algorithm
                if name == 'softcalibration':
                    self.thread.started.connect(self.worker.softcalibration)
                elif name == 'hardcalibration':
                    self.thread.started.connect(self.worker.hardcalibration)
                elif name == 'mockworker':
                    self.thread.started.connect(self.worker.mockworker)
                
                # start worker
                self.thread.start()
                
            logging.info('QThread isFinished: ' + str(self.thread.isFinished()))
            logging.info('QThread isRunning: ' + str(self.thread.isRunning()))
    
    @property
    def camerathread(self):
        return self._camerathread
    
    @camerathread.setter
    def camerathread(self, camerathread_handle):
        self._camerathread = camerathread_handle
        self._camerathread.start()
    
    @camerathread.deleter
    def camerathread(self):
        self._camerathread.stop()
        self._camerathread = None
    
    @property
    def amplifierthread(self):
        return self._amplifierthread
    
    @amplifierthread.setter
    def amplifierthread(self, patchamplifier):
        self._amplifierthread = patchamplifier
    
    @amplifierthread.deleter
    def amplifierthread(self):
        self._amplifierthread = None
    
    
    @property
    def pressurethread(self):
        return self._pressurethread
    
    @pressurethread.setter
    def pressurethread(self, pressurecontroller):
        self._pressurethread = pressurecontroller
    
    @pressurethread.deleter
    def pressurethread(self):
        self._pressurethread = None
        
        
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
    
    
    @property
    def objectivemotor(self):
        return self._objectivemotor
    
    @objectivemotor.setter
    def objectivemotor(self, objective):
        self._objectivemotor = objective
    
    @objectivemotor.deleter
    def objectivemotor(self):
        self._objectivemotor = None
    
    
    @property
    def pixel_size(self):
        return self._pixel_size
    
    @pixel_size.setter
    def pixel_size(self, size):
        logging.info('Set pixel size to: ' + str(size))
        if type(size) == float or type(size) == int:
            self._pixel_size = size
        else:
            raise ValueError('pixelsize should be a float or integer')
    
    @pixel_size.deleter
    def pixel_size(self):
        self._pixel_size = None
    
    
    @property
    def pipette_orientation(self):
        return self._pipette_orientation
    
    @pipette_orientation.setter
    def pipette_orientation(self, angle):
        if type(angle) == float or type(angle) == int:
            logging.info('Set pipette orientation: ' + str(angle) + ' degrees')
            self._pipette_orientation = angle
        else:
            raise ValueError('micromanipulator orientation should be a float or integer')
    
    @pipette_orientation.deleter
    def pipette_orientation(self):
        self._pipette_orientation = None
    
    
    @property
    def pipette_diameter(self):
        return self._pipette_diameter
    
    @pipette_diameter.setter
    def pipette_diameter(self, diameter):
        if type(diameter) == float or type(diameter) == int:
            logging.info('Set pipette opening diameter: D = ' + str(diameter))
            self._pipette_diameter = diameter
        else:
            raise ValueError('Pipette opening diameter should be a float or integer')
    
    @pipette_diameter.deleter
    def pipette_diameter(self):
        self._pipette_diameter = None
    
    
    @property
    def R(self):
        return self._R
    
    @R.setter
    def R(self, alpha, beta, gamma):
        if type(alpha) and type(beta) and type(gamma) == float or int:
            logging.info('Set rotation angles alpha beta gamma:', alpha,beta,gamma)
            R_alpha = np.array([[1, 0, 0],
                                [0, np.cos(alpha), np.sin(alpha)],
                                [0, -np.sin(alpha), np.cos(alpha)]])
            R_beta = np.array([[np.cos(beta), 0, -np.sin(beta)],
                               [0, 1, 0],
                               [np.sin(beta), 0, np.cos(beta)]])
            R_gamma = np.array([[np.cos(gamma), np.sin(gamma), 0],
                                [-np.sin(gamma), np.cos(gamma), 0],
                                [0, 0, 1]])
            self._R = R_gamma @ R_beta @ R_alpha
        else:
            raise ValueError('rotation matrix must be a numpy.ndarray')
    
    @R.deleter
    def R(self):
        self._R = np.eye(3)
    
    
    @property
    def operation_mode(self):
        return self._operation_mode
    
    @operation_mode.setter
    def operation_mode(self, mode):
        if type(mode) == str:
            logging.info('Set operation mode for workers to: ' + mode)
            self._operation_mode = mode
        else:
            raise ValueError('operation mode for workers should be a string')
    
    @operation_mode.deleter
    def operation_mode(self):
        self._operation_mode = 'default'
    
    
    @property
    def pipette_coordinates_pair(self):
        return self._pipette_coordinates_pair
    
    @pipette_coordinates_pair.setter
    def pipette_coordinates_pair(self, coords):
        if type(coords) is np.ndarray:
            logging.info('Couple pipette coordinates:\n[X,Y,Z] = ' + str(coords[0,:]) + '\n[row,col,obj] = ' + str(coords[1,:]))
            if coords.shape == (2,3):
                self._pipette_coordinates_pair = coords
            else:
                raise ValueError('coordinates-pair size should be 2x3')
        else:
            raise ValueError('pipette coordinates should be a numpy.ndarray')
    
    @pipette_coordinates_pair.deleter
    def pipette_coordinates_pair(self):
        self._pipette_coordinates_pair = np.array([None, None])
    
    
    @property
    def target_coordinates(self):
        return self._target_coordinates
    
    @target_coordinates.setter
    def target_coordinates(self, coords):
        logging.info('Set target coordinates:\n[row,col,obj] = ' + str(coords))
        if type(coords) is np.ndarray:
            if coords.shape == (3,):
                self._target_coordinates = coords
            else:
                raise ValueError('length of target coordinates must be 3')
        else:
            raise ValueError('target coordinates should be a numpy.ndarray')
    
    @target_coordinates.deleter
    def target_coordinates(self):
        self._pipette_coordinates = np.array([None, None])
    
    
    @property
    def STOP(self):
        return self._STOP
    
    @STOP.setter
    def STOP(self, state):
        """
        # 1. Stop QThreads
        # 2. Stop hardware from moving
        # 3. Set pressure to ATM
        # 4. Set current/voltage to 0
        """
        if state == True:
            self._STOP = True
            logging.info('Emergency stop active')
            self.worker.STOP = True
            if self.micromanipulator != None:
                self._micromanipulator.stop()
                self._micromanipulator.stop()
                self._micromanipulator.stop()
        elif state == False:
            self._STOP = False
            logging.info('Emergency stop standby')
            self.worker.STOP = False
        else:
            raise ValueError('Emergency stop should be either TRUE or False')
    