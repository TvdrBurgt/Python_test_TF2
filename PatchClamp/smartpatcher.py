# -*- coding: utf-8 -*-
"""
Created on Sun Feb  6 11:31:11 2022

@author: tvdrb
"""

import os
import json
import logging
import numpy as np
from PyQt5.QtCore import QObject, QThread

from PatchClamp.workers import Worker


class SmartPatcher(QObject):
    def __init__(self):
        # Hardware devices
        self._camerathread = None
        self._sealtestthread = None
        self._pressurethread = None
        self._micromanipulator = None
        self._objectivemotor = None
        self._XYstage = None
        
        # Default hardware constants
        self._pixel_size = 229.8                    # in nanometers
        self._image_size = [2048, 2048]             # dimension of FOV in pix
        self._pipette_orientation = 0               # in radians
        self._pipette_diameter = 16                 # in pixels (16=patchclamp, ??=cell-picking)
        self._rotation_angles = [0,0,0.043767]      # (alp,bet,gam) in radians
        self._focus_offset = 30                     # in micron above coverslip
        self.update_constants_from_JSON()           # overwrite constants from JSON
        
        # Autopatch variables
        self._state_message = "-"
        self._warning_message = "-"
        self._operation_mode = "Default"
        
        # Data collection
        self.window_size_i = 200
        self.window_size_v = 200
        self.window_size_p = 200
        self.window_size_r = 200
        self.window_size_c = 200
        self.n_i = 0
        self.n_v = 0
        self.n_p = 0
        self.n_r = 0
        self.n_c = 0
        self._current = np.array([])
        self._voltage = np.array([])
        self._pressure = np.array([[],[]])
        self._resistance = np.array([])
        self._capacitance = np.array([])
        
        # Worker thread
        self.worker = Worker(self)
        self.thread = QThread()
        self.worker.moveToThread(self.thread)
        self.worker.finished.connect(self.thread.quit)
    
    
    def update_constants_from_JSON(self):
        # read json file with autopatcher constants and update them in backend
        try:
            filedirectory = os.path.dirname(os.path.abspath('__file__'))
            with open(filedirectory+"/autopatch_configuration.txt", "r") as json_infile:
                data = json.load(json_infile)
                self.pixel_size = data["pixel_size"]
                self.image_size = data["image_size"]
                self.pipette_orientation = data["pipette_orientation"]
                self.pipette_diameter = data["pipette_diameter"]
                self.rotation_angles = data["rotation_angles"]
                self.focus_offset = data["focus_offset"]
        except FileNotFoundError:
            self.write_constants_to_JSON()
    
    def write_constants_to_JSON(self):
        data = {
            "pixel_size": self.pixel_size,
            "image_size": self.image_size,
            "pipette_orientation": self.pipette_orientation,
            "pipette_diameter": self.pipette_diameter,
            "rotation_angles": self.rotation_angles,
            "focus_offset": self.focus_offset
            }
        filedirectory = os.path.dirname(os.path.abspath('__file__'))
        with open(filedirectory+"/autopatch_configuration.txt", "w") as json_outfile:
            json.dump(data, json_outfile)
    
    
    @property
    def current(self):
        return self._current
    @current.setter
    def current(self, current_array):
        self._current = current_array
    @current.deleter
    def current(self):
        self._current = np.array([])
    
    @property
    def voltage(self):
        return self._voltage
    @voltage.setter
    def voltage(self, voltage_array):
        self._voltage = voltage_array
    @voltage.deleter
    def voltage(self):
        self._voltage = np.array([])
    
    @property
    def pressure(self):
        return self._pressure
    @pressure.setter
    def pressure(self, pressure_array):
        self._pressure = pressure_array
    @pressure.deleter
    def pressure(self):
        self._pressure = np.array([[],[]])
    
    @property
    def resistance(self):
        return self._resistance
    @resistance.setter
    def resistance(self, resistance_array):
        self._resistance = resistance_array
    @resistance.deleter
    def resistance(self):
        self._resistance = np.array([])
    
    @property
    def capacitance(self):
        return self._capacitance
    @capacitance.setter
    def capacitance(self, capacitance_array):
        self._capacitance = capacitance_array
    @capacitance.deleter
    def capacitance(self):
        self._capacitance = np.array([])
    
    
    def _current_append_(self, values):
        """Append new values to a sliding window."""
        length = len(values)
        if self.n_i + length > self.window_size_i:
            # Buffer is full so make room.
            copySize = self.window_size_i - length
            self.current = self.current[-copySize:]
            self.n_i = copySize
        self.current = np.append(self.current, values)
        self.n_i += length
    
    def _voltage_append_(self, values):
        """Append new values to a sliding window."""
        length = len(values)
        if self.n_v + length > self.window_size_v:
            # Buffer is full so make room.
            copySize = self.window_size_v - length
            self.voltage = self.voltage[-copySize:]
            self.n_v = copySize
        self.voltage = np.append(self.voltage, values)
        self.n_v += length
    
    def _pressure_append_(self, values, timings):
        """Append new values to a sliding window."""
        length = 1
        if self.n_p + length > self.window_size_p:
            # Buffer is full so make room.
            copySize = self.window_size_p - length
            self.pressure = self.pressure[:,-copySize:]
            self.n_p = copySize
        self.pressure = np.append(self.pressure, np.array([[values],[timings]]), axis=1)
        self.n_p += length
    
    def _resistance_append_(self, Rvalues):
        """Append new values to a sliding window."""
        length = 1
        if self.n_r + length > self.window_size_r:
            # Buffer is full so make room.
            copySize = self.window_size_r - length
            self.resistance = self.resistance[-copySize:]
            self.n_r = copySize
        self.resistance = np.append(self.resistance, Rvalues)
        self.n_r += length
    
    def _capacitance_append_(self, Cvalues):
        """Append new values to a sliding window."""
        length = 1
        if self.n_c + length > self.window_size_c:
            # Buffer is full so make room.
            copySize = self.window_size_c - length
            self.capacitance = self.capacitance[-copySize:]
            self.n_c = copySize
        self.capacitance = np.append(self.capacitance, Cvalues)
        self.n_c += length
    
    
    
    
    
    
    
    
    
    
        
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
    def objectivemotor(self):
        return self._objectivemotor
    @objectivemotor.setter
    def objectivemotor(self, objective_handle):
        self._objectivemotor = objective_handle
    @objectivemotor.deleter
    def objectivemotor(self):
        self._objectivemotor.disconnect()
        self._objectivemotor = None
    
    @property
    def micromanipulator(self):
        return self._micromanipulator
    @micromanipulator.setter
    def micromanipulator(self, micromanipulator_handle):
        
        self._micromanipulator = micromanipulator_handle
    @micromanipulator.deleter
    def micromanipulator(self):
        self._micromanipulator.stop()
        self._micromanipulator = None
    
    @property
    def micromanipulator(self):
        return self._micromanipulator
    @micromanipulator.setter
    def micromanipulator(self, micromanipulator_handle):
        micromanipulator_handle.getPos()
        self._micromanipulator = micromanipulator_handle
    @micromanipulator.deleter
    def micromanipulator(self):
        self._micromanipulator.stop()
        self._micromanipulator = None
    
    @property
    def XYstage(self):
        return self._XYstage
    @XYstage.setter
    def XYstage(self, stage_handle):
        stage_handle.getPos()
        self._XYstage = stage_handle
    @XYstage.deleter
    def XYstage(self):
        self._XYstage = None
    
    @property
    def sealtestthread(self):
        return self._sealtestthread
    @sealtestthread.setter
    def sealtestthread(self, sealtestthread_handle):
        self._sealtestthread = sealtestthread_handle
        self._sealtestthread.setWave(0.1, 0.01, 0)
        self._sealtestthread.start()
    @sealtestthread.deleter
    def sealtestthread(self):
        self._sealtestthread.stop()
        self._sealtestthread = None
    
    @property
    def pressurethread(self):
        return self._pressurethread
    @pressurethread.setter
    def pressurethread(self, pressurecontroller_handle):
        self._pressurethread = pressurecontroller_handle
        self._pressurethread.parent = self
        self._pressurethread.start()
    @pressurethread.deleter
    def pressurethread(self):
        self._pressurethread.stop()
        self._pressurethread = None
    
    
    @property
    def pixel_size(self):
        return self._pixel_size
    @pixel_size.setter
    def pixel_size(self, size):
        if isinstance(size, float) or isinstance(size, int):
            self._pixel_size = size
        else:
            raise ValueError('pixelsize should be a float or integer')
    @pixel_size.deleter
    def pixel_size(self):
        self._pixel_size = None
    
    @property
    def image_size(self):
        return self._image_size
    @image_size.setter
    def image_size(self, size):
        width,height = size
        if type(width) and type(height) == float or int:
            self._image_size = [width, height]
        else:
            raise ValueError('Image size should have width and height of type float or integer')
    @image_size.deleter
    def image_size(self):
        self._image_size = [None, None]
    
    @property
    def rotation_angles(self):
        return self._rotation_angles
    @rotation_angles.setter
    def rotation_angles(self, alphabetagamma):
        if len(alphabetagamma) == 3:
            alpha,beta,gamma = alphabetagamma
            if type(alpha) and type(beta) and type(gamma) == float or int:
                logging.info('Set rotation angles alpha beta gamma: '+str(alpha)+' '+str(beta)+' '+str(gamma))
                self._rotation_angles = [alpha, beta, gamma]
                self.R = (alpha, beta, gamma)
            else:
                raise ValueError('rotation angles should be integers or floats')
        else:
            raise ValueError('rotation angles should be a 3 element array or tuple')
    @rotation_angles.deleter
    def rotation_angles(self):
        self._rotation_angles = [0,0,0]
        del self.R
    
    
    @property
    def R(self):
        return self._R
    @R.setter
    def R(self, alphabetagamma):
        alpha,beta,gamma = alphabetagamma
        R_alpha = np.array([[1, 0, 0],
                            [0, np.cos(alpha), np.sin(alpha)],
                            [0, -np.sin(alpha), np.cos(alpha)]])
        R_beta = np.array([[np.cos(beta), 0, -np.sin(beta)],
                           [0, 1, 0],
                           [np.sin(beta), 0, np.cos(beta)]])
        R_gamma = np.array([[np.cos(gamma), np.sin(gamma), 0],
                            [-np.sin(gamma), np.cos(gamma), 0],
                            [0, 0, 1]])
        try:
            self._R = self._R @ R_gamma @ R_beta @ R_alpha
        except RuntimeError:
            self._R = R_gamma @ R_beta @ R_alpha
    @R.deleter
    def R(self):
        self._R = np.eye(3)
    
    
    
    
    
    
    
    
    
    
    
    @property
    def state_message(self):
        return self._state_message
    @state_message.setter
    def state_message(self, message):
        if isinstance(message, str):
            self._state_message = message
        else:
            raise ValueError('message is not a string')
    @state_message.deleter
    def state_message(self):
        self._state_message = "-"
    
    @property
    def warning_message(self):
        return self._warning_message
    @warning_message.setter
    def warning_message(self, message):
        if isinstance(message, str):
            self._warning_message = message
        else:
            raise ValueError('message is not a string')
    @warning_message.deleter
    def warning_message(self):
        self._warning_message = "-"
    
    @property
    def operation_mode(self):
        return self._operation_mode
    @operation_mode.setter
    def operation_mode(self, mode):
        if isinstance(mode, str):
            self._operation_mode = mode
        else:
            raise ValueError('operation mode for workers should be a string')
    @operation_mode.deleter
    def operation_mode(self):
        self._operation_mode = 'default'
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    def request(self, name, mode='Default'):
        if self.thread.isRunning() == True:
            pass
        else:
            # disconnect the started method to prevent double execution
            try:
                self.thread.started.disconnect()
            except TypeError:
                pass
                
            # workers can use operation modes for extra variable input
            self.operation_mode = mode
            
            # connected the started method to an executable algorithm
            if name == 'target2center':
                if self.XYstage == None or np.array_equal(self.target_coordinates, [None,None,None]):
                    raise ValueError('XY stage not connected')
                else:
                    self.thread.started.connect(self.worker.target2center)
            elif name == 'hardcalibration':
                if self.camerathread == None or self.micromanipulator == None:
                    raise ValueError('Camera and/or micromanipulator not connected')
                else:
                    self.thread.started.connect(self.worker.hardcalibration)
            elif name == 'mockworker':
                    self.thread.started.connect(self.worker.mockworker)
            
            # start worker
            self.thread.start()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    




