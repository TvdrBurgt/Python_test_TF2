# -*- coding: utf-8 -*-
"""
Created on Mon Nov 15 16:29:20 2021

@author: TvdrBurgt
"""


import time
import serial
import logging
import numpy as np
from PyQt5.QtCore import pyqtSignal, pyqtSlot, QThread


class PressureThread(QThread):
    """ Pressure control through serial communication
    This class is for controlling the Pressure Controller.
    """
    measurement = pyqtSignal(np.ndarray)
    
    def __init__(self, address='COM21', baud=9600):
        self.parent = None
        self.waveform = None
        
        # QThread attributes
        super().__init__()
        self.isrunning = False
        self.isrecording = False
        self.moveToThread(self)
        self.started.connect(self.measure)
        
        # Serial attributes
        self.ATM = False        # Only true if 'release pressure' is pressed
        self.port = address     # COM port micromanipulator is connected to
        self.baudrate = baud    # Baudrate of the micromanipulator
        self.ENDOFLINE = '\n'   # Carriage return
        self.controller = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=1)
    
    @property
    def parent(self):
        return self._parent
    
    @parent.setter
    def parent(self, parent):
        self._parent = parent
    
    @property
    def waveform(self):
        return self._waveform
    
    @waveform.setter
    def waveform(self, function_handle):
        self._waveform = function_handle
    
    @waveform.deleter
    def waveform(self):
        self._waveform = None
        
    
    def stop(self):
        self.isrecording = False
        self.isrunning = False
        self.quit()
        self.wait()
    
    def set_pressure(self, pressure):
        """
        Sets the pressure immediately, and does not depend on the measurement
        thread.
        """
        
        command = "P %d" % pressure + self.ENDOFLINE
        
        # Encode the command to ascii and send to the device
        self.controller.write(command.encode('ascii'))
    
    def release_pressure(self):
        """
        Calling this function once, releases sets the pressure to ATM
        immediately and stops writing the waveform. Calling this function a
        second time releases the block on waveform writing.
        """
        self.ATM = not self.ATM
        if self.ATM:
            self.set_pressure(0)
    
    def set_waveform(self, high, low, high_T, low_T):
        
        P = lambda t: high*(np.heaviside(t%(high_T+low_T),1) - np.heaviside(t%(high_T+low_T)-high_T,1)) + \
            low*(np.heaviside(t%(high_T+low_T)-high_T,1) - np.heaviside(t%(high_T+low_T)-high_T-low_T,1))
        
        self.waveform = P
        
        
    @pyqtSlot()
    def measure(self):
        logging.info('pressure thread started')
        
        self.isrunning = True
        start = time.time()
        old_pressure = 0
        while self.isrunning:
            
            # Read pressure controller, flush input, and emit pressure measurements
            if self.controller.inWaiting():
                response = self.controller.read_until(self.ENDOFLINE.encode('ascii'))
                while self.controller.inWaiting():
                    response = self.controller.read_until(self.ENDOFLINE.encode('ascii'))
                try:
                    response = response.decode('utf-8')
                except:
                    response = ""
                response = response.split()
                if len(response) > 0:
                    if response[0] == "PS":
                        try:
                            PS1 = float(response[1])
                            PS2 = float(response[2])
                            self.measurement.emit(np.array([PS1, PS2]))
                        except ValueError:
                            pass
            
            # Write waveform if active
            if self.waveform is not None:
                new_pressure = self.waveform(time.time()-start)
                if new_pressure != old_pressure and not self.ATM:
                    self.set_pressure(new_pressure)
                    old_pressure = new_pressure
            
            # Enter the record function
            if not self.isrecording:
                QThread.msleep(10)
            else:
                self.record()
        
        # Set pressure back to ATM and close the serial port
        self.set_pressure(0)
        self.controller.close()
            
        logging.info('pressure thread stopped')
    
    
    def record(self):
        logging.info("pressure recording started")
        
        save_directory = self._parent.save_directory
        
        PS1 = []
        PS2 = []
        timing = []
        start = time.time()
        while self.isrecording:
            
            # Read pressure controller, flush input, and emit pressure measurements
            response = self.controller.read_until(self.ENDOFLINE.encode('ascii'))
            try:
                response = response.decode('utf-8')
                T = time.time() - start
            except:
                response = ""
            response = response.split()
            if len(response) > 0:
                if response[0] == "PS":
                    PS1.append(float(response[1]))
                    PS2.append(float(response[2]))
                    timing.append(T)
                    self.measurement.emit(np.array([PS1[-1], PS2[-1]]))
            
            # Determines the sampling rate
            QThread.msleep(5)
        
        # Save measurements and close the serial port
        np.save(save_directory+'pressure_recording_sensor1', PS1)
        np.save(save_directory+'pressure_recording_sensor2', PS2)
        np.save(save_directory+'pressure_recording_timing', timing)
            
        logging.info('pressure recording stopped')



# class PressureThread(QThread):
#     """ Pressure control through serial communication
#     This class is for controlling the Pressure Controller.
#     """
#     measurement = pyqtSignal(np.ndarray)
    
#     def __init__(self, address='COM4', baud=9600):
#         self.parent = None
#         self.waveform = None
        
#         self.port = address     # COM port micromanipulator is connected to
#         self.baudrate = baud    # Baudrate of the micromanipulator
#         self.ENDOFLINE = '\n'   # Carriage return
        
#         self.pressure_offset = 0
#         self.ATM = False
        
#         # QThread attributes
#         super().__init__()
#         self.isrunning = False
#         self.isrecording = False
#         self.moveToThread(self)
#         self.started.connect(self.measure)
    
#     @property
#     def parent(self):
#         return self._parent
    
#     @parent.setter
#     def parent(self, parent):
#         self._parent = parent
    
#     @property
#     def waveform(self):
#         return self._waveform
    
#     @waveform.setter
#     def waveform(self, function_handle):
#         self._waveform = function_handle
    
#     @waveform.deleter
#     def waveform(self):
#         self._waveform = None
    
#     def stop(self):
#         self.isrecording = False
#         self.isrunning = False
#         self.quit()
#         self.wait()
    
#     def set_pressure(self, target_pressure):
#         self.pressure_offset = target_pressure
    
#     def release_pressure(self):
#         self.ATM = not self.ATM
#         if self.ATM:
#             self.set_pressure(0)
    
#     def set_waveform(self, high, low, high_T, low_T):
        
#         P = lambda t: high*(np.heaviside(t%(high_T+low_T),1) - np.heaviside(t%(high_T+low_T)-high_T,1)) + \
#             low*(np.heaviside(t%(high_T+low_T)-high_T,1) - np.heaviside(t%(high_T+low_T)-high_T-low_T,1))
        
#         self.waveform = P
    
#     @pyqtSlot()
#     def measure(self):
#         logging.info('pressure thread started')
        
#         self.isrunning = True
#         start = time.time()
#         old_pressure = 0
#         while self.isrunning:
#             output = self.pressure_offset + np.random.rand(2)*10-5
#             self.measurement.emit(np.array([output[0], output[1]]))
            
#             # Write waveform if active
#             if self.waveform is not None:
#                 new_pressure = self.waveform(time.time()-start, -100, -200, 1, 1)
#                 if new_pressure != old_pressure and not self.ATM:
#                     self.set_pressure(new_pressure)
#                     old_pressure = new_pressure
            
#             # Enter the record function
#             if not self.isrecording:
#                 QThread.msleep(10)
#             else:
#                 self.record()
        
#         logging.info('pressure thread stopped')

#     def record(self):
#         logging.info("pressure recording started")
        
#         PS1 = []
#         PS2 = []
#         timing = []
#         start = time.time()
#         while self.isrecording:
#             output = self.pressure_offset + np.random.rand(2)*10-5
#             T = time.time() - start
            
#             PS1.append(output[0])
#             PS2.append(output[1])
#             timing.append(T)
#             self.measurement.emit(np.array([PS1[-1], PS2[-1]]))
            
#             # Determines the sampling rate
#             QThread.msleep(5)
            
#         logging.info('pressure recording stopped')
