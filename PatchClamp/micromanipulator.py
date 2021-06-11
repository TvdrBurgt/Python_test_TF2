# -*- coding: utf-8 -*-
"""
Created on Wed Jun  2 12:58:59 2021

@author: tvdrb
"""

import serial
import numpy as np
import time


class ScientificaPatchStar(serial.Serial):
    """ Serial instance for Scientifica PatchStar control
    This class is for controlling the Scientifica PatchStar micromanipulator.
    The class features a built-in rotation matrix for aligning the coordinate
    system of the PatchStar with the camera field-of-fiew as reference.
    """
    def __init__(self, address='COM16', baud=38400):
        # Set up serial connection
        super().__init__(port=address, baudrate=baud, timeout=1)
        
        # Initiate default settings
        self.CRending = '\r'                    # Carriage return
        self.origin = np.array([0, 0, 0])       # Coordinate origin
        self.manipcoords = np.array([0, 0, 0])  # Manipulator coordinate FOR
        self.camcoords = np.array([0, 0, 0])    # Camera coordinate FOR
        self.R = np.identity(3)                 # 3D rotation matrix
        self.Rinv = np.identity(3)              # 3D rotation matrix inverse
        
        # Fill-in known settings
        self.origin = self.getPos()
        self.manipcoords = self.origin
        self.cameracoords = self.manipcoord
        [self.R, self.Rinv] = self.constructrotationmatrix()
        
    @staticmethod
    def constructrotationmatrix(alpha=0, beta=0, gamma=0):
        """
        This function constructs the 3D rotation matrix that aligns the
        PatchStar coordinate system with that of the camera field-of-view.
        Default angles (=0) return the identitiy matrix as rotation matrix.
        """
        R_alpha = np.array([[1, 0, 0],
                            [0, np.cos(alpha), np.sin(alpha)],
                            [0, -np.sin(alpha), np.cos(alpha)]])
        R_beta = np.array([[np.cos(beta), 0, -np.sin(beta)],
                           [0, 1, 0],
                           [np.sin(beta), 0, np.cos(beta)]])
        R_gamma = np.array([[np.cos(gamma), np.sin(gamma), 0],
                            [-np.sin(gamma), np.cos(gamma), 0],
                            [0, 0, 1]])
        
        # Full rotation matrix and its inverse
        R = R_gamma @ R_beta @ R_alpha
        Rinv = np.transpose(R)
        
        self.R = R
        self.Rinv = Rinv
        
        return R, Rinv
    
    def wait_until_finished(self):
        """
        This function checks if motors are idle and updates the instance
        coordinates afterwards.
        'S' returns the status of the patchstar, '0' means motors are idle
        'P' returns the patchstar coordinates, 'x y z' is the position
        """
        command = 'S' + self.CRending
        
        response = '-1'
        while True:
            try:
                # Encode the command to ascii and send to PatchStar
                self.write(command.encode('ascii'))
                # Wait until all data is written
                self.flush()
                # Read PatchStar response until carriage return
                response = self.read_until(self.CRending.encode('ascii'))
                # Decodes response to utf-8
                response = response.decode('utf-8')
                # Strip off the carriage return
                response = response.rstrip(self.CRending)
            except:
                print('Busy traffic')
                
            if response != '0':
                time.sleep(0.1)
            else:
                break
            
        return response
    
    def getPos(self):
        """
        Reports the position for all three axes separated by tabs, example:
            Send: POS or P
            Response: 1321 543 2
        """
        command = 'P' + self.CRending
        
        # Encode the command to ascii and send to PatchStar
        self.write(command.encode('ascii'))
        # Wait until all data is written
        self.flush()
        # Read PatchStar response until carriage return
        response = self.read_until(self.CRending.encode('ascii'))
        # Decodes response to utf-8
        response = response.decode('utf-8')
        # Strip off the carriage return
        response = response.rstrip(self.CRending)
        
        # Split response by at the tabs
        [x, y, z] = response.split('\t')
        
        # Convert coordinates to float and apply rotation matrix
        [x, y, z] = self.R @ [float(x), float(y), float(z)]
        
        return x, y, z
    
    def moveAbs(self, x, y, z):
        """
        Moves the patchstar to the given absolute position, example:
            Send: ABS 100 26 3
            Response: A (if move allowed else E)
        """
        # Transform camera- to micromanipulator frame of reference
        self.camcoords = [x, y, z]
        self.manipcoords = self.Rinv @ [x, y, z]
        
        # Add coordinate origin
        [x, y, z] = self.origin + self.manipcoords
        
        command = "ABS %d %d %d" % (x,y,z) + self.CRending
        
        # Encode the command to ascii and send to PatchStar
        self.write(command.encode('ascii'))
        # Wait until all data is written
        self.flush()
        # Read PatchStar response until carriage return
        response = self.read_until(self.CRending.encode('ascii'))
        # Decodes response to utf-8
        response = response.decode('utf-8')
        # Strip off the carriage return
        response = response.rstrip(self.CRending)
        
        # Wait untill move has finished
        self.wait_until_finished()
        
        return response
    
    def moveAbsZ(self, z):
        """
        Moves the PatchStar to the given absolute position in z. The x and y
        coordinates do not change.
        """
        # Target location in camera frame of reference
        target = [self.camcoords[0], self.camcoords[1], z]
        
        return self.moveAbs(target)
        
    def moveRel(self, x=0, y=0, z=0):
        """
        Moves the patchstar relative from the initial position. The function
        calls the absolute movement function - to bypass building up
        imperfections from relative movements - and calculates the target
        position by adding the current position.
        """
        # Target location in camera frame of reference
        target = self.camcoords + [x, y, z]
        
        return self.moveAbs(target)
    
    def stop(self):
        """
        Stops any motion.
        """
        command = 'STOP' + self.CRending
        
        # Encode the command to ascii and send to PatchStar
        self.write(command.encode('ascii'))
        # Wait until all data is written
        self.flush()
        # Read PatchStar response until carriage return
        response = self.read_until(self.CRending.encode('ascii'))
        # Decodes response to utf-8
        response = response.decode('utf-8')
        # Strip off the carriage return
        response = response.rstrip(self.CRending)
            
        return response
    
    