# -*- coding: utf-8 -*-
"""
Created on Tue Feb 16 16:56:57 2021

@author: tvdrb
"""

import serial

class ScientificaPatchStar:
    """
    This class is for controlling the Scientifica PatchStar micromanipulator.
    """
    def __init__(self, address='COM16', baudrate=38400):
        # Serial settings
        self.port = address
        self.baudrate = baudrate   #Either 9600 or 38400
        
        # Additional settings
        self.CRending = '\r'
        
        # try out connection
        command = 'DESC' + self.CRending
        with serial.Serial(self.port, self.baudrate, timeout=3) as patchstar:
            # Encode the command to ascii and send to PatchStar
            patchstar.write(command.encode('ascii'))
            # Wait until all data is written
            patchstar.flush()
            # Read PatchStar response until carriage return
            response = patchstar.read_until(self.CRending.encode('ascii'))
            # Decodes response to utf-8
            response = response.decode('utf-8')
            # Strip off the carriage return
            response = response.rstrip(self.CRending)
        if response != '':
            print('Scientifica device found: {}'.format(response))
        else:
            print('No Scientifica devices found')
        
    
    def stop(self):
        """
        Stops any motion.
        """
        command = 'STOP' + self.CRending
        
        with serial.Serial(self.port, self.baudrate) as patchstar:
            # Encode the command to ascii and send to PatchStar
            patchstar.write(command.encode('ascii'))
            # Wait until all data is written
            patchstar.flush()
            # Read PatchStar response until carriage return
            response = patchstar.read_until(self.CRending.encode('ascii'))
            # Decodes response to utf-8
            response = response.decode('utf-8')
            # Strip off the carriage return
            response = response.rstrip(self.CRending)
            
        return response
    
    def reportStatus(self):
        """
        Returns the status of the patchstar, responses:
            0 - Motors idle.
            1 - Start speed (point to point moves)
            2 - Accelaration (point to point moves)
            3 - Max speed (point to point moves)
            4 - Deceleration (point to point moves)
            5 - Stopping (point to point moves)
            6 - Input device moves
            7 - Constant velocity moves
        """
        command = 'S' + self.CRending
        
        with serial.Serial(self.port, self.baudrate) as patchstar:
            # Encode the command to ascii and send to PatchStar
            patchstar.write(command.encode('ascii'))
            # Wait until all data is written
            patchstar.flush()
            # Read PatchStar response until carriage return
            response = patchstar.read_until(self.CRending.encode('ascii'))
            # Decodes response to utf-8
            response = response.decode('utf-8')
            # Strip off the carriage return
            response = response.rstrip(self.CRending)
            
        return response
    
    def getPos(self):
        """
        Reports the position for all three axes separated by tabs, example:
            Send: POS or P
            Response: 1321 543 2
        """
        command = 'P' + self.CRending
        
        with serial.Serial(self.port, self.baudrate) as patchstar:
            # Encode the command to ascii and send to PatchStar
            patchstar.write(command.encode('ascii'))
            # Wait until all data is written
            patchstar.flush()
            # Read PatchStar response until carriage return
            response = patchstar.read_until(self.CRending.encode('ascii'))
            # Decodes response to utf-8
            response = response.decode('utf-8')
            # Strip off the carriage return
            response = response.rstrip(self.CRending)
            
            [x, y, z] = response.split('\t')
            
        return [float(x), float(y), float(z)]
        
    def moveAbs(self, x, y, z):
        """
        Moves the patchstar to the given absolute position, example:
            Send: ABS 100 26 3
            Response: A (if move allowed else E)
        """
        command = "ABS %d %d %d" % (x,y,z) + self.CRending
        
        with serial.Serial(self.port, self.baudrate) as patchstar:
            # Encode the command to ascii and send to PatchStar
            patchstar.write(command.encode('ascii'))
            # Wait until all data is written
            patchstar.flush()
            # Read PatchStar response until carriage return
            response = patchstar.read_until(self.CRending.encode('ascii'))
            # Decodes response to utf-8
            response = response.decode('utf-8')
            # Strip off the carriage return
            response = response.rstrip(self.CRending)
            
        return response
    
    def moveAbsZ(self, z):
        """
        Moves the patchstar to the absolute position on the z axis, example:
            Send: ABSZ 128
            Response: A (if move allowed else E)
        """
        command = "ABSZ %d" % z + self.CRending
        
        with serial.Serial(self.port, self.baudrate) as patchstar:
            # Encode the command to ascii and send to PatchStar
            patchstar.write(command.encode('ascii'))
            # Wait until all data is written
            patchstar.flush()
            # Read PatchStar response until carriage return
            response = patchstar.read_until(self.CRending.encode('ascii'))
            # Decodes response to utf-8
            response = response.decode('utf-8')
            # Strip off the carriage return
            response = response.rstrip(self.CRending)
            
        return response
        
    def moveRel(self, x=0, y=0, z=0):
        """
        Moves the patchstar relative to the current position, example:
            Send: REL 100 26 3
            Response: A (if move allowed else E) but it is always allowed, it
            just moves to the edge...
        """
        command = "ABS %d %d %d" % (x,y,z) + self.CRending
        
        with serial.Serial(self.port, self.baudrate) as patchstar:
            # Encode the command to ascii and send to PatchStar
            patchstar.write(command.encode('ascii'))
            # Wait until all data is written
            patchstar.flush()
            # Read PatchStar response until carriage return
            response = patchstar.read_until(self.CRending.encode('ascii'))
            # Decodes response to utf-8
            response = response.decode('utf-8')
            # Strip off the carriage return
            response = response.rstrip(self.CRending)
            
        return response
    
    def setZero(self):
        """
        Sets the current position to (0,0,0) as long as the PatchStar is not
        moving (S command returns 0), example:
            Send: ZERO
            Response: A (if set allowed else E)
        """
        command = 'ZERO' + self.CRending
        
        with serial.Serial(self.port, self.baudrate) as patchstar:
            # Encode the command to ascii and send to PatchStar
            patchstar.write(command.encode('ascii'))
            # Wait until all data is written
            patchstar.flush()
            # Read PatchStar response until carriage return
            response = patchstar.read_until(self.CRending.encode('ascii'))
            # Decodes response to utf-8
            response = response.decode('utf-8')
            # Strip off the carriage return
            response = response.rstrip(self.CRending)
            
        return response

if __name__ == '__main__':
    manipulator = ScientificaPatchStar('COM16')
    # manipulator.getPos()
    # manipulator.echoLine()
    
    