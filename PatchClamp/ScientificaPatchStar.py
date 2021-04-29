# -*- coding: utf-8 -*-
"""
Created on Tue Feb 16 16:56:57 2021

@author: tvdrb
"""

import serial

class MicroManipulator:
    """
    This class is for controlling the Scientifica PatchStar micromanipulator.
    """
    def __init__(self, address):
        # Serial settings
        self.port = address
        self.baudrate = 38400   #Either 9600 or 38400
        # Additional settings
        self.CRending = '\r'
        
        # try out connection
        with serial.Serial(self.port, self.baudrate, timeout=5) as patchstar:
            patchstar.write('TYPE'.encode('ascii'))
            response = patchstar.read()
            # Read the remaining characters in the response buffer
            buffersize = patchstar.in_waiting
            response += patchstar.read_until(size=buffersize)
        if response == '':
            print('Scientifica device not found')
        else:
            print('Scientifica device found: {} (5 = PatchStar)'.format(response))
        
    
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
            # Read first character from PatchStar response
            response = patchstar.read()
            # Read the remaining characters in the response buffer
            buffersize = patchstar.in_waiting
            response += patchstar.read_until(size=buffersize)
            # Decodes response to utf-8
            response = response.decode('utf-8')
            
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
            # Read first character from PatchStar response
            response = patchstar.read()
            # Read the remaining characters in the response buffer
            buffersize = patchstar.in_waiting
            response += patchstar.read_until(size=buffersize)
            # Decodes response to utf-8
            response = response.decode('utf-8')
            
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
            # Read first character from PatchStar response
            response = patchstar.read()
            # Read the remaining characters in the response buffer
            buffersize = patchstar.in_waiting
            response += patchstar.read_until(size=buffersize)
            # Decodes response to utf-8
            response = response.decode('utf-8')
            
            [x, y, z] = response.split()
            
        return [float(x), float(y), float(z)]
        
    def moveAbs(self, x=0, y=0, z=0):
        """
        Moves the patchstar to the given absolute position on, example:
            Send: ABS 100 26 3
            Response: A (if move allowed else E)
        """
        command = "ABS %d %d %d" % (x,y,z) + self.CRending
        
        with serial.Serial(self.port, self.baudrate) as patchstar:
            # Encode the command to ascii and send to PatchStar
            patchstar.write(command.encode('ascii'))
            # Wait until all data is written
            patchstar.flush()
            # Read first character from PatchStar response
            response = patchstar.read()
            # Read the remaining characters in the response buffer
            buffersize = patchstar.in_waiting
            response += patchstar.read_until(size=buffersize)
            # Decodes response to utf-8
            response = response.decode('utf-8')
            
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
            # Read first character from PatchStar response
            response = patchstar.read()
            # Read the remaining characters in the response buffer
            buffersize = patchstar.in_waiting
            response += patchstar.read_until(size=buffersize)
            # Decodes response to utf-8
            response = response.decode('utf-8')
            
        return response
        
    def echoLine(self):
        """
        Causes the patchstar to echo the input, example:
            Send: > hello
            Response: HELLO
        """
        command = '> hello' + self.CRending
        
        with serial.Serial(self.port, self.baudrate) as patchstar:
            # Encode the command to ascii and send to PatchStar
            patchstar.write(command.encode('ascii'))
            # Wait until all data is written
            patchstar.flush()
            # Read first character from PatchStar response
            response = patchstar.read()
            # Read the remaining characters in the response buffer
            buffersize = patchstar.in_waiting
            response += patchstar.read_until(size=buffersize)
            # Decodes response to utf-8
            response = response.decode('utf-8')
            
            print('Send: {}'.format(command))
            print('Response: {}'.format(response))
        
    def getApproachAngle(self):
        """
        Return the angle being used to determine the approach axis, example:
            Send: ANGLE
            Response: 24
        """
        command = 'ANGLE' + self.CRending
        
        with serial.Serial(self.port, self.baudrate) as patchstar:
            # Encode the command to ascii and send to PatchStar
            patchstar.write(command.encode('ascii'))
            # Wait until all data is written
            patchstar.flush()
            # Read first character from PatchStar response
            response = patchstar.read()
            # Read the remaining characters in the response buffer
            buffersize = patchstar.in_waiting
            response += patchstar.read_until(size=buffersize)
            # Decodes response to utf-8
            response = response.decode('utf-8')
            
        return response
        
    def setApproachAngle(self, a=0):
        """
        Sets the angle in degrees of the approach axis, example:
            Send: ANGLE 24
            Response: A (if set allowed else E)
        """
        command = 'ANGLE %d' % a + self.CRending
        
        with serial.Serial(self.port, self.baudrate) as patchstar:
            # Encode the command to ascii and send to PatchStar
            patchstar.write(command.encode('ascii'))
            # Wait until all data is written
            patchstar.flush()
            # Read first character from PatchStar response
            response = patchstar.read()
            # Read the remaining characters in the response buffer
            buffersize = patchstar.in_waiting
            response += patchstar.read_until(size=buffersize)
            # Decodes response to utf-8
            response = response.decode('utf-8')
            
        return response

if __name__ == '__main__':
    patchstar = MicroManipulator('COM16')
    # patchstar.getPos()
    # patchstar.moveAbsZ(0)
    # patchstar.echoLine()