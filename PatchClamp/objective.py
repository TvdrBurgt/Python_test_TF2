# -*- coding: utf-8 -*-
"""
Created on Thu Dec  9 10:18:47 2021

@author: TvdrBurgt
"""


import sys
import time

sys.path.append('../')
from pipython import GCSDevice, pitools


class PIMotor:
    """ Physik Intrumente Objective motor control
    The objective is attached to a motor that can move it up and down. This
    file is a minimalistic version of xinmeng's 'focuser.py' but contains the
    exact same functionality.
    """
    
    def __init__(self, objective_motor_handle=None):
        # Connect the objective motor if it is not given
        if objective_motor_handle == None:
            self.objective = GCSDevice(gcsdll=__file__+'/../../'+'/PI_ObjectiveMotor/PI_GCS2_DLL_x64.dll')
            # serialstring = self.objective.EnumerateUSB()
            self.objective.ConnectUSB(serialnum='PI C-863 Mercury SN 0185500828')
        else:
            self.objective = objective_motor_handle
    
    
    def disconnect(self):
        """
        Disconnects the objective motor, initialization necessary to use it
        again.
        """
        self.objective.CloseConnection()
    
    
    def move(self, target):
        """
        Moves the objective motor to a target position, 'target' is a position
        on the z-axis with units 10 micron. Example:
            target = 3.45 moves the objective 34.5 micrometers above zero.
        """
        self.objective.MOV(self.objective.axes, target)
        pitools.waitontarget(self.objective)
        
        # below this line is necessary?
        time.sleep(0.3)
        positions = self.objective.qPOS(self.objective.axes)
        for axis in self.objective.axes:
            print('position of axis {} = {:.5f}'.format(axis, positions[axis]))
    
    
    def getPos(self):
        """
        Reports the position of the objective motor in units of 10 micron.
        Example: 
            position = 3.45 means the motor position is 34.5 microns from 0.
        """
        # positions is a dictionary with key being axis name, here '1'.
        positions = self.objective.qPOS(self.objective.axes)
        
        return positions['1']
    





if __name__ == '__main__':
    objective = PIMotor()