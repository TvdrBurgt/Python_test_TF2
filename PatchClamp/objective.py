# -*- coding: utf-8 -*-
"""
Created on Wed Dec  8 17:31:40 2021

@author: TvdrBurgt
"""


import os
import time

if __name__ == "__main__":
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname+'/../')

from pipython import GCSDevice
from pipython import pitools 


class PIMotor:
    def __init__(self):
        try:
            # Get the path to dll in the same folder.
            abspath = os.path.abspath(__file__)
            dname = os.path.dirname(abspath) + '/PI_GCS2_DLL_x64.dll'
            print(dname)
            
            self.pidevice = GCSDevice(gcsdll = dname)
            print(self.pidevice.EnumerateUSB())
            # InterfaceSetupDlg() is an interactive dialog. There are other methods to
            # connect to an interface without user interaction.
            serialstring = self.pidevice.EnumerateUSB()
            print(serialstring[0])
            self.pidevice.ConnectUSB(serialnum='PI C-863 Mercury SN 0185500828')
            print('connected: {}'.format(self.pidevice.qIDN().strip()))
            if self.pidevice.HasqVER():
                print('version info: {}'.format(self.pidevice.qVER().strip()))
        except:
            print("PI device not initilized.")
    
    
    def move(self, target_pos):
        targets = [target_pos]
        self.pidevice.MOV(self.pidevice.axes, targets)
        pitools.waitontarget(self.pidevice)
        time.sleep(0.3)
        positions = self.pidevice.qPOS(self.pidevice.axes)
        for axis in self.pidevice.axes:
            print('position of axis {} = {:.5f}'.format(axis, positions[axis]))
    
    
    def GetCurrentPos(self):
        positions = self.pidevice.qPOS(self.pidevice.axes)
        
        return positions['1']
    
    
    def disconnect(self):
        pass
