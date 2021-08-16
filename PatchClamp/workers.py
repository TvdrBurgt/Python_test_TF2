# -*- coding: utf-8 -*-
"""
Created on Wed Aug 11 15:15:30 2021

@author: tvdrb
"""

import numpy as np
from skimage import io
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot


class Worker(QObject):
    draw = pyqtSignal(str, np.ndarray)
    progress = pyqtSignal()
    finished = pyqtSignal()
    
    @pyqtSlot()
    def mockfunction(self):
        print(1)
    
    @pyqtSlot()
    def detect_tip(self, camera_handle, micromanipulator_handle):
        print('Detect tip')
    
    @pyqtSlot()
    def autofocus_tip(self, camera_handle, micromanipulator_handle):
        print('autofocus tip')
    
    # @pyqtSlot()
    # def request_imagexygrid(self):
    #     savedirectory = r'M:\tnw\ist\do\projects\Neurophotonics\Brinkslab\Data\Thijs\Save directory\\'
    #     x,y,z = self.backend._micromanipulator.getPos()
    #     stepsize = 50
    #     for i in range(0, 11):
    #         for j in range(0, 11):
    #             self.backend._micromanipulator.moveAbs(x+i*stepsize, y+j*stepsize,z)
    #             for k in ['a','b']:
    #                 if k == 'b':
    #                     self.backend._micromanipulator.moveRel(dx=5, dy=0, dz=0)
    #                 snap = self.backend.camerathread.snap()
    #                 io.imsave(savedirectory+'X%dY%d'%(i*stepsize,j*stepsize)+k+'.tif', snap, check_contrast=False)
    
    