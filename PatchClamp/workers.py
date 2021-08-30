# -*- coding: utf-8 -*-
"""
Created on Wed Aug 11 15:15:30 2021

@author: tvdrb
"""

import numpy as np
from skimage import io
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot

from PatchClamp.ImageProcessing_patchclamp import PatchClampImageProcessing as ia


class Worker(QObject):
    draw = pyqtSignal(list)
    progress = pyqtSignal()
    finished = pyqtSignal()
    
    
    @pyqtSlot()
    def mockfunction(self):
        print(1)
        self.finished.emit()
    
    
    @pyqtSlot()
    def softcalibration(self, camera_handle=None, micromanipulator_handle=None):
        positions = np.array([[-100,-50,0],
                              [0,-50,0],
                              [100,-50,0],
                              [-100,50,0],
                              [0,50,0],
                              [100,50,0]])
        tipcoords = positions[:,0:2] * 0
        reference = micromanipulator_handle.getPos()
        
        for i in range(0, positions.shape[0]):
            # snap images for pipettet tip detection
            micromanipulator_handle.moveAbs(reference+positions[i])
            image_left = camera_handle.snap()
            micromanipulator_handle.moveRel(x=5, y=0, z=0)
            image_right = camera_handle.snap()
            
            # emergency stop
            if self.STOP == True:
                break
            
            # pipette tip detection algorithm
            x1, y1 = ia.detectPipettetip(image_left, image_right, diameter=16, orientation=0)
            W = ia.makeGaussian(size=image_left.shape, mu=(x1,y1), sigma=(image_left.shape[0]//12,image_left.shape[1]//12))
            x, y = ia.detectPipettetip(np.multiply(image_left,W), np.multiply(image_right,W), diameter=20, orientation=0)
            tipcoords[i,:] = x,y
            self.draw.emit(['cross',x,y])
            
            
        # move pipette back to initial coordinates and ask user to correct bias
        micromanipulator_handle.moveAbs(reference)
        
        self.finished.emit()
        
    
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
    
