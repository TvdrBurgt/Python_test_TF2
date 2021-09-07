# -*- coding: utf-8 -*-
"""
Created on Wed Aug 11 15:15:30 2021

@author: tvdrb
"""

import logging
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot

from PatchClamp.ImageProcessing_patchclamp import PatchClampImageProcessing as ia


class Worker(QThread):
    draw = pyqtSignal(list)
    progress = pyqtSignal()
    finished = pyqtSignal()
    
    def __init__(self, parent):
        self.parent = parent
        
        # QThread attributes
        super().__init__()
        self.moveToThread(self)
        self.finished.connect(self.quit)
    
    @pyqtSlot()
    def mockworker(self):
        print('printed in thread')
        self.draw.emit(['cross',1000,1000])
        self.draw.emit(['line',500,500,-(10)])
        self.draw.emit(['line',500,500,-(10-90)])
        self.finished.emit()
    
    
    @pyqtSlot()
    def hardcalibration(self):
        """ Hardcalibration aligns the coordinate system of the micromanipulator
        with that of the camera field-of-view (FOV) by constructing a rotation
        matrix that maps coordinates between the coordinate systems. It can
        also estimate the pixelsize.
        
        We can choose three modes of operation:
            XY          for aligning only the x- and y-axes,
            XYZ         for aligning the x-, y- and z-axes,
            pixelsize   for estimating the pixel size.
        We can set the variable:
            stepsize    should be set small enough to keep the pipette visible
        
        The rotation angles are calculated as passive rotation angles (counter-
        clockwise + acting on coordinate systems) in the order: gamma, alpha, 
        beta.
        The matrix E contains the micromanipulator axes in unit vectors as:
        E = [Exx Exy Exz
             Eyx Eyy Eyz
             Ezx Ezy Ezz],
        if perfectly aligned with the FOV, E should be the 3x3 identity matrix.
        
        outputs:
            gamma       (rotation angle of the micromanipulator z-axis w.r.t.
                         camera z-axis)
            alpha       (rotation angle of the micromanipulator x-axis w.r.t. 
                         camera x-axis)
            beta        (rotation angle of the micromanipulator y-axis w.r.t.
                         camera y-axis)
            pixelsize   (pixel size in nanometers)
        """
        micromanipulator = self.parent.micromanipulator_handle
        camera = self.parent.camera_handle
        mode = self.parent.operation_mode
        D = self.parent.diameter
        O = self.parent.orientation
        
        # algorithm variables
        stepsize = 10   # in microns
        
        # modes of operation
        if mode == 'XY':
            dimension = 2
        elif mode == 'XYZ':
            dimension = 3
        elif mode == 'pixelsize':
            dimension = 2
        else:
            logging.warning('Hardcalibration mode not recognized')
        
        positions = np.linspace(-3*stepsize, 3*stepsize, num=7)
        directions = np.eye(3)
        tipcoords = np.tile(np.nan, (3,len(positions),3))
        reference = micromanipulator.getPos()
        
        # move hardware to retrieve tip coordinates
        for i in range(dimension):
            for j in positions:
                # snap images for pipettet tip detection
                micromanipulator.moveAbs(reference+directions[i]*positions[j])
                image_left = camera.snap()
                micromanipulator.moveRel(dx=5)
                image_right = camera.snap()
                
                # pipette tip detection algorithm
                x1, y1 = ia.detectPipettetip(image_left, image_right, diameter=D, orientation=O)
                W = ia.makeGaussian(size=image_left.shape, mu=(x1,y1), sigma=(image_left.shape[0]//12,image_left.shape[1]//12))
                x, y = ia.detectPipettetip(np.multiply(image_left,W), np.multiply(image_right,W), diameter=(5/4)*D, orientation=O)
                
                # save tip coordinates
                tipcoords[i,j,:] = np.array([x,y,np.nan])
                self.draw.emit(['cross',x,y])
        
        # move hardware back to start position
        micromanipulator.moveAbs(reference)
        
        # reduce tip coordinates to a unit step mean deviation
        E = np.mean(tipcoords,1)/stepsize
        
        # calculate rotation angles
        if E[0,0] > 0:
            gamma = np.arctan(-E[0,1]/E[0,0])
        else:
            gamma = np.arctan(-E[0,1]/E[0,0]) - np.pi
        alpha = np.arcsin(E[2,0]*np.sin(gamma) + E[2,1]*np.cos(gamma))
        beta = np.arcsin((-E[2,0]*np.cos(gamma) + E[2,1]*np.sin(gamma))/np.cos(alpha))
        
        # set rotation angles or calculate pixelsize
        if mode == 'XY':
            self.parent.R(alpha=0, beta=0, gamma=gamma)
            self.draw.emit(['cross',tipcoords[0,3,0],tipcoords[0,3,1],-np.rad2deg(gamma)])
            self.draw.emit(['cross',tipcoords[0,3,0],tipcoords[0,3,1],-np.rad2deg(gamma-np.pi/2)])
        elif mode == 'XYZ':
            self.parent.R(alpha=alpha, beta=beta, gamma=gamma)
            self.draw.emit(['cross',tipcoords[0,3,0],tipcoords[0,3,1],-np.rad2deg(gamma)])
            self.draw.emit(['cross',tipcoords[0,3,0],tipcoords[0,3,1],-np.rad2deg(gamma-np.pi/2)])
        elif mode == 'pixelsize':
            # get all micromanipulator-pixel pairs in the XY plane
            pixcoords = tipcoords[0:2,:,0:2]
            realcoords = np.tile(np.nan, (2,len(positions),2))
            for i in range(dimension):
                for j in positions:
                    realcoords[i,j] = reference+directions[i]*positions[j]
            
            # couple micrometer distance with pixeldistance and take the mean
            diff_realcoords = np.abs(np.diff(realcoords, axis=1))   # in microns
            diff_pixcoords = np.abs(np.diff(pixcoords, axis=1))     # in pixels
            sample = diff_realcoords/diff_pixcoords     # microns per pixel
            sample_mean = np.mean(sample)
            sample_var = np.var(sample)
            
            # real = np.random.rand(2,7,2)+1555
            # pix = np.random.rand(2,7,2)*10
            # diff_real = np.abs(np.diff(real, axis=1))
            # diff_pix = np.abs(np.diff(pix, axis=1))
            # smp = diff_real/diff_pix
            # smp_mean = np.mean(smp)
            # smp_var = np.var(smp)
            # print('mean pixel size = ' + str(smp_mean))
            # print('variance pixel size = ' + str(smp_var))
            
            # # set pixelsize to backend
            # self.parent.pixelsize = sample_mean
            # logging.info('pixelsize estimation precision: s.d. = ' + str(np.sqrt(sample_var)))
        
        self.finished.emit()
    
    
    @pyqtSlot()
    def softcalibration(self):
        micromanipulator = self.parent.micromanipulator_handle
        camera = self.parent.camera_handle
        D = self.parent.diameter
        O = self.parent.orientation
        
        positions = np.array([[-100,-50,0],
                              [0,-50,0],
                              [100,-50,0],
                              [-100,50,0],
                              [0,50,0],
                              [100,50,0]])
        tipcoords = positions[:,0:2] * 0
        reference = micromanipulator.getPos()
        
        for i in range(0, positions.shape[0]):
            # snap images for pipettet tip detection
            micromanipulator.moveAbs(reference+positions[i])
            image_left = camera.snap()
            micromanipulator.moveRel(dx=5)
            image_right = camera.snap()
            
            # emergency stop
            if self.parent.STOP == True:
                break
            
            # pipette tip detection algorithm
            x1, y1 = ia.detectPipettetip(image_left, image_right, diameter=D, orientation=O)
            W = ia.makeGaussian(size=image_left.shape, mu=(x1,y1), sigma=(image_left.shape[0]//12,image_left.shape[1]//12))
            x, y = ia.detectPipettetip(np.multiply(image_left,W), np.multiply(image_right,W), diameter=(5/4)*D, orientation=O)
            
            # save tip coordinates in an array
            tipcoords[i,:] = x,y
            self.draw.emit(['cross',x,y])
        
        # user bias correction
        tipcoord = np.mean(tipcoords, axis=0)
        micromanipulator.moveAbs(reference)
        userbias = np.array([0, 0])     #should come from human input!
        tipcoord += userbias
        self.draw.emit(['cross',tipcoord[0],tipcoord[1]])
        
        # set micromanipulator and camera coordinate pair of pipette tip
        self.pipette_coordinates_pair = np.vstack([reference, np.array([tipcoord[0], tipcoord[1], np.nan])])
        
        self.finished.emit()
        
    
    @pyqtSlot()
    def autofocus_tip(self, camera_handle=None, micromanipulator_handle=None):
        pass
        
        
    
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
    
