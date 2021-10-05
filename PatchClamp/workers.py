# -*- coding: utf-8 -*-
"""
Created on Wed Aug 11 15:15:30 2021

@author: TvdrBurgt
"""


import logging
import numpy as np
from skimage import io
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot

from PatchClamp.ImageProcessing_patchclamp import PatchClampImageProcessing as ia


class Worker(QObject):
    draw = pyqtSignal(list)
    sharpnessfunction = pyqtSignal(np.ndarray)
    progress = pyqtSignal()
    finished = pyqtSignal()
    
    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        self.STOP = False
        
    @property
    def parent(self):
        return self._parent
    
    @parent.setter
    def parent(self, parent):
        self._parent = parent
    
    
    @pyqtSlot()
    def mockworker(self):
        print('printed in thread')
        self.draw.emit(['cross',1000,1000])
        self.draw.emit(['calibrationline',500,500,-(10)])
        self.draw.emit(['calibrationline',500,500,-(10-90)])
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
        # get all relevant parent attributes
        save_directory = self._parent.save_directory
        micromanipulator = self._parent.micromanipulator
        camera = self._parent.camerathread
        account4rotation = self._parent.account4rotation
        mode = self._parent.operation_mode
        D = self._parent.pipette_diameter
        O = self._parent.pipette_orientation
        
        # algorithm variables
        stepsize = 25   # in microns
        
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
            for j, pos in enumerate(positions):
                # snap images for pipettet tip detection
                x,y,z = account4rotation(origin=reference, target=reference+directions[i]*pos)
                micromanipulator.moveAbs(x,y,z)
                image_left = camera.snap()
                micromanipulator.moveRel(dx=5)
                image_right = camera.snap()
                
                # pipette tip detection algorithm
                x1, y1 = ia.detectPipettetip(image_left, image_right, diameter=D, orientation=O)
#                self.draw.emit(['cross',x1,y1])
                W = ia.makeGaussian(size=image_left.shape, mu=(x1,y1), sigma=(image_left.shape[0]//12,image_left.shape[1]//12))
                camera.snapsignal.emit(np.multiply(image_right,W))
                x, y = ia.detectPipettetip(np.multiply(image_left,W), np.multiply(image_right,W), diameter=(5/4)*D, orientation=O)
                
                # save tip coordinates
                tipcoords[i,j,:] = np.array([x,y,np.nan])
                self.draw.emit(['cross',x,y])
                np.save(save_directory+'hardcalibration'+mode, tipcoords)  #FLAG: relevant for MSc thesis
        
        # move hardware back to start position
        x,y,z = reference
        micromanipulator.moveAbs(x,y,z)
        
        # reduce tip coordinates to a unit step mean deviation
        tipcoords_diff = np.diff(tipcoords,axis=1)
        E = np.mean(tipcoords_diff,axis=1)/stepsize
        
        # calculate rotation angles
        if E[0,0] > 0:
            gamma = np.arctan(-E[0,1]/E[0,0])
        else:
            gamma = np.arctan(-E[0,1]/E[0,0]) - np.pi
        alpha = np.arcsin(E[2,0]*np.sin(gamma) + E[2,1]*np.cos(gamma))
        beta = np.arcsin((-E[2,0]*np.cos(gamma) + E[2,1]*np.sin(gamma))/np.cos(alpha))
        
        # set rotation angles or calculate pixelsize
        if mode == 'XY':
            self._parent.rotation_angles = (0, 0, -gamma)
            self.draw.emit(['calibrationline',tipcoords[0,3,0],tipcoords[0,3,1],-np.rad2deg(gamma)])
            self.draw.emit(['calibrationline',tipcoords[0,3,0],tipcoords[0,3,1],-np.rad2deg(gamma-np.pi/2)])
        elif mode == 'XYZ':
            self._parent.rotation_angles = (-alpha, -beta, -gamma)
            self.draw.emit(['calibrationline',tipcoords[0,3,0],tipcoords[0,3,1],-np.rad2deg(gamma)])
            self.draw.emit(['calibrationline',tipcoords[0,3,0],tipcoords[0,3,1],-np.rad2deg(gamma-np.pi/2)])
        elif mode == 'pixelsize':
            # get all micromanipulator-pixel pairs in the XY plane
            pixcoords = tipcoords[0:2,:,0:2]
            realcoords = np.tile(np.nan, (2,len(positions),2))
            for i in range(dimension):
                for j, pos in enumerate(positions):
                    x,y,z = account4rotation(origin=reference, target=reference+directions[i]*pos)
                    realcoords[i,j] = np.array([x,y])
            
            # couple micrometer distance with pixeldistance and take the mean
            diff_realcoords = np.abs(np.diff(realcoords, axis=1))   # in microns
            diff_pixcoords = np.abs(np.diff(pixcoords, axis=1))     # in pixels
            samples = diff_realcoords/diff_pixcoords     # microns per pixel
            sample = np.array([samples[0,:,0], samples[1,:,1]])
            sample_mean = np.mean(sample)
            sample_var = np.var(sample)
            
            # set pixelsize to backend
            self._parent.pixelsize = sample_mean
            logging.info('pixelsize estimation: mean +/- s.d. = ' + str(sample_mean) + ' +/- ' + str(np.sqrt(sample_var)))
            
            # # simulate pixel size estimation
            # real = np.array([np.transpose([np.linspace(0,100,7),np.linspace(0,100,7)]),np.transpose([np.linspace(0,100,7),-np.linspace(0,100,7)])])
            # pix = np.array([np.transpose([np.linspace(750,1250,7),np.linspace(750,1250,7)]),np.transpose([np.linspace(750,1250,7),-np.linspace(750,1250,7)])])
            # real += np.random.rand(2,7,2)+1555
            # pix += np.random.rand(2,7,2)*10
            # diff_real = np.abs(np.diff(real, axis=1))
            # diff_pix = np.abs(np.diff(pix, axis=1))
            # smp = diff_real/diff_pix
            # smp_mean = np.mean(smp)
            # smp_var = np.var(smp)
            # print('mean pixel size = ' + str(smp_mean))
            # print('variance pixel size = ' + str(smp_var))
            # print('true pixel size = ' + str((101/7)/(501/7)))
        
        self.finished.emit()
    
    
    @pyqtSlot()
    def softcalibration(self):
        """ Softcalibration couples the micromanipulator coordinates with the
        pixelcoordinates of a pipette tip.
        
        The user can select to correct for bias.
        
        The positions matrix contains the calibration points. We can change,
        add, or remove calibration points if necessary.
        
        outputs:
            pipette_coordinates_pair    np.array([reference (in microns);
                                                  tip coordinates (in pixels)])
        """
        # get all relevant parent attributes
        save_directory = self._parent.save_directory
        micromanipulator = self._parent.micromanipulator
        camera = self._parent.camerathread
        account4rotation = self._parent.account4rotation
        D = self._parent.pipette_diameter
        O = self._parent.pipette_orientation
        
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
            x,y,z = account4rotation(origin=reference, target=reference+positions[i])
            micromanipulator.moveAbs(x,y,z)
            image_left = camera.snap()
            micromanipulator.moveRel(dx=5)
            image_right = camera.snap()
            
            # emergency stop
            if self.STOP == True:
                break
            
            # pipette tip detection algorithm
            x1, y1 = ia.detectPipettetip(image_left, image_right, diameter=D, orientation=O)
#            self.draw.emit(['cross',x1,y1])
            W = ia.makeGaussian(size=image_left.shape, mu=(x1,y1), sigma=(image_left.shape[0]//12,image_left.shape[1]//12))
            camera.snapsignal.emit(np.multiply(image_right,W))
            x, y = ia.detectPipettetip(np.multiply(image_left,W), np.multiply(image_right,W), diameter=(5/4)*D, orientation=O)
            
            # save tip coordinates in an array
            tipcoords[i,:] = x,y
            self.draw.emit(['cross',x,y])
            np.save(save_directory+'softcalibration', tipcoords)  #FLAG: relevant for MSc thesis
        
        # user bias correction
        tipcoord = np.mean(tipcoords, axis=0)
        x,y,z = reference
        micromanipulator.moveAbs(x,y,z)
        userbias = np.array([0, 0])     #should come from human input!
        tipcoord += userbias
        self.draw.emit(['cross',tipcoord[0],tipcoord[1]])
        I = camera.snap()
        io.imsave(save_directory+'softcalibration'+'.tif', I, check_contrast=False)    #FLAG: relevant for MSc thesis
        
        # set micromanipulator and camera coordinate pair of pipette tip
        self._parent.pipette_coordinates_pair = np.vstack([reference, np.array([tipcoord[0], tipcoord[1], np.nan])])
        
        self.finished.emit()
        
    
    @pyqtSlot()
    def autofocus_tip(self):
        # get all relevant parent attributes
        save_directory = self._parent.save_directory
        micromanipulator = self._parent.micromanipulator
        camera = self._parent.camerathread
        
        # algorithm variables
        focusprecision = 0.1    # focal plane finding precision (in microns)
        optimalstepsize = 12    # peak recognizing step size (in microns)
        margin = 0.90           # threshold for max values (percentage)
        
        reference = micromanipulator.getPos()
        
        # Construct Gaussian window
        image = camera.snap()
        width,height = image.shape
        W = ia.makeGaussian(size=image.shape, mu=(width//2,height//2), sigma=(width//12,height//12))
        
        # Initialize array for storing [positions; penalties]
        positionhistory = np.zeros(3)
        penaltyhistory = np.zeros(3)
        
        """"Step up three times to compute penalties [p1,p2,p3]"""
        penalties = np.zeros(3)
        for i in range(3):
            # Capture image
            I = camera.snap()
            
            # Apply image window
            IW = I * W
            
            # Calculate out of focus penalty score
            penalties[i] = ia.comp_variance_of_Laplacian(IW)
            
            # Save position and penalty in history
            positionhistory[i] = micromanipulator.getPos()[2]
            penaltyhistory[i] = penalties[i]
            
            # Move pipette up
            if i < 2:
                micromanipulator.moveRel(dz=optimalstepsize)
            
        """Iteratively find peak in focus penalty values"""
        stepsize = optimalstepsize
        stepsizemin = optimalstepsize/2
        stepsizemax = optimalstepsize*8
        pinbool = np.zeros(3)
        while not np.array_equal(pinbool, [0,1,0]):
            # emit sharpness function
            self.sharpnessfunction.emit(np.vstack([positionhistory,penaltyhistory]))
            # Adjust threshold
            margin = margin*.95 + .05
            logging.info(pinbool)
            
            # Find maximum, middle, and minimum penalty values
            i_min, i_mdl, i_max = np.argsort(penalties)
            p_min, p_mdl, p_max = np.sort(penalties)
            
            # Check which penalty is significant (maximum, minimum, none)
            if margin*p_max > p_mdl:
                pinbool[i_max] = 1
                pinbool[i_mdl] = 0
                pinbool[i_min] = 0
            elif margin*p_max > p_min:
                pinbool[i_max] = 1
                pinbool[i_mdl] = 1
                pinbool[i_min] = 0
            else:
                pinbool[i_max] = 1
                pinbool[i_mdl] = 1
                pinbool[i_min] = 1
            
            # Move micromanipulator towards local maximum through (7 modes)
            if np.array_equal(pinbool, [0,1,0]):
                pass
            elif np.array_equal(pinbool, [1,0,1]):
                x,y,z = reference-np.array([0,0,stepsize])
                micromanipulator.moveAbs(x,y,z)
                I = camera.snap()
                IW = I * W
                penalties[1] = penalties[0]
                penalties[0] = ia.comp_variance_of_Laplacian(IW)
                positionhistory = np.append(positionhistory, micromanipulator.getPos()[2])
                penaltyhistory = np.append(penaltyhistory, penalties[0])
                reference = reference - np.array([0,0,stepsize])
            elif np.array_equal(pinbool, [1,0,0]):
                if stepsize > optimalstepsize:
                    stepsize = stepsize/2
                    penalties[1] = penalties[0]
                    x,y,z = reference+np.array([0,0,stepsize])
                    micromanipulator.moveAbs(x,y,z)
                    I = camera.snap()
                    IW = I * W
                    penalties[2] = ia.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, micromanipulator.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    x,y,z = reference-np.array([0,0,stepsize])
                    micromanipulator.moveAbs(x,y,z)
                    I = camera.snap()
                    IW = I * W
                    penalties[0] = ia.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, micromanipulator.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - np.array([0,0,stepsize])
                elif stepsize < optimalstepsize:
                    stepsize = stepsize*2
                    penalties[1] = penalties[0]
                    x,y,z = reference-np.array([0,0,stepsize])
                    micromanipulator.moveAbs(x,y,z)
                    I = camera.snap()
                    IW = I * W
                    penalties[0] = ia.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, micromanipulator.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - np.array([0,0,stepsize])
                else:
                    penalties = np.roll(penalties,1)
                    x,y,z = reference-np.array([0,0,stepsize])
                    micromanipulator.moveAbs(x,y,z)
                    I = camera.snap()
                    IW = I * W
                    penalties[0] = ia.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, micromanipulator.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - np.array([0,0,stepsize])
            elif np.array_equal(pinbool, [0,0,1]):
                if stepsize > optimalstepsize:
                    stepsize = stepsize/2
                    penalties[1] = penalties[2]
                    x,y,z = reference+3*np.array([0,0,stepsize])
                    micromanipulator.moveAbs(x,y,z)
                    I = camera.snap()
                    IW = I * W
                    penalties[0] = ia.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, micromanipulator.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    x,y,z = reference+5*np.array([0,0,stepsize])
                    micromanipulator.moveAbs(x,y,z)
                    I = camera.snap()
                    IW = I * W
                    penalties[2] = ia.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, micromanipulator.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + 3*np.array([0,0,stepsize])
                elif stepsize < optimalstepsize:
                    stepsize = stepsize*2
                    penalties[1] = penalties[2]
                    x,y,z = reference+2*np.array([0,0,stepsize])
                    micromanipulator.moveAbs(x,y,z)
                    I = camera.snap()
                    IW = I * W
                    penalties[2] = ia.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, micromanipulator.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference
                else:
                    penalties = np.roll(penalties,-1)
                    x,y,z = reference+3*np.array([0,0,stepsize])
                    micromanipulator.moveAbs(x,y,z)
                    I = camera.snap()
                    IW = I * W
                    penalties[2] = ia.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, micromanipulator.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + np.array([0,0,stepsize])
            elif np.array_equal(pinbool, [1,1,0]):
                if stepsize == stepsizemin:
                    penalties = np.roll(penalties,1)
                    x,y,z = reference-np.array([0,0,stepsize])
                    micromanipulator.moveAbs(x,y,z)
                    I = camera.snap()
                    IW = I * W
                    penalties[0] = ia.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, micromanipulator.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - np.array([0,0,stepsize])
                else:
                    stepsize = stepsize/2
                    penalties[1] = penalties[0]
                    x,y,z = reference+np.array([0,0,stepsize])
                    micromanipulator.moveAbs(x,y,z)
                    I = camera.snap()
                    IW = I * W
                    penalties[2] = ia.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, micromanipulator.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    x,y,z = reference-np.array([0,0,stepsize])
                    micromanipulator.moveAbs(x,y,z)
                    I = camera.snap()
                    IW = I * W
                    penalties[0] = ia.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, micromanipulator.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - np.array([0,0,stepsize])
            elif np.array_equal(pinbool, [0,1,1]):
                if stepsize == stepsizemin:
                    penalties = np.roll(penalties,-1)
                    x,y,z = reference+3*np.array([0,0,stepsize])
                    micromanipulator.moveAbs(x,y,z)
                    I = camera.snap()
                    IW = I * W
                    penalties[2] = ia.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, micromanipulator.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + np.array([0,0,stepsize])
                else:
                    stepsize = stepsize/2
                    penalties[1] = penalties[2]
                    x,y,z = reference+3*np.array([0,0,stepsize])
                    micromanipulator.moveAbs(x,y,z)
                    I = camera.snap()
                    IW = I * W
                    penalties[0] = ia.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, micromanipulator.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    x,y,z = reference+5*np.array([0,0,stepsize])
                    micromanipulator.moveAbs(x,y,z)
                    I = camera.snap()
                    IW = I * W
                    penalties[2] = ia.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, micromanipulator.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + 3*np.array([0,0,stepsize])
            elif np.array_equal(pinbool, [1,1,1]):
                if stepsize == stepsizemax:
                    penalties = np.roll(penalties,1)
                    x,y,z = reference-np.array([0,0,stepsize])
                    micromanipulator.moveAbs(x,y,z)
                    I = camera.snap()
                    IW = I * W
                    penalties[0] = ia.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, micromanipulator.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - np.array([0,0,stepsize])
                else:
                    stepsize = 2*stepsize
                    x,y,z = reference-np.array([0,0,stepsize])
                    micromanipulator.moveAbs(x,y,z)
                    I = camera.snap()
                    IW = I * W
                    penalties[1] = penalties[0]
                    penalties[0] = ia.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, micromanipulator.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - np.array([0,0,stepsize])
        
        """Final approach by sampling between the zeros in [0,1,0]"""
        print('Coarse focus found, continue with finetuning')
        while stepsize > focusprecision:
            # Sample ten points between outer penalty values
            penalties = np.zeros(6)
            positions = np.linspace(reference[2], reference[2]+2*stepsize, 6)
            for idx, pos in enumerate(positions):
                micromanipulator.moveAbs(x=reference[0], y=reference[1], z=pos)
                I = camera.snap()
                IW = I * W
                penalties[idx] = ia.comp_variance_of_Laplacian(IW)
                positionhistory = np.append(positionhistory, micromanipulator.getPos()[2])
                penaltyhistory = np.append(penaltyhistory, penalties[idx])
            
            np.save(save_directory+'autofocus_positionhistory_stepsize='+str(stepsize), positions)     #FLAG: relevant for MSc thesis
            np.save(save_directory+'autofocus_penaltyhistory_stepsize='+str(stepsize), penalties)      #FLAG: relevant for MSc thesis
            
            # Locate maximum penalty value
            i_max = penalties.argmax()
            
            # Decrease step size
            stepsize = stepsize/5
            
            # Set reference position one step below maximum penalty position
            reference[2] = positions[i_max] - stepsize
            
        # Move micromanipulator to the position of maximal penalty
        micromanipulator.moveAbs(x=reference[0], y=reference[1], z=positions[i_max])
        np.save(save_directory+'autofocus_positionhistory_foundfocus', positions[i_max])                   #FLAG: relevant for MSc thesis
        I = camera.snap()
        io.imsave(save_directory+'foundfocus'+'.tif', I, check_contrast=False)   #FLAG: relevant for MSc thesis
        
        logging.info('Focus offset found!')
        
        self.finished.emit()
        
        
    
    # @pyqtSlot()
    # def request_imagexygrid(self):
    #     save_directory = self._parent.save_directory
    #     micromanipulator = self._parent.micromanipulator
    #     camera = self._parent.camerathread
    #     x,y,z = micromanipulator.getPos()
    #     stepsize = 50
    #     for i in range(0, 11):
    #         for j in range(0, 11):
    #             micromanipulator.moveAbs(x+i*stepsize, y+j*stepsize,z)
    #             for k in ['a','b']:
    #                 if k == 'b':
    #                     micromanipulator.moveRel(dx=5, dy=0, dz=0)
    #                 snap = camera.snap()
    #                 io.imsave(save_directory+'X%dY%d'%(i*stepsize,j*stepsize)+k+'.tif', snap, check_contrast=False)
    