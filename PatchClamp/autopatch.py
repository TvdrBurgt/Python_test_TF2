# -*- coding: utf-8 -*-
"""
Created on Thu Apr 15 11:12:26 2021

@author: tvdrb
"""

import os
import time
import numpy as np
import datetime
from skimage import io

from PyQt5.QtCore import pyqtSignal, QObject

from copy import copy

from PyQt5.QtCore import pyqtSignal, QObject

# Ensure that the Widget can be run either independently or as part of Tupolev.
if __name__ == "__main__":
    os.chdir(os.getcwd() + '\\..')
from HamamatsuCam.HamamatsuActuator import CamActuator
from PI_ObjectiveMotor.focuser import PIMotor
from PatchClamp.micromanipulator import ScientificaPatchStar

from PatchClamp.ImageProcessing_AutoPatch import PipetteTipDetector, PipetteAutofocus


class AutomaticPatcher(QObject):
    livesignal = pyqtSignal(np.ndarray)
    snapsignal = pyqtSignal(np.ndarray)
    
    def __init__(self, camera_handle = None, motor_handle = None, \
                 micromanipulator_handle = None, *args, **kwargs):
        """
        
        Parameters
        ----------
        camera_handle : TYPE, optional
            Handle to control Hamamatsu camera. The default is None.
        motor_handle : TYPE, optional
            Handle to control PI motor. The default is None.
        patchstar_handle : TYPE, optional
            Handle to control Scientifica PatchStar. Default is None.

        Returns
        -------
        None.
        
        """
        QObject.__init__(self, *args, **kwargs)
        
        self.savedirectory = r'M:\tnw\ist\do\projects\Neurophotonics\Brinkslab\Data\Thijs\Save directory\\'
        
        # Static parameter settings
        self.exposure_time = 0.02   # camera exposure time (in seconds)
        
        # Variable paramater settings that methods will update
        # self.manipulator_position_absolute = [0, 0, 0]  # x,y,z in micrometers
        # self.manipulator_position_relative = [0, 0, 0]  # x,y,z in micrometers
        self.objective_position = 0                     # z in micrometers/10
        
        """
        #====================== Connect hardware devices ======================
        """
        # Create a camera instance if the handle is not provided.
        print('Connecting camera...')
        if camera_handle == None:
            self.hamamatsu_cam_instance = CamActuator()
            self.hamamatsu_cam_instance.initializeCamera()
        else:
            self.hamamatsu_cam_instance = camera_handle
        
        # # Create an objective motor instance if the handle is not provided.
        # print('Connecting objective motor...')
        # if motor_handle == None:
        #     self.pi_device_instance = PIMotor()
        # else:
        #     self.pi_device_instance = motor_handle
        
        # Create a micromanipulator instance if the handle is not provided.
        print('Connecting micromanipulator...')
        if micromanipulator_handle == None:
            self.micromanipulator_instance = ScientificaPatchStar()
        else:
            self.micromanipulator_instance = micromanipulator_handle
        
        """
        #==================== Get hardware device settings ====================
        """
        # self.manipulator_position_absolute = self.micromanipulator_instance.getPos()
        # self.objective_position = self.pi_device_instance.GetCurrentPos()
    
    def disconnect_devices(self):
        # Disconnect camera
        try:
            self.hamamatsu_cam_instance.Exit()
            print('Camera disconnected.')
        except:
            pass
        
        # Disconnect objective motor
        try:
            self.pi_device_instance.CloseMotorConnection()
            print('Objective motor disconnected.')
        except:
            pass
    
    
    def snap_image(self):
        # Snap image with camera instance
        snapped_image = self.hamamatsu_cam_instance.SnapImage(self.exposure_time)
        print('snap!')
        
        # Emit the newly snapped image for display in widget
        self.snapsignal.emit(snapped_image)
        
        return snapped_image
        
    def autofocus_pipette(self):
        """ Focus pipette tip from above.
        
        This method bring the pipette tip into focus by approaching the focal
        plane from above using the micromanipulator only. We start by moving
        the pipette up to check the position relative to the focal plane as a
        safety measure. Furthermore, we assume a pre-calibrated coordinate
        system and the pipette tip in the FOV center.
        """
        # Parameters to vary
        focusprecision = 1      # focal plane finding precision (micrometers/100)
        optimalstepsize = 1000  # peak recognizing step size (micrometers/100)
        margin = 0.96           # threshold for max values (percentage)
        
        # Construct Gaussian window
        I = self.snap_image()
        window = PipetteAutofocus.comp_Gaussian_kernel(size=I.shape[1], fwhm=I.shape[1]/4)
        
        # Set reference pipette position as origin
        # self.micromanipulator_instance.setZero()
        # reference = self.micromanipulator_instance.getPos()[2]
        reference = self.micromanipulator_instance.position[2]
        
        # Initialize array for storing [positions; penalties]
        positionhistory = np.zeros(3)
        penaltyhistory = np.zeros(3)
        
        """"Step up three times to compute penalties [p1,p2,p3]"""
        print('Step up two times')
        penalties = np.zeros(3)
        for i in range(3):
            # Capture image
            I = self.snap_image()
            io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
            
            # Apply image window
            IW = I * window
            
            # Calculate out of focus penalty score
            penalties[i] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
            
            # Save position and penalty in history
            positionhistory[i] = self.micromanipulator_instance.position[2]
            penaltyhistory[i] = penalties[i]
            
            # Move pipette up
            if i < 2:
                print('step up')
                # self.micromanipulator_instance.moveAbsZ(reference+(i+1)*optimalstepsize)
                self.micromanipulator_instance.moveRel(0,0,optimalstepsize)
            
        """Iteratively find peak in focus penalty values"""
        stepsize = optimalstepsize
        stepsizemin = optimalstepsize/2
        stepsizemax = optimalstepsize*8
        pinbool = np.zeros(3)
        while not np.array_equal(pinbool, [0,1,0]):
            # Adjust threshold
            margin = margin*.995 + .005
            
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
                
            print(pinbool)
            print(reference)
            
            # Move micromanipulator towards local maximum through (7 modes)
            if np.array_equal(pinbool, [0,1,0]):
                pass
            elif np.array_equal(pinbool, [1,0,1]):
                # self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                I = self.snap_image()
                io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                IW = I * window
                penalties[1] = penalties[0]
                penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                positionhistory = np.append(positionhistory, self.micromanipulator_instance.position[2])
                penaltyhistory = np.append(penaltyhistory, penalties[0])
                reference = reference - stepsize
            elif np.array_equal(pinbool, [1,0,0]):
                if stepsize > optimalstepsize:
                    stepsize = stepsize/2
                    penalties[1] = penalties[0]
                    self.micromanipulator_instance.moveAbsZ(reference+stepsize)
                    I = self.snap_image()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.position[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.snap_image()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.position[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
                elif stepsize < optimalstepsize:
                    stepsize = stepsize*2
                    penalties[1] = penalties[0]
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.snap_image()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.position[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
                else:
                    penalties = np.roll(penalties,1)
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.snap_image()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.position[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
            elif np.array_equal(pinbool, [0,0,1]):
                if stepsize > optimalstepsize:
                    stepsize = stepsize/2
                    penalties[1] = penalties[2]
                    self.micromanipulator_instance.moveAbsZ(reference+3*stepsize)
                    I = self.snap_image()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.position[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    self.micromanipulator_instance.moveAbsZ(reference+5*stepsize)
                    I = self.snap_image()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.position[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + 3*stepsize
                elif stepsize < optimalstepsize:
                    stepsize = stepsize*2
                    penalties[1] = penalties[2]
                    self.micromanipulator_instance.moveAbsZ(reference+2*stepsize)
                    I = self.snap_image()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.position[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference
                else:
                    penalties = np.roll(penalties,-1)
                    self.micromanipulator_instance.moveAbsZ(reference+3*stepsize)
                    I = self.snap_image()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.position[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + stepsize
            elif np.array_equal(pinbool, [1,1,0]):
                if stepsize == stepsizemin:
                    penalties = np.roll(penalties,1)
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.snap_image()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.position[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
                else:
                    stepsize = stepsize/2
                    penalties[1] = penalties[0]
                    self.micromanipulator_instance.moveAbsZ(reference+stepsize)
                    I = self.snap_image()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.position[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.snap_image()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.position[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
            elif np.array_equal(pinbool, [0,1,1]):
                if stepsize == stepsizemin:
                    penalties = np.roll(penalties,-1)
                    self.micromanipulator_instance.moveAbsZ(reference+3*stepsize)
                    I = self.snap_image()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.position[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + stepsize
                else:
                    stepsize = stepsize/2
                    penalties[1] = penalties[2]
                    self.micromanipulator_instance.moveAbsZ(reference+3*stepsize)
                    I = self.snap_image()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.position[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    self.micromanipulator_instance.moveAbsZ(reference+5*stepsize)
                    I = self.snap_image()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.position[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + 3*stepsize
            elif np.array_equal(pinbool, [1,1,1]):
                if stepsize == stepsizemax:
                    penalties = np.roll(penalties,1)
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.snap_image()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.position[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
                else:
                    stepsize = 2*stepsize
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.snap_image()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[1] = penalties[0]
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.position[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
        
        """Final approach by sampling between the zeros in [0,1,0]"""
        print('Coarse focus found, continue with finetuning')
        while stepsize > focusprecision:
            # Sample ten points between outer penalty values
            penalties = np.zeros(11)
            positions = np.linspace(reference, reference+2*stepsize, 11)
            for idx, pos in enumerate(positions):
                self.micromanipulator_instance.moveAbsZ(pos)
                I = self.snap_image()
                io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                IW = I * window
                penalties[idx] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                positionhistory = np.append(positionhistory, self.micromanipulator_instance.position[2])
                penaltyhistory = np.append(penaltyhistory, penalties[idx])
            
            # Locate maximum penalty value
            i_max = penalties.argmax()
            
            # Decrease step size
            stepsize = stepsize/5
            
            # Set reference position one step below maximum penalty position
            reference = positions[i_max] - stepsize
        
        print('Focus offset found!')
        np.save(self.savedirectory+'penaltyhistory.txt', penaltyhistory)
        np.save(self.savedirectory+'positionhistory.txt', positionhistory)
                
    
    def detect_pipette_tip(self):
        
        # Snap image
        image = self.snap_image()
        
        UPPER_ANGLE = 97.5      #Only for first calibration
        LOWER_ANGLE = 82.5      #Only for first calibration
        PIPETTEDIAMETER = 16.5  #Only for first calibration
    
        # First round of pipette tip detection
        x1, y1 = PipetteTipDetector.locate_tip(image,
                                               UPPER_ANGLE,
                                               LOWER_ANGLE,
                                               PIPETTEDIAMETER,
                                               blursize=15,
                                               angle_range=10,
                                               num_angles=1000,
                                               num_peaks=8
                                               )
        
        # Crop image
        cropped_image, xref, yref, faultylocalisation = PipetteTipDetector.crop_image(image, x1, y1)
        
        # Second round of pipette tip detection
        if not faultylocalisation:
            x2, y2 = PipetteTipDetector.locate_tip(cropped_image,
                                                  UPPER_ANGLE,
                                                  LOWER_ANGLE,
                                                  PIPETTEDIAMETER,
                                                  blursize=4,
                                                  angle_range=10,
                                                  num_angles=5000,
                                                  num_peaks=6
                                                  )
        else:
            x2 = np.nan
            y2 = np.nan
        
        # Adjusting x2 and y2 with the reference coordinates
        x = xref + x2
        y = yref + y2
        
        # Return pipette tip coordinates
        print('Pipette tip detected @ (x,y) = (%f,%f)' % (x,y))
        
        return x, y
        
    def calibrate_coordsys(self):
        """ Calibrate PatchStar coordinate system.
        
        This method returns the rotation angles that align the coordinate-
        systems of the PatchStar with the camera. We assume the z-axis of the
        PatchStar always points up.
        """
        # Specify the number of steps and their size for slope detection
        numsteps = 10
        xstep = 500           # in micrometer/100
        ystep = 500           # in micrometer/100
        zstep = 500           # in micrometer/100
        
        # Set the micromanipulator absolute- and relative position
        self.manipulator_position_absolute = self.micromanipulator_instance.position
        step2pos = lambda pos: np.add(self.manipulator_position_absolute, pos)
        
        
        for idx, step in enumerate([[xstep,0,0], [0,ystep,0], [0,0,zstep]]):
            v = np.empty(numsteps)
            w = np.empty(numsteps)
            
            # Detect pipette tip and move micromanipulator
            for i in numsteps:
                v[i], w[i] = self.detect_pipette_tip()
                if i < numsteps:
                    self.micromanipulator_instance.moveAbs(step2pos(np.multiply(step,(i+1))))
                else:
                    self.micromanipulator_instance.moveAbs(step2pos([0,0,0]))
            
            # Reduce array of pipette coordinates to one x and one y
            if idx == 1:
                Ex = [np.nanmean(v)/xstep, np.nanmean(w)/xstep, 0]
            elif idx == 2:
                Ey = [np.nanmean(v)/ystep, np.nanmean(w)/ystep, 0]
            elif idx == 3:
                Ez = [np.nanmean(v)/zstep, np.nanmean(w)/zstep, 0]
        
        # Calculate rotation angle: gamma
        gamma = np.arctan(-Ex[0]/Ex[1])
        
        # Choose between gamma and gamma-pi and rotate accordingly
        self.micromanipulator_instance.constructrotationmatrix(0, 0, gamma)
        Ey_gammarotated = self.micromanipulator_instance.Rinv.dot(Ey)
        if Ey_gammarotated[1] < 0:
            gamma = gamma - np.pi
            self.micromanipulator_instance.constructrotationmatrix(0, 0, gamma)
        
        # Calculate rotation angles: alpha, beta
        alpha = np.arcsin(Ez[0]*np.sin(gamma) + Ez[1]*np.cos(gamma))
        beta = (-Ez[0]*np.cos(gamma) + Ez[1]*np.sin(gamma))/np.cos(alpha)
        
        # Apply full rotation matrix and verify Ey maps to y-axis
        self.micromanipulator_instance.constructrotationmatrix(alpha, beta, gamma)
        Ey_rotated = self.micromanipulator_instance.Rinv.dot(Ey)
        print(Ey_rotated)
    
    
    # def move_focus(self, distance):
    #     self.initial_focus_position = self.pi_device_instance.pidevice.qPOS(self.pi_device_instance.pidevice.axes)['1']
    #     print("init_focus_position : {}".format(self.initial_focus_position))
        
    #     self.target_position = self.initial_focus_position + distance
        
    #     self.pi_device_instance.move(self.target_position)
    
    
if __name__ == "__main__":
    instance = AutomaticPatcher()
    
