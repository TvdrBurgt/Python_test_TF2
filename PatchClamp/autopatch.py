# -*- coding: utf-8 -*-
"""
Created on Thu Apr 15 11:12:26 2021

@author: tvdrb
"""

import os
import time
import numpy as np

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
    newimage = pyqtSignal(np.ndarray)
    
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
        
        self.lastview = np.random.rand(2048,2048)
        
        # Static parameter settings
        self.exposure_time = 0.02   # camera exposure time (in seconds)
        
        # Variable paramater settings that methods will update
        self.manipulator_position_absolute = [0, 0, 0]  # x,y,z in micrometers
        self.manipulator_position_relative = [0, 0, 0]  # x,y,z in micrometers
        self.objective_position = 0                     # z in micrometers/10
        
        """
        #====================== Connect hardware devices ======================
        # """
        # # Create a camera instance if the handle is not provided.
        # print('Connecting camera...')
        # if camera_handle == None:
        #     self.hamamatsu_cam_instance = CamActuator()
        #     self.hamamatsu_cam_instance.initializeCamera()
        # else:
        #     self.hamamatsu_cam_instance = camera_handle
        
        # # Create an objective motor instance if the handle is not provided.
        # print('Connecting objective motor...')
        # if motor_handle == None:
        #     self.pi_device_instance = PIMotor()
        # else:
        #     self.pi_device_instance = motor_handle
        
        # # Create a micromanipulator instance if the handle is not provided.
        # print('Connecting micromanipulator...')
        # if micromanipulator_handle == None:
        #     self.micromanipulator_instance = ScientificaPatchStar()
        # else:
        #     self.micromanipulator_instance = micromanipulator_handle
        
        """
        #==================== Get hardware device settings ====================
        """
        # self.manipulator_position_absolute = self.micromanipulator_instance.getPos()
        # self.objective_position = self.pi_device_instance.GetCurrentPos()
        
    def run(self):
        print('data acquisition started')
        while True:
            self.lastview = np.random.rand(2048,2048)
            time.sleep(self.exposure_time)
    
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
        snapped_image = copy(self.lastview)
        print('snap!')
        
        # Emit the newly snapped image for display in widget
        self.newimage.emit(snapped_image)
        
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
        focusprecision = 0.01   # focal plane finding precision (micrometers)
        optimalstepsize = 10    # peak recognizing step size (micrometers)
        margin = 0.95           # threshold for max values (percentage)
        
        # Construct Gaussian window
        I = self.snap_image()
        window = PipetteAutofocus.comp_Gaussian_kernel(size=I.shape[1], fwhm=I.shape[1]/4)
        
        # Set reference pipette position
        reference = self.micromanipulator_instance.getPos()[2]
        
        # Initialize array for storing [positions; penalties]
        positionhistory = np.zeros(3)
        penaltyhistory = np.zeros(3)
        
        """"Step up three times to compute penalties [p1,p2,p3]"""
        print('Step up two times')
        penalties = np.zeros(3)
        for i in range(3):
            # Capture image
            I = self.snap_image()
            
            # Apply image window
            IW = I * window
            
            # Calculate out of focus penalty score
            penalties[i] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
            
            # Save position and penalty in history
            positionhistory[i] = self.micromanipulator_instance.getPos()[2]
            penaltyhistory[i] = penalties[i]
            
            # Move pipette up
            if i < 2:
                print('step up')
                self.micromanipulator_instance.moveAbsZ(reference+(i+1)*optimalstepsize)
            
        """Iteratively find peak in focus penalty values"""
        stepsize = optimalstepsize
        stepsizemin = optimalstepsize/4
        stepsizemax = optimalstepsize*4
        pinbool = np.zeros(3)
        while pinbool != [0,1,0]:
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
                self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                I = self.snap_image()
                IW = I * window
                penalties[1] = penalties[0]
                penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                penaltyhistory = np.append(penaltyhistory, penalties[0])
                reference = reference - stepsize
            elif np.array_equal(pinbool, [1,0,0]):
                if stepsize > optimalstepsize:
                    stepsize = stepsize/2
                    penalties[1] = penalties[0]
                    self.micromanipulator_instance.moveAbsZ(reference+stepsize)
                    I = self.snap_image()
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.snap_image()
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
                elif stepsize < optimalstepsize:
                    stepsize = stepsize*2
                    penalties[1] = penalties[0]
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.snap_image()
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
                else:
                    penalties = np.roll(penalties,1)
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.snap_image()
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
            elif np.array_equal(pinbool, [0,0,1]):
                if stepsize > optimalstepsize:
                    stepsize = stepsize/2
                    penalties[1] = penalties[2]
                    self.micromanipulator_instance.moveAbsZ(reference+3*stepsize)
                    I = self.snap_image()
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    self.micromanipulator_instance.moveAbsZ(reference+5*stepsize)
                    I = self.snap_image()
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + 3*stepsize
                elif stepsize < optimalstepsize:
                    stepsize = stepsize*2
                    penalties[1] = penalties[2]
                    self.micromanipulator_instance.moveAbsZ(reference+2*stepsize)
                    I = self.snap_image()
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference
                else:
                    penalties = np.roll(penalties,-1)
                    self.micromanipulator_instance.moveAbsZ(reference+3*stepsize)
                    I = self.snap_image()
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + stepsize
            elif np.array_equal(pinbool, [1,1,0]):
                if stepsize == stepsizemin:
                    penalties = np.roll(penalties,1)
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.snap_image()
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
                else:
                    stepsize = stepsize/2
                    penalties[1] = penalties[0]
                    self.micromanipulator_instance.moveAbsZ(reference+stepsize)
                    I = self.snap_image()
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.snap_image()
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
            elif np.array_equal(pinbool, [0,1,1]):
                if stepsize == stepsizemin:
                    penalties = np.roll(penalties,-1)
                    self.micromanipulator_instance.moveAbsZ(reference+3*stepsize)
                    I = self.snap_image()
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + stepsize
                else:
                    stepsize = stepsize/2
                    penalties[1] = penalties[2]
                    self.micromanipulator_instance.moveAbsZ(reference+3*stepsize)
                    I = self.snap_image()
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    self.micromanipulator_instance.moveAbsZ(reference+5*stepsize)
                    I = self.snap_image()
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + 3*stepsize
            elif np.array_equal(pinbool, [1,1,1]):
                if stepsize == stepsizemax:
                    penalties = np.roll(penalties,1)
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.snap_image()
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
                else:
                    stepsize = 2*stepsize
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.snap_image()
                    IW = I * window
                    penalties[1] = penalties[0]
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
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
                IW = I * window
                penalties[idx] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                penaltyhistory = np.append(penaltyhistory, penalties[idx])
            
            # Locate maximum penalty value
            i_max = penalties.argmax()
            
            # Decrease step size
            stepsize = stepsize/5
            
            # Set reference position one step below maximum penalty position
            reference = positions[i_max] - stepsize
        
        print('Focus offset found!')
                
    
    
    
    
    # def move_focus(self, distance):
    #     self.initial_focus_position = self.pi_device_instance.pidevice.qPOS(self.pi_device_instance.pidevice.axes)['1']
    #     print("init_focus_position : {}".format(self.initial_focus_position))
        
    #     self.target_position = self.initial_focus_position + distance
        
    #     self.pi_device_instance.move(self.target_position)
    
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
        
        This method returns the coordinate transformation coefficients that
        align the coordinate systems of the PatchStar with the camera.
        """
        
        pass
    
        
if __name__ == "__main__":
    instance = AutomaticPatcher()
    
