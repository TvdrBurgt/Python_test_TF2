# -*- coding: utf-8 -*-
"""
Created on Thu Apr 15 11:12:26 2021

@author: tvdrb
"""

import numpy as np

from HamamatsuCam.HamamatsuActuator import CamActuator
from PI_ObjectiveMotor.focuser import PIMotor
from PatchClamp import ScientificaPatchStar

from ImageAnalysis.ImageProcessing_AutoPatch import PipetteTipDetector, PipetteAutofocus


class AutomaticPatcher():
    
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
        super().__init__(*args, **kwargs)
        
        # Static parameter settings
        self.exposure_time = 0.02   # in seconds
        
        # Variable paramater settings that methods will update
        self.manipulator_position_absolute = [0, 0, 0]  # x,y,z in micrometers
        self.manipulator_position_relative = [0, 0, 0]  # x,y,z in micrometers
        self.objective_position = 0                     # z in micrometers/10
        self.focus_penalty = 0                          # variance in pixel^2
        
        """
        #====================== Connect hardware devices ======================
        """
        # Create a camera instance if the handle is not provided.
        print('Connecting camera')
        if camera_handle == None:
            self.hamamatsu_cam_instance = CamActuator()
            self.hamamatsu_cam_instance.initializeCamera()
        else:
            self.hamamatsu_cam_instance = camera_handle
        
        # Create an objective motor instance if the handle is not provided.
        print('Connecting objective motor')
        if motor_handle == None:
            self.pi_device_instance = PIMotor()
        else:
            self.pi_device_instance = motor_handle
            
        # Create a micromanipulator instance if the handle is not provided.
        print('Initiating micromanipulator')
        if micromanipulator_handle == None:
            self.scientifica_device_instance = ScientificaPatchStar()
        else:
            self.scientifica_device_instance = micromanipulator_handle
        
        """
        #==================== Get hardware device settings ====================
        """
        self.manipulator_position_absolute = self.scientifica_device_instance.getPos()
        self.objective_position = self.pi_device_instance.GetCurrentPos()
        
    
    def disconnect_devices(self):
        # Disconnect camera
        try:
            self.HamamatsuCam_instance.Exit()
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
        snapped_image = self.HamamatsuCam_instance.SnapImage(self.exposure_time)
        
        # Update figure...
        
        return snapped_image
        
    def autofocus(self):
        """ Focus pipette tip from above.
        
        This method bring the pipette tip into focus by approaching the focal
        plane from above using the micromanipulator only. We assume a pre-
        calibrated coordinate system and the pipette tip in the FOV center.
        """
        
        # Step sizes to lower the micromanipulator with
        stepsize = 10  # Step size in micrometers
        
        # Construct Gaussian window
        I = self.snap_image()
        window = PipetteAutofocus.comp_Gaussian_kernel(size=I.shape[1], fwhm=I.shape[1]/4)
        
        """"Step up three times and check position relative to the focus"""
        penalties = np.zeros(3)
        for i in range(len(penalties)):
            
            # Capture image and apply Gaussian window
            I = self.snap_image()
            IW = I * window
            
            # Calculate out of focus penalty score
            penalties[i] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
            
            # Move pipette up
            if i < len(penalties)-1:
                self.manipulator_position_relative(stepsize)
        
        """"Continue finding the focus (offset)"""
        if np.argmax(penalties) == 0:
            # Focal plane is down
            self.manipulator_position_relative(-(len(penalties)-1)*stepsize)
            
            while stepsize >= 0.1:
                # Move pipette down
                self.manipulator_position_relative(-stepsize)
                
                # Capture image and apply Gaussian window
                I = self.snap_image()
                IW = I * window
                
                # Calculate out of focus penalty score
                penalties = np.roll(penalties, 1)
                penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                
                if np.argmax(penalties) == 0:
                    pass
                else:
                    self.manipulator_position_relative(stepsize)
                    stepsize = stepsize/10
            
        elif np.argmax(penalties) == 1:
            # Focus plane is in between
            pass
            
        else:
            # Focal plane is above
            pass
    
    def move_focus(self, distance):
        """
        # =============================================================================
        #         connect the Objective motor
        # =============================================================================
        """
        print('----------------------Starting to connect the Objective motor-------------------------')
        self.pi_device_instance = PIMotor()
        print('Objective motor connected.')
        self.initial_focus_position = self.pi_device_instance.pidevice.qPOS(self.pi_device_instance.pidevice.axes)['1']
        print("init_focus_position : {}".format(self.initial_focus_position))
        
        self.target_position = self.initial_focus_position + distance
        
        self.pi_device_instance.move(self.target_position)
    
    def detect_pipette_tip(image):
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
    
    def focus_pipette(self):
        # step 1. localize pipette tip
        # step 2. move tip to the center of FOV
        # step 3. initiate focus finder
        #           a. calculate out of focus penalty
        penalty = PipetteAutofocus.comp_variance_of_Laplacian(self.snap_image())
        print(penalty)
        #           b. move pipette down
        #           repeat until local maximum found
        pass
        
    def calibrate_coordsys(self):
        """ Calibrate PatchStar coordinate system.
        
        This method returns the coordinate transformation coefficients that
        align the coordinate systems of the PatchStar with the camera.
        """
        
        pass
    
        
