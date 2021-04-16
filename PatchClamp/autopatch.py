# -*- coding: utf-8 -*-
"""
Created on Thu Apr 15 11:12:26 2021

@author: tvdrb
"""

import numpy as np

from HamamatsuCam.HamamatsuActuator import CamActuator
from PI_ObjectiveMotor.focuser import PIMotor

from ImageAnalysis.ImageProcessing_AutoPatch import PipetteTipDetector


class AutomaticPatcher():
    
    def __init__(self, camera_handle = None, motor_handle = None, \
                 patchstar_handle = None, *args, **kwargs):
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
        
        # Create a camera instance if the handle is not provided.
        if camera_handle == None:
            self.HamamatsuCam_ins = CamActuator()
            self.HamamatsuCam_ins.initializeCamera()
        else:
            self.HamamatsuCam_ins = camera_handle
        
        # Create an objective motor instance if the handle is not provided.
        if motor_handle == None:
            self.pi_device_instance = PIMotor()
        else:
            self.pi_device_instance = motor_handle
        
    
    def snap_image(self):
        exposure_time = 0.1     # Seconds
        snapped_image = self.HamamatsuCam_ins.SnapImage(exposure_time)
        
        return snapped_image
    
    def detect_pipette_tip(image):
        UPPER_ANGLE = 97.5         # Only for first calibration
        LOWER_ANGLE = 82.5         # Only for first calibration
        PIPETTEDIAMETER = 16.5     # Only for first calibration
    
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
        
        pass
        
    def calibrate_coordsys(self):
        """ Calibrate PatchStar coordinate system.
        
        This method returns the coordinate transformation coefficients that
        align the coordinate systems of the PatchStar with the camera.
        """
        
        pass
    
        
