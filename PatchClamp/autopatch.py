# -*- coding: utf-8 -*-
"""
Created on Thu Apr 15 11:12:26 2021

@author: tvdrb
"""

from HamamatsuCam.HamamatsuActuator import CamActuator
from PI_ObjectiveMotor.focuser import PIMotor

from ImageAnalysis.ImageProcessingAutopatch import DetectPipetteTips



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
    
    def focus_pipette(self):
        
        pass
        
    def calibrate_coordsys(self):
        """ Calibrate PatchStar coordinate system.
        
        This method returns the coordinate transformation coefficients that
        align the coordinate systems of the PatchStar with the camera.
        """
        
        pass
    
        
