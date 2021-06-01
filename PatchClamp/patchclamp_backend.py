# -*- coding: utf-8 -*-
"""
Created on Tue Jun  1 11:35:14 2021

@author: tvdrb
"""

import numpy as np
import os
import datetime
from skimage import io
import ctypes

from copy import copy

from PyQt5.QtCore import pyqtSignal, QThread

if __name__ == "__main__":
    os.chdir(os.getcwd() + '\\..')
from HamamatsuCam.HamamatsuDCAM import HamamatsuCameraMR, DCAMAPI_INIT
from PatchClamp.micromanipulator import ScientificaPatchStar

from PatchClamp.ImageProcessing_AutoPatch import PipetteTipDetector, PipetteAutofocus


class CameraThread(QThread):
    snapsignal = pyqtSignal(np.ndarray)
    livesignal = pyqtSignal(np.ndarray)
    
    def __init__(self):
        # Class settings
        self.live = True
        # Camera settings
        self.isrunning = False
        self.frame = np.ndarray
        # self.initializeCamera()
        # QThread settings
        super().__init__()
        self.moveToThread(self)
        self.started.connect(self.acquire)
        
    def initializeCamera(self):
        dcamapi_path = r"M:\tnw\ist\do\projects\Neurophotonics\Brinkslab\People\Xin Meng\Code\Python_test\HamamatsuCam\19_12\dcamapi.dll"
        self.dcam = ctypes.WinDLL(dcamapi_path)

        paraminit = DCAMAPI_INIT(0, 0, 0, 0, None, None)
        paraminit.size = ctypes.sizeof(paraminit)
        error_code = self.dcam.dcamapi_init(ctypes.byref(paraminit))

        n_cameras = paraminit.iDeviceCount
        print("found:", n_cameras, "cameras")

        if n_cameras > 0:
            self.hcam = HamamatsuCameraMR(camera_id=0)

            # Enable defect correction
            self.hcam.setPropertyValue("defect_correct_mode", 2)
            # Set the readout speed to fast.
            self.hcam.setPropertyValue("readout_speed", 2)
            # Set the binning to 1.
            self.hcam.setPropertyValue("binning", "1x1")
            # Set exposure time to 0.02
            self.hcam.setPropertyValue("exposure_time", 0.1)

            self.GetKeyCameraProperties()
        
    def acquire(self):
        self.isrunning = True
        # self.hcam.acquisition_mode = "run_till_abort"
        # self.hcam.startAcquisition()
        print("camera acquisition started")

        while self.isrunning:
            ## Mutex lock here?
            # [frames, dims] = self.hcam.getFrames()
            # self.frame = np.resize(frames[-1].np_array, (dims[1], dims[0]))
            self.frame = np.random.rand(2048, 2048)
            QThread.msleep(10)
            if self.live:
                self.livesignal.emit(self.frame)
            else:
                pass
        print("camera acquisition stopped")
        
    def snap(self):
        # Mutex lock here?
        last_view = copy(self.frame)
        print("snap!")
        self.snapsignal.emit(last_view)
        
        return last_view
        
    def start_canvasupdates(self):
        self.live = True
        
    def stop_canvasupdates(self):
        self.live = False
    
    def __del__(self):
        self.isrunning = False
        # self.hcam.stopAcquisition()
        self.quit()
        self.wait()



class AutoPatchThread(QThread):

    def __init__(self, camera_handle):
        self.camera = camera_handle
        
        super().__init__()
        self.moveToThread(self)
        
        # print('Connecting micromanipulator...')
        # self.micromanipulator_instance = ScientificaPatchStar()
        # self.manipulator_position_absolute = self.micromanipulator_instance.getPos()
        
    def __del__(self):
        self.quit()
        self.wait()
    
    def detect_pipette_tip(self):
        """ Detects the pipette tip position
        
        This method detects the pipette tip in a FOV and returns the x- and y-
        coordinates of the tip in pixels. Note that this algorithm works best
        with a pipette tip in the center of the field of view.
        """
        print('tip detection started')
        UPPER_ANGLE = 97.5      #Only for first calibration
        LOWER_ANGLE = 82.5      #Only for first calibration
        PIPETTEDIAMETER = 16.5  #Only for first calibration
        
        # Make snap
        image = self.camera.snap()
        
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
    
    def autofocus_pipette(self):
        """ Focus pipette tip from above.
        
        This method bring the pipette tip into focus by approaching the focal
        plane from above using the micromanipulator only. We start by moving
        the pipette up to check the position relative to the focal plane as a
        safety measure. Furthermore, we assume a pre-calibrated coordinate
        system and the pipette tip in the FOV center.
        """
        print('autofocus started')
        self.micromanipulator_instance.setZero()
        
        # Parameters to vary
        focusprecision = 10     # focal plane finding precision (micrometers/100)
        optimalstepsize = 1000  # peak recognizing step size (micrometers/100)
        margin = 0.96           # threshold for max values (percentage)
        
        # Get pipette tip position
        [xpos, ypos] = self.detect_pipette_tip()
        
        # Construct Gaussian window
        I = self.camera.snap()
        window = PipetteAutofocus.comp_Gaussian_kernel(size=I.shape[1], fwhm=I.shape[1]/4, center=[xpos,ypos])
        
        # Set reference pipette position as origin
        self.micromanipulator_instance.setZero()
        reference = self.micromanipulator_instance.getPos()[2]
        
        # Initialize array for storing [positions; penalties]
        positionhistory = np.zeros(3)
        penaltyhistory = np.zeros(3)
        
        """"Step up three times to compute penalties [p1,p2,p3]"""
        print('Step up two times')
        penalties = np.zeros(3)
        for i in range(3):
            # Capture image
            I = self.camera.snap()
            io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
            
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
                self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                I = self.camera.snap()
                io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
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
                    I = self.camera.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.camera.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
                elif stepsize < optimalstepsize:
                    stepsize = stepsize*2
                    penalties[1] = penalties[0]
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.camera.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
                else:
                    penalties = np.roll(penalties,1)
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.camera.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
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
                    I = self.camera.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    self.micromanipulator_instance.moveAbsZ(reference+5*stepsize)
                    I = self.camera.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + 3*stepsize
                elif stepsize < optimalstepsize:
                    stepsize = stepsize*2
                    penalties[1] = penalties[2]
                    self.micromanipulator_instance.moveAbsZ(reference+2*stepsize)
                    I = self.camera.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference
                else:
                    penalties = np.roll(penalties,-1)
                    self.micromanipulator_instance.moveAbsZ(reference+3*stepsize)
                    I = self.camera.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + stepsize
            elif np.array_equal(pinbool, [1,1,0]):
                if stepsize == stepsizemin:
                    penalties = np.roll(penalties,1)
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.camera.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
                else:
                    stepsize = stepsize/2
                    penalties[1] = penalties[0]
                    self.micromanipulator_instance.moveAbsZ(reference+stepsize)
                    I = self.camera.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.camera.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
            elif np.array_equal(pinbool, [0,1,1]):
                if stepsize == stepsizemin:
                    penalties = np.roll(penalties,-1)
                    self.micromanipulator_instance.moveAbsZ(reference+3*stepsize)
                    I = self.camera.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + stepsize
                else:
                    stepsize = stepsize/2
                    penalties[1] = penalties[2]
                    self.micromanipulator_instance.moveAbsZ(reference+3*stepsize)
                    I = self.camera.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    self.micromanipulator_instance.moveAbsZ(reference+5*stepsize)
                    I = self.camera.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + 3*stepsize
            elif np.array_equal(pinbool, [1,1,1]):
                if stepsize == stepsizemax:
                    penalties = np.roll(penalties,1)
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.camera.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_instance.getPos()[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
                else:
                    stepsize = 2*stepsize
                    self.micromanipulator_instance.moveAbsZ(reference-stepsize)
                    I = self.camera.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
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
                I = self.camera.snap()
                io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
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
        np.save(self.savedirectory+'penaltyhistory.txt', penaltyhistory)
        np.save(self.savedirectory+'positionhistory.txt', positionhistory)
    
    







