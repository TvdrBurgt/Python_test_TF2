# -*- coding: utf-8 -*-
"""
Created on Thu Apr 15 11:12:26 2021

@author: tvdrb
"""

import os
import ctypes
import datetime
import logging
import numpy as np

from copy import copy
from skimage import io

from PyQt5.QtCore import pyqtSignal, pyqtSlot, QThread, QMutex

if __name__ == "__main__":
    os.chdir(os.getcwd() + '\\..')
from HamamatsuCam.HamamatsuDCAM import HamamatsuCameraMR, DCAMAPI_INIT
# from PI_ObjectiveMotor.focuser import PIMotor
from PatchClamp.ImageProcessing_AutoPatch import PipetteTipDetector, PipetteAutofocus


class CameraThread(QThread):
    snapsignal = pyqtSignal(np.ndarray)
    livesignal = pyqtSignal(np.ndarray)

    def __init__(self):
        # Class attributes
        self.frame = np.random.rand(2048, 2048)
        # self.initializeCamera()
        
        # QThread attributes
        super().__init__()
        self.isrunning = False
        self.mutex = QMutex()
        self.moveToThread(self)
        self.started.connect(self.acquire)

    def __del__(self):
        self.isrunning = False
        # self.dcam.dcamapi_uninit()
        self.quit()
        self.wait()
        
    def initializeCamera(self):
        # Can we make this path relative?
        dcamapi_path = r"M:\tnw\ist\do\projects\Neurophotonics\Brinkslab\People\Xin Meng\Code\Python_test\HamamatsuCam\19_12\dcamapi.dll"
        self.dcam = ctypes.WinDLL(dcamapi_path)

        paraminit = DCAMAPI_INIT(0, 0, 0, 0, None, None)
        paraminit.size = ctypes.sizeof(paraminit)
        self.dcam.dcamapi_init(ctypes.byref(paraminit))

        n_cameras = paraminit.iDeviceCount
        print("found:", n_cameras, "cameras")

        if n_cameras > 0:
            self.hcam = HamamatsuCameraMR(camera_id=0)
            
            # Enable defect correction
            self.hcam.setPropertyValue("defect_correct_mode", 2)
            # Set the readout speed to fast
            self.hcam.setPropertyValue("readout_speed", 2)
            # Set the binning
            self.hcam.setPropertyValue("binning", "1x1")
            # Set exposure time
            self.hcam.setPropertyValue("exposure_time", 0.2)
    
    @pyqtSlot()
    def acquire(self):
        # Start acquisition and wait more than exposure time for camera start
        # self.hcam.startAcquisition()
        QThread.msleep(1000) # Not sure if necessary with mutex lock
        logging.info("Camera acquisition started")
        
        self.isrunning = True
        while self.isrunning:
            # Mutex lock prevents external access of variables and executes lines in order
            self.mutex.lock()
            
            # Retrieve frames from the camera buffer
            # [frames, dims] = self.hcam.getFrames()
            
            # Resize last frame in the buffer to be displayed
            # self.frame = np.resize(frames[-1].np_array, (dims[1], dims[0]))
            self.frame = np.random.rand(2048, 2048)
            QThread.msleep(200)
            
            # Emit a frame every time it is refreshed
            self.livesignal.emit(self.frame)
            
            # Mutex unlock so snap can access last frame
            self.mutex.unlock()
        
        # Stop camera acquisition when exiting the thread
        # self.hcam.stopAcquisition()
        logging.info("Camera acquisition stopped")
        

    def snap(self):
        # Mutex lock to wait for camera thread to release a frame
        self.mutex.lock()
        
        # copy released frame
        snapshot = copy(self.frame)
        
        # Mutex unlock so the camera thread can continue again
        self.mutex.unlock()
        
        # Emit frame to display
        self.snapsignal.emit(snapshot)
        logging.info("snap!")

        return snapshot


class AutoPatchThread(QThread):
    finished = pyqtSignal()

    def __init__(self, camera_handle=None, manipulator_handle=None, objective_handle=None):
        # Class attributes
        self.camera_handle = camera_handle
        self.micromanipulator_handle = manipulator_handle
        self.objective_handle = objective_handle
        self.pipettetip = [np.nan, np.nan]
        
        # QThread attributes
        super().__init__()
        self.isrunning = False
        self.moveToThread(self)
        self.finished.connect(self.__del__)
        self.started.connect(self.__del__)

    def __del__(self):
        self.isrunning = False
        self.quit()
        self.wait()
        
    def request(self, slot):
        self.started.disconnect()
        if slot == "autofocus":
            self.started.connect(self.autofocus_pipette)
        elif slot == "detect":
            self.started.connect(self.detect_pipette_tip)
        elif slot == "calibrate":
            self.started.connect(self.calibrate_coordsys)
        self.isrunning = True
        self.start()
    
    @pyqtSlot()
    def detect_pipette_tip(self):
        """ Detects the pipette tip position
        
        This method detects the pipette tip in a FOV and returns the x- and y-
        coordinates of the tip in pixels. Note that this algorithm works best
        with a pipette tip in the center of the field of view.
        """
        logging.info('tip detection started')
        UPPER_ANGLE = 97.5      #Only for first calibration
        LOWER_ANGLE = 82.5      #Only for first calibration
        PIPETTEDIAMETER = 16.5  #Only for first calibration
        
        # Make snap
        image = self.camera_handle.snap()
        
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
        self.pipettetip = [xref + x2, yref + y2]
        
        # Return pipette tip coordinates
        logging.info('Pipette tip detected @ (x,y) = (%f,%f)' % (self.pipettetip[0], self.pipettetip[1]))
        
        self.finished.emit()
    
    @pyqtSlot()
    def autofocus_pipette(self):
        """ Focus pipette tip from above.
        
        This method bring the pipette tip into focus by approaching the focal
        plane from above using the micromanipulator only. We start by moving
        the pipette up to check the position relative to the focal plane as a
        safety measure. Furthermore, we assume a pre-calibrated coordinate
        system and the pipette tip in the FOV center.
        """
        logging.info('autofocus started')
        self.micromanipulator_handle.setZero()
        
        # Parameters to vary
        focusprecision = 10     # focal plane finding precision (micrometers/100)
        optimalstepsize = 1000  # peak recognizing step size (micrometers/100)
        margin = 0.96           # threshold for max values (percentage)
        
        # Get pipette tip position
        [xpos, ypos] = self.detect_pipette_tip()
        
        # Construct Gaussian window
        I = self.camera_handle.snap()
        window = PipetteAutofocus.comp_Gaussian_kernel(size=I.shape[1], fwhm=I.shape[1]/4, center=[xpos,ypos])
        
        # Set reference pipette position as origin
        # self.micromanipulator_instance.setZero()
        # reference = self.micromanipulator_instance.getPos()[2]
        reference = self.micromanipulator_handle.camcoords[2]
        
        # Initialize array for storing [positions; penalties]
        positionhistory = np.zeros(3)
        penaltyhistory = np.zeros(3)
        
        """"Step up three times to compute penalties [p1,p2,p3]"""
        logging.info('Step up two times')
        penalties = np.zeros(3)
        for i in range(3):
            # Capture image
            I = self.camera_handle.snap()
            io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
            
            # Apply image window
            IW = I * window
            
            # Calculate out of focus penalty score
            penalties[i] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
            
            # Save position and penalty in history
            positionhistory[i] = self.micromanipulator_handle.camcoords[2]
            penaltyhistory[i] = penalties[i]
            
            # Move pipette up
            if i < 2:
                print('step up')
                # self.micromanipulator_instance.moveAbsZ(reference+(i+1)*optimalstepsize)
                self.micromanipulator_handle.moveRel(0,0,optimalstepsize)
            
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
                
            logging.info(pinbool)
            logging.info(reference)
            
            # Move micromanipulator towards local maximum through (7 modes)
            if np.array_equal(pinbool, [0,1,0]):
                pass
            elif np.array_equal(pinbool, [1,0,1]):
                self.micromanipulator_handle.moveAbsZ(reference-stepsize)
                I = self.camera_handle.snap()
                io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                IW = I * window
                penalties[1] = penalties[0]
                penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                positionhistory = np.append(positionhistory, self.micromanipulator_handle.camcoords[2])
                penaltyhistory = np.append(penaltyhistory, penalties[0])
                reference = reference - stepsize
            elif np.array_equal(pinbool, [1,0,0]):
                if stepsize > optimalstepsize:
                    stepsize = stepsize/2
                    penalties[1] = penalties[0]
                    self.micromanipulator_handle.moveAbsZ(reference+stepsize)
                    I = self.camera_handle.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_handle.camcoords[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    self.micromanipulator_handle.moveAbsZ(reference-stepsize)
                    I = self.camera_handle.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_handle.camcoords[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
                elif stepsize < optimalstepsize:
                    stepsize = stepsize*2
                    penalties[1] = penalties[0]
                    self.micromanipulator_handle.moveAbsZ(reference-stepsize)
                    I = self.camera_handle.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_handle.camcoords[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
                else:
                    penalties = np.roll(penalties,1)
                    self.micromanipulator_handle.moveAbsZ(reference-stepsize)
                    I = self.camera_handle.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_handle.camcoords[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
            elif np.array_equal(pinbool, [0,0,1]):
                if stepsize > optimalstepsize:
                    stepsize = stepsize/2
                    penalties[1] = penalties[2]
                    self.micromanipulator_handle.moveAbsZ(reference+3*stepsize)
                    I = self.camera_handle.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_handle.camcoords[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    self.micromanipulator_handle.moveAbsZ(reference+5*stepsize)
                    I = self.camera_handle.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_handle.camcoords[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + 3*stepsize
                elif stepsize < optimalstepsize:
                    stepsize = stepsize*2
                    penalties[1] = penalties[2]
                    self.micromanipulator_handle.moveAbsZ(reference+2*stepsize)
                    I = self.camera_handle.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_handle.camcoords[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference
                else:
                    penalties = np.roll(penalties,-1)
                    self.micromanipulator_handle.moveAbsZ(reference+3*stepsize)
                    I = self.camera_handle.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_handle.camcoords[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + stepsize
            elif np.array_equal(pinbool, [1,1,0]):
                if stepsize == stepsizemin:
                    penalties = np.roll(penalties,1)
                    self.micromanipulator_handle.moveAbsZ(reference-stepsize)
                    I = self.camera_handle.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_handle.camcoords[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
                else:
                    stepsize = stepsize/2
                    penalties[1] = penalties[0]
                    self.micromanipulator_handle.moveAbsZ(reference+stepsize)
                    I = self.camera_handle.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_handle.camcoords[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    self.micromanipulator_handle.moveAbsZ(reference-stepsize)
                    I = self.camera_handle.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_handle.camcoords[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
            elif np.array_equal(pinbool, [0,1,1]):
                if stepsize == stepsizemin:
                    penalties = np.roll(penalties,-1)
                    self.micromanipulator_handle.moveAbsZ(reference+3*stepsize)
                    I = self.camera_handle.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_handle.camcoords[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + stepsize
                else:
                    stepsize = stepsize/2
                    penalties[1] = penalties[2]
                    self.micromanipulator_handle.moveAbsZ(reference+3*stepsize)
                    I = self.camera_handle.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_handle.camcoords[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    self.micromanipulator_handle.moveAbsZ(reference+5*stepsize)
                    I = self.camera_handle.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[2] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_handle.camcoords[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[2])
                    reference = reference + 3*stepsize
            elif np.array_equal(pinbool, [1,1,1]):
                if stepsize == stepsizemax:
                    penalties = np.roll(penalties,1)
                    self.micromanipulator_handle.moveAbsZ(reference-stepsize)
                    I = self.camera_handle.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_handle.camcoords[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
                else:
                    stepsize = 2*stepsize
                    self.micromanipulator_handle.moveAbsZ(reference-stepsize)
                    I = self.camera_handle.snap()
                    io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                    IW = I * window
                    penalties[1] = penalties[0]
                    penalties[0] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                    positionhistory = np.append(positionhistory, self.micromanipulator_handle.camcoords[2])
                    penaltyhistory = np.append(penaltyhistory, penalties[0])
                    reference = reference - stepsize
        
        """Final approach by sampling between the zeros in [0,1,0]"""
        print('Coarse focus found, continue with finetuning')
        while stepsize > focusprecision:
            # Sample ten points between outer penalty values
            penalties = np.zeros(11)
            positions = np.linspace(reference, reference+2*stepsize, 11)
            for idx, pos in enumerate(positions):
                self.micromanipulator_handle.moveAbsZ(pos)
                I = self.camera_handle.snap()
                io.imsave(self.savedirectory+str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))+'.tif', I, check_contrast=False)
                IW = I * window
                penalties[idx] = PipetteAutofocus.comp_variance_of_Laplacian(IW)
                positionhistory = np.append(positionhistory, self.micromanipulator_handle.camcoords[2])
                penaltyhistory = np.append(penaltyhistory, penalties[idx])
            
            # Locate maximum penalty value
            i_max = penalties.argmax()
            
            # Decrease step size
            stepsize = stepsize/5
            
            # Set reference position one step below maximum penalty position
            reference = positions[i_max] - stepsize
        
        logging.info('Focus offset found!')
        np.save(self.savedirectory+'penaltyhistory.txt', penaltyhistory)
        np.save(self.savedirectory+'positionhistory.txt', positionhistory)
        self.finished.emit()
    
    @pyqtSlot()
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
        manipulator_position_absolute = self.micromanipulator_handle.camcoords
        step2pos = lambda pos: np.add(manipulator_position_absolute, pos)
        
        
        for idx, step in enumerate([[xstep,0,0], [0,ystep,0], [0,0,zstep]]):
            v = np.empty(numsteps)
            w = np.empty(numsteps)
            
            # Detect pipette tip and move micromanipulator
            for i in numsteps:
                v[i], w[i] = self.detect_pipette_tip()
                if i < numsteps:
                    self.micromanipulator_handle.moveAbs(step2pos(np.multiply(step,(i+1))))
                else:
                    self.micromanipulator_handle.moveAbs(step2pos([0,0,0]))
            
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
        self.micromanipulator_handle.constructrotationmatrix(0, 0, gamma)
        Ey_gammarotated = self.micromanipulator_handle.Rinv.dot(Ey)
        if Ey_gammarotated[1] < 0:
            gamma = gamma - np.pi
            self.micromanipulator_handle.constructrotationmatrix(0, 0, gamma)
        
        # Calculate rotation angles: alpha, beta
        alpha = np.arcsin(Ez[0]*np.sin(gamma) + Ez[1]*np.cos(gamma))
        beta = (-Ez[0]*np.cos(gamma) + Ez[1]*np.sin(gamma))/np.cos(alpha)
        
        # Apply full rotation matrix and verify Ey maps to y-axis
        self.micromanipulator_handle.constructrotationmatrix(alpha, beta, gamma)
        Ey_rotated = self.micromanipulator_handle.Rinv.dot(Ey)
        logging.info(Ey_rotated)
        
        self.finished.emit()


