# -*- coding: utf-8 -*-
"""
Created on Thu Apr 15 11:12:26 2021

@author: tvdrb
"""

import os
import datetime
import logging
import numpy as np

from skimage import io
from PyQt5.QtCore import pyqtSignal, pyqtSlot, QThread

if __name__ == "__main__":
    os.chdir(os.getcwd() + '\\..')
from PatchClamp.ImageProcessing_AutoPatch import PipetteTipDetector, PipetteAutofocus


class AutoPatchThread(QThread):
    finished = pyqtSignal()
    sketches = pyqtSignal(list)
    # drawings = pyqtSignal(tuple)
    crosshair = pyqtSignal(np.ndarray)
    drawsignal = pyqtSignal(tuple)
    
    def __init__(self, camera_handle=None, manipulator_handle=None, objective_handle=None):
        # Class attributes
        self.camera_handle = camera_handle
        self.micromanipulator_handle = manipulator_handle
        self.objective_handle = objective_handle
        self.pipettetip = [np.nan, np.nan]
        self.savedirectory = r'M:\tnw\ist\do\projects\Neurophotonics\Brinkslab\Data\Thijs\Save directory\\'
        
        # QThread attributes
        super().__init__()
        self.isrunning = False
        self.moveToThread(self)
        self.finished.connect(self.stop)
        self.started.connect(self.stop)
    
    def __del__(self):
        """
        Before we delete the autopatch backend we need to: interrupt every
        algorithm that is running as soon as possible, stop active hardware, 
        disconnect inactive hardware, and quit the thread.
        """
        self.isrunning = False
        if self.micromanipulator_handle != None:
            self.micromanipulator_handle.stop()
            self.micromanipulator_handle.close()
        self.quit()
        self.wait()
        
    def stop(self):
        """
        To make sure the algorithm and micromanipulator stop, we reset the
        button 'isrunning' to False and tell the micromanipulator to be idle.
        Afterwards, we wait for the thread to quit so it can be reused.
        """
        self.isrunning = False
        if self.micromanipulator_handle != None:
            self.micromanipulator_handle.stop()
        self.quit()
        self.wait()
        
    def request(self, slot):
        """
        Request forms the bridget between frontend and backend. A button press
        event triggers connecting to one of the pyqtSlots and starts the thread.
        """
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
    def detect_pipette_tip(self, emit_when_finished=True):
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
        self.sketches.emit(cropped_image)
        
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
        logging.info('Pipette tip detected @ (x,y) = (%f,%f)' % (xref + x2, yref + y2))
        
        self.sketches.emit(image)
        self.crosshair.emit(np.array([xref + x2, yref + y2]))
        if emit_when_finished:
            self.finished.emit()
    
    @pyqtSlot()
    def autofocus_pipette(self, emit_when_finished=True):
        """ Focus pipette tip from above.
        
        This method bring the pipette tip into focus by approaching the focal
        plane from above using the micromanipulator only. We start by moving
        the pipette up to check the position relative to the focal plane as a
        safety measure. Furthermore, we assume a pre-calibrated coordinate
        system and the pipette tip in the FOV center.
        """
        
        # Parameters to vary
        focusprecision = 0.1    # focal plane finding precision (micrometers/100)
        optimalstepsize = 10    # peak recognizing step size (micrometers/100)
        margin = 0.96           # threshold for max values (percentage)
        
        # Get pipette tip position
        self.detect_pipette_tip(emit_when_finished=False)
        
        # Construct Gaussian window around pipete tip
        I = self.camera_handle.snap()
        window = PipetteAutofocus.comp_Gaussian_kernel(size=I.shape[1], fwhm=I.shape[1]/4, center=self.pipettetip)
        
        # Emit Gaussian windowed field of view
        IW = I * window
        self.sketches.emit(IW)
        
        # Set reference pipette position as origin
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
                self.micromanipulator_handle.moveRel(0,0,optimalstepsize)
            
        """Iteratively find peak in focus penalty values"""
        stepsize = optimalstepsize
        stepsizemin = optimalstepsize/2
        stepsizemax = optimalstepsize*8
        pinbool = np.zeros(3)
        while not np.array_equal(pinbool, [0,1,0]) & self.isrunning:
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
        logging.info('Coarse focus found, continue with finetuning')
        while stepsize > focusprecision & self.isrunning:
            # Sample ten points between outer penalty values
            penalties = np.zeros(6)
            positions = np.linspace(reference, reference+2*stepsize, 6)
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
        np.savetxt(self.savedirectory+'penaltyhistory.txt', penaltyhistory)
        np.savetxt(self.savedirectory+'positionhistory.txt', positionhistory)
        
        self.finished.emit()
    
    @pyqtSlot()
    def calibrate_coordsys(self, emit_when_finished=True):
        """ Calibrate PatchStar coordinate system.
        
        This method returns the rotation angles that align the coordinate-
        systems of the PatchStar with the camera. We assume the z-axis of the
        PatchStar always points up.
        """
        # Specify the number of steps and their size for slope detection
        numsteps = 10
        xstep = 5           # in micrometer
        ystep = 5           # in micrometer
        zstep = 5           # in micrometer
        
        # Set the reference position of the micromanipulator
        reference = self.micromanipulator_handle.manipcoords
        
        for idx, step in enumerate([[xstep,0,0], [0,ystep,0], [0,0,zstep]]):
            v = np.empty(numsteps)
            w = np.empty(numsteps)
            
            # Detect pipette tip and move micromanipulator
            for i in range(numsteps):
                self.detect_pipette_tip(emit_when_finished=False)
                v[i], w[i] = self.pipettetip
                if i < numsteps:
                    self.micromanipulator_handle.moveRel(step[0], step[1], step[2])
                else:
                    self.micromanipulator_handle.moveAbs(reference[0], reference[1], reference[2])
            
            # Average array of pipette coordinates to get mean [dx,dy] per step
            if idx == 0:
                logging.info('Calibrated X-axis')
                Ex = [np.nanmean(v)/xstep, np.nanmean(w)/xstep, 0]
                self.drawsignal.emit(np.array(reference, Ex))
            elif idx == 1:
                logging.info('Calibrated Y-axis')
                Ey = [np.nanmean(v)/ystep, np.nanmean(w)/ystep, 0]
                self.drawsignal.emit(np.array(reference, Ey))
            elif idx == 2:
                logging.info('Calibrated Z-axis')
                Ez = [np.nanmean(v)/zstep, np.nanmean(w)/zstep, 0]
                self.drawsignal.emit(np.array(reference, Ez))
        
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
        
        # Construct full rotation matrix
        [R, Rinv] = self.micromanipulator_handle.constructrotationmatrix(alpha, beta, gamma)
        
        # Apply rotation matrix to the rotation matrix that is already there
        self.micromanipulator_handle.R = R @ self.micromanipulator_handle.R # Is this order correct?
        self.micromanipulator_handle.Rinv = Rinv @ self.micromanipulator_handle.Rinv # Is this order correct?
        
        self.finished.emit()


