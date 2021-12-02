# -*- coding: utf-8 -*-
"""
Created on Wed Aug 11 15:15:30 2021

@author: TvdrBurgt
"""


from datetime import datetime
import time
import logging
import numpy as np
from skimage import io
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot

from PatchClamp.ImageProcessing_patchclamp import PatchClampImageProcessing as ia


class Worker(QObject):
    draw = pyqtSignal(list)
    graph = pyqtSignal(np.ndarray)
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
        self.graph.emit(np.array([1,2,3]))
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
        self._parent.pipette_coordinates_pair = np.vstack([reference, np.array([tipcoord[0], tipcoord[1], None])])
        
        self.finished.emit()
        
    
    @pyqtSlot()
    def autofocus_tip(self):
        """ Autofocus pipette tip iteratively moves the pipette up or down
        untill the tip is in focus.
        
        The algorithm uses the variance of the Laplacian as its sharpness
        function. The sharpness values are high for images with sharp edges and
        low for images with little sharp edges. We use this phenomenon to find
        the maximum of the sharpness function, the image contains sharp lines
        when the pipette is in-focus. Note that the sharpness values go up when
        the pipette moves through the focal plane, thereby creating more sharp
        edges as a result of pipette geometry and light bending. However, the
        sharpness function dips a little when the pipette tip is in focus. The
        autofocus algorithm here is designed to stop at the local maximum
        before the dip. Another advantage is that this dip is present at a
        constant height above the focal plance, this allows us to move past the
        the peak without pressing the pipette down into the sample/coverslip.
        
        The *focusbias* is around 10-30 micrometers!!!
        """
        # get all relevant parent attributes
        save_directory = self._parent.save_directory
        micromanipulator = self._parent.micromanipulator
        camera = self._parent.camerathread
        
        # algorithm variables
        stepsize = 10
        min_taillength = 10
        
        reference = micromanipulator.getPos()
        penaltyhistory = np.array([])
        positionhistory = np.array([])
        
        #I) fill first three sharpness scores towards the tail of the graph
        pen = np.zeros(3)
        pos = np.zeros(3)
        for i in range(0,3):
            micromanipulator.moveRel(dz=+stepsize)
            I = camera.snap()
            pen[i] = ia.comp_variance_of_Laplacian(I)
            pos[i] = reference[2] + (i+1)*stepsize
        penaltyhistory = np.append(penaltyhistory, pen)
        positionhistory = np.append(positionhistory, pos)
        
        going_up = True
        going_down = not going_up
        lookingforpeak = True
        while lookingforpeak:
            
            # emit graph
            self.graph.emit(np.vstack([positionhistory,penaltyhistory]))
            
            #II) check which side of the sharpness graph to extend
            move = None
            if going_up:
                pen = penaltyhistory[-3::]
            else:
                pen = penaltyhistory[0:3]
            
            #III) check where maximum penalty score is: left, middle, right
            if np.argmax(pen) == 0:
                maximum = 'left'
            elif np.argmax(pen) == 1:
                maximum = 'middle'
            elif np.argmax(pen) == 2:
                maximum = 'right'
            
            #IVa) possible actions to undertake while going up
            if maximum == 'right' and going_up:
                move = 'step up'
            elif maximum == 'left' and going_up:
                going_up = False
                going_down = True
            elif maximum == 'middle' and going_up:
                if pen[1] == np.max(penaltyhistory):
                    pos = positionhistory[-1]
                    micromanipulator.moveAbs(x=reference[0], y=reference[1], z=pos)
                    penaltytail = pen[1::]
                    for i in range(2, min_taillength):
                        micromanipulator.moveRel(dz=+stepsize)
                        I = camera.snap()
                        penalty = ia.comp_variance_of_Laplacian(I)
                        penaltytail = np.append(penaltytail, penalty)
                    monotonicity_condition = np.all(np.diff(penaltytail) <= 0)
                    if monotonicity_condition:
                        logging.info("Detected maximum is a sharpness peak!")
                        lookingforpeak = False
                        move = None
                        micromanipulator.moveAbs(x=reference[0], y=reference[1], z=positionhistory[-2])
                    else:
                        logging.info("Detected maximum is noise")
                        going_up = False
                        going_down = True
                else:
                    going_up = False
                    going_down = True
            
            #IVb) possible actions to undertake while going down
            elif maximum == 'left' and going_down:
                move = 'step down'
            elif maximum == 'right' and going_down:
                if pen[2] == np.max(penaltyhistory):
                    pos = positionhistory[2]
                    micromanipulator.moveAbs(x=reference[0], y=reference[1], z=pos)
                    penaltytail = pen[2]
                    for i in range(2, min_taillength):
                        micromanipulator.moveRel(dz=+stepsize)
                        I = camera.snap()
                        penalty = ia.comp_variance_of_Laplacian(I)
                        penaltytail = np.append(penaltytail, penalty)
                    monotonicity_condition = np.all(np.diff(penaltytail) <= 0)
                    if monotonicity_condition:
                        logging.info("Detected maximum is a sharpness peak!")
                        lookingforpeak = False
                        move = None
                        micromanipulator.moveAbs(x=reference[0], y=reference[1], z=positionhistory[2])
                    else:
                        logging.info("Detected maximum is noise")
                        move = 'step down'
                else:
                    move = 'step down'
            elif maximum == 'middle' and going_down:
                penaltytail = penaltyhistory[1::]
                taillength = len(penaltytail)
                pos = positionhistory[-1]
                if taillength < min_taillength:
                    micromanipulator.moveAbs(x=reference[0], y=reference[1], z=pos)
                    for i in range(taillength, min_taillength):
                        micromanipulator.moveRel(dz=+stepsize)
                        I = camera.snap()
                        penalty = ia.comp_variance_of_Laplacian(I)
                        penaltytail = np.append(penaltytail, penalty)
                else:
                    penaltytail = penaltytail[0:min_taillength]
                monotonicity_condition = np.all(np.diff(penaltytail) <= 0)
                if monotonicity_condition:
                    logging.info("Detected maximum is a sharpness peak!")
                    lookingforpeak = False
                    move = None
                    micromanipulator.moveAbs(x=reference[0], y=reference[1], z=positionhistory[1])
                else:
                    logging.info("Detected maximum is noise")
                    move = 'step down'
        
            #V) extend the sharpness function on either side
            if move == 'step up':
                pos = positionhistory[-1] + stepsize
                micromanipulator.moveAbs(x=reference[0], y=reference[1], z=pos)
                I = camera.snap()
                pen = ia.comp_variance_of_Laplacian(I)
                penaltyhistory = np.append(penaltyhistory, pen)
                positionhistory = np.append(positionhistory, pos)
            elif move == 'step down':
                pos = positionhistory[0] - stepsize
                micromanipulator.moveAbs(x=reference[0], y=reference[1], z=pos)
                I = camera.snap()
                pen = ia.comp_variance_of_Laplacian(I)
                penaltyhistory = np.append(pen, penaltyhistory)
                positionhistory = np.append(pos, positionhistory)
        
        timestamp = str(datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))                   #FLAG: relevant for MSc thesis
        np.save(save_directory+'autofocus_positionhistory_'+timestamp, positionhistory) #FLAG: relevant for MSc thesis
        np.save(save_directory+'autofocus_penaltyhistory_'+timestamp, penaltyhistory)   #FLAG: relevant for MSc thesis
        
        #VI) continue with finding the fine focus position
        logging.info('Coarse focus found, continue with finetuning')
        _,_,z = micromanipulator.getPos()
        for step in [stepsize, stepsize/5, stepsize/25]:
            # Sample six points between the last three penalty scores
            penalties = np.zeros(6)
            positions = np.linspace(z-step, z+step, 6)
            for idx, pos in enumerate(positions):
                micromanipulator.moveAbs(x=reference[0], y=reference[1], z=pos)
                I = camera.snap()
                penalties[idx] = ia.comp_variance_of_Laplacian(I)
                positionhistory = np.append(positionhistory, pos)
                penaltyhistory = np.append(penaltyhistory, penalties[idx])
            
            # Locate maximum penalty value
            i_max = np.argmax(penalties)
            z = positions[i_max]
            
            # emit graph
            self.graph.emit(np.vstack([positionhistory,penaltyhistory]))
        
        #VII) correct for bias in focus height and move pipette into focus
        foundfocus = z
        micromanipulator.moveAbs(x=reference[0], y=reference[1], z=foundfocus)
        
        I = camera.snap()                                                                   #FLAG: relevant for MSc thesis
        io.imsave(save_directory+'autofocus_'+timestamp+'.tif', I, check_contrast=False)    #FLAG: relevant for MSc thesis
        np.save(save_directory+'autofocus_positionhistory_'+timestamp, positionhistory)     #FLAG: relevant for MSc thesis
        np.save(save_directory+'autofocus_penaltyhistory_'+timestamp, penaltyhistory)       #FLAG: relevant for MSc thesis
        
        self.finished.emit()
    
    
    @pyqtSlot()
    def target2center(self):
        """ Target to center moves the XY stage so that the user-selected
        target ends up in the center of the camera field-of-view.
        
        The camera field-of-view is turned 90 degrees counter-clockwise 
        compared to the ludl XY sample stage.
        """
        stage = self._parent.XYstage
        pixelsize = self._parent.pixel_size
        width,height = self._parent.image_size
        xtarget,ytarget,_ = self._parent.target_coordinates
        
        # Calculate image center pixels
        xcenter,ycenter = int(width/2),int(height/2)
        
        # Caculate path to travel from target to center
        dx_pi = (xtarget - xcenter)    #in pixels
        dy_pi = (ytarget - ycenter)    #in pixels
        
        # Convert pixels to ludl indices (1577 ludl index = 340 um)
        dx = int(dx_pi * pixelsize/1000 * 1577/340)
        dy = int(dy_pi * pixelsize/1000 * 1577/340)
        
        # Move XY stage a distance (-dx,-dy), note that stage axis are rotated
        ismoving = True
        stage.moveRel(xRel=dy, yRel=-dx)
        while ismoving:
            ismoving = not stage.motorsStopped()
            time.sleep(0.1)
        
        # Update target coordinates in the backend
        self.parent.target_coordinates = np.array([xcenter,ycenter,None])
        self.draw.emit(['target', dx_pi, dy_pi])
        
        self.finished.emit()
    
    
    @pyqtSlot()
    def formgigaseal(self):
        """ Form Gigaseal brings the pipette to the target cell and forms a 
        gigaseal with the membrane.
        
        I) Calculate trajectory and bring pipette tip above the target cell.
        II) Adjust pipette pressure to small overpressure.
        III) Pipette tip descent until resistance increases slightly.
        IV) Release pressure, possibly apply light suction, to form Gigaseal
        
        Safety measures in place:
            <!>     Pipette descent range: <50 microns so pipette does not
                    penatrate the petridish.
            <!>     Resistance check: Resistance rises when pipette tip is
                    blocked by a cell or the petridish, a broken tip results
                    in an abrupt drop in resistance after which we stop
                    pipette descent immediately.
        """
        save_directory = self._parent.save_directory
        micromanipulator = self._parent.micromanipulator
        pressurecontroller = self._parent.pressurethread
        sealtestthread = self._parent.sealtestthread
        account4rotation = self._parent.account4rotation
        pixelsize = self._parent.pixel_size
        tipcoords_manip,tipcoords_cam = self._parent.pipette_coordinates_pair
        xtarget,ytarget,_ = self._parent.target_coordinates
        
        # Safety measures in place
        PIPETTE_DESCENT_RANGE = 50  # microns
        RESISTANCE_DROP = 0.1       # drop-ratio
        # Algorithm variables
        R_INCREASE = 0.3e6          # MΩ
        STEPSIZE = 0.1              # microns
        TIMEOUT = 30                # seconds
        
        #Ia) calculate shortest trajectory to target and apply coordinate transformation
        micromanipulator.moveAbs(x=tipcoords_manip[0], y=tipcoords_manip[1], z=tipcoords_manip[2])
        dx = xtarget - tipcoords_cam[0]                     #x trajectory (in pixels)
        dy = ytarget - tipcoords_cam[1]                     #y trajectory (in pixels)
        trajectory = np.array([dx,dy,0])*pixelsize/1000     #trajectory (in microns)
        trajectory = account4rotation(origin=tipcoords_manip, target=tipcoords_manip+trajectory)
        
        #Ib) manoeuvre the micromanipulator above the target
        micromanipulator.moveRel(dx=trajectory[0], dy=trajectory[1], dz=trajectory[2])
        
        #II) apply small pressure to the pipette for final approach
        pressurecontroller.set_pressure(30)
        resistance_reference = np.nanmean(self._parent.resistance)
        
        #III) descent pipette with one micron at a time until R increases
        logging.info("Tip descent started...")
        resistance = resistance_reference
        position = micromanipulator.getPos()[2]
        resistancehistory = np.array([resistance])
        positionhistory = np.array([position])
        while resistance < resistance_reference + R_INCREASE:
            # step down and measure the resistance
            micromanipulator.moveRel(dx=0, dy=0, dz=-STEPSIZE)
            position = micromanipulator.getPos()[2]
            time.sleep(0.01)
            resistance = np.nanmean(self._parent.resistance[-10::])
            self.graph.emit(resistancehistory)
            
            # safety checks on maximum descent range and resistance drop
            if resistance < np.nanmax(resistancehistory)*RESISTANCE_DROP:
                break
            else:
                resistancehistory = np.append(resistancehistory, resistance)
            if positionhistory[-1]-positionhistory[0] >= PIPETTE_DESCENT_RANGE:
                break
            else:
                positionhistory = np.append(positionhistory, position)
            
        
        #IVa) set pressure to ATM
        pressurecontroller.set_pressure(0)
        
        #IVb) wait for Gigaseal
        logging.info("Attempting gigaseas...")
        start = time.time()
        while resistance < 1e9 and time.time()-start < TIMEOUT:
            resistance = np.nanmax(self._parent.resistance[-10::])
            self.graph.emit(resistancehistory)
            resistancehistory = np.append(resistancehistory, resistance)
            time.sleep(0.1)
        
        #IVc) wait for Gigaseal with suction pulses
        if resistance > 1e9:
            logging.info("Gigaseal formed!")
        else:
            logging.info("Timeout reached, starting suction pulses...")
            pressurecontroller.set_waveform(high=-10, low=-30, high_T=2, low_T=0.5)
            while resistance < 1e9 and time.time()-start < TIMEOUT:
                resistance = np.nanmax(self._parent.resistance[-10::])
                self.graph.emit(resistancehistory)
                resistancehistory = np.append(resistancehistory, resistance)
                time.sleep(0.1)
            del pressurecontroller.waveform
            pressurecontroller.set_pressure(0)
        
        timestamp = str(datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))   
        np.save(save_directory+'gigaseal_positionhistory_'+timestamp, positionhistory)      #FLAG: relevant for MSc thesis
        np.save(save_directory+'gigaseal_resistancehistory_'+timestamp, resistancehistory)  #FLAG: relevant for MSc thesis
        
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

