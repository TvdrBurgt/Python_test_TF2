# -*- coding: utf-8 -*-
"""
Created on Fri Apr 16 11:56:47 2021

@author: tvdrb
"""

import numpy as np
from skimage import filters, feature, transform


class PipetteTipDetector:
    
    def locate_tip(I, upper_angle, lower_angle, diameter, blursize=15,
                   angle_range=10, num_angles=5000, num_peaks=8):
        """ Tip detection algorithm
        
        input parameters:
            blursize    = kernel size for gaussian blur
            angle_range = angle search range (in degree)
            num_angles  = number of sampling angles in Hough transform
            num_peaks   = number of peaks to find in Hough space
            plotflag    = if True it will generate figures
        output parameters:
            xpos        = x position of the pipette tip
            ypos        = y position of the pipette tip
        """
        
        # Gaussian blur
        print('I)\t Gaussian blurring...')
        IB = filters.gaussian(I, blursize)
        
        # Canny edge detection
        print('II)\t Canny edge detection...')
        BW = feature.canny(IB, sigma=10, low_threshold=0.9, high_threshold=0.7, use_quantiles=True)
        
        # Double-sided Hough transform
        print('III) Calculating Hough transform...')
        if np.abs(upper_angle-lower_angle) < angle_range:
            theta1 = upper_angle + np.linspace(angle_range/2, -np.abs(upper_angle-lower_angle)/2, num_angles)
            theta2 = lower_angle + np.linspace(np.abs(upper_angle-lower_angle)/2, -angle_range/2, num_angles)
        else:
            theta1 = upper_angle + np.linspace(angle_range/2, -angle_range/2, num_angles)
            theta2 = lower_angle + np.linspace(angle_range/2, -angle_range/2, num_angles)
        # append theta's and transform to radians
        theta = np.deg2rad(np.append(theta1,theta2))
        # calculate Hough transform
        H, T, R = transform.hough_line(BW,theta)
        # split Hough transform in two because of two angles
        H1, H2 = np.hsplit(H,2)
        T1, T2 = np.hsplit(T,2)
        
        # extract most common lines in the image from the double-sided Hough transform
        print('IV)\t Finding most common lines from Hough transform...')
        _, Tcommon1, Rcommon1 = transform.hough_line_peaks(H1,T1,R,num_peaks=num_peaks,threshold=0)
        _, Tcommon2, Rcommon2 = transform.hough_line_peaks(H2,T2,R,num_peaks=num_peaks,threshold=0)
        # find the average value so we end up with two lines
        angle1 = np.mean(Tcommon1)
        dist1 = np.mean(Rcommon1)
        angle2 = np.mean(Tcommon2)
        dist2 = np.mean(Rcommon2)
        
        # find intersection between X1*cos(T1)+Y1*sin(T1)=R1 and X2*cos(T2)+Y2*sin(T2)=R2
        print('V)\t Calculating preliminary pipette point...')
        LHS = np.array([[np.cos(angle1), np.sin(angle1)], [np.cos(angle2), np.sin(angle2)]])
        RHS = np.array([dist1, dist2])
        xpos, ypos = np.linalg.solve(LHS, RHS)
        
        # account for xposition overestimation bias
        print('VI)\t Correcting for pipette diameter...')
        deltax = (diameter*np.sin((angle1+angle2)/2))/(2*np.tan((angle1-angle2)/2))
        deltay = -(diameter*np.cos((angle1+angle2)/2))/(2*np.tan((angle1-angle2)/2))
        xpos = xpos - deltax
        ypos = ypos - deltay    
        print('dx = %.2f' % (deltax))
        print('dy = %.2f' % (deltay))
        
        return xpos, ypos
    
    def crop_image(I,xpos,ypos,xsize=600,ysize=150):
        """
        This functions crops the input figure around the coordinates from the
        first tip detection algorithm.
        """
        
        # round pipette coordinates to integers
        xpos = np.round(xpos)
        ypos = np.round(ypos)
        
        # specify cropping region
        left = int(xpos-xsize/2)
        right = int(xpos+xsize/2)
        up = int(ypos-ysize/2)
        down = int(ypos+ysize/2)
        
        # check if the cropping exceeds image boundaries
        faultylocalisation = False
        height = I.shape[0]
        width = I.shape[1]
        if left < 0:
            left = 0
            if right < 0:
                right = xsize
                faultylocalisation = True
        if right > width-1:
            right = width-1
            if left > width-1:
                left = width-1 - xsize
                faultylocalisation = True
        if up < 0:
            up = 0
            if down < 0:
                down = ysize
                faultylocalisation = True
        if down > height-1:
            down = height-1
            if up > height-1:
                up = height-1 - ysize
                faultylocalisation = True
        
        # crop image
        Icropped = I[up:down,left:right]
        
        # reference point (x0,y0) is (0,0) in the cropped image
        x0 = left
        y0 = up
        
        return Icropped, x0, y0, faultylocalisation

