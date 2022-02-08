# -*- coding: utf-8 -*-
"""
Created on Mon Sep 27 15:48:59 2021

@author: lhuismans, TvdrBurgt

"""


import sys
import logging
import numpy as np
from PyQt5.QtCore import pyqtSignal, QThread, pyqtSlot

import nidaqmx
from nidaqmx.stream_readers import AnalogMultiChannelReader
from nidaqmx.stream_writers import AnalogSingleChannelWriter

sys.path.append("../")
from NIDAQ.wavegenerator import blockWave
from NIDAQ.constants import MeasurementConstants, NiDaqChannels


class SealTestThread(QThread):
    """Class for doing a patchclamp seal test. A continuous measurement can be done
    of which the data is returned continuously as well.
    We want to use the nidaqmx.task.timing to assign a clock and a conversion rate to the task.
    Then we need to make sure to call the read function regularly to read all the samples automatically generated by the card.

    The measurement for the sealtest does not need to be checked. Only a pop up window in the UI provided asking if the gains
    set in the UI are corresponding to the gains on the patchclamp.
    """
    
    measurement = pyqtSignal(np.ndarray, np.ndarray)  # Voltage, Current
    
    def __init__(self):
        # inherit a QThread to run parallel from the GUI
        super().__init__()
        self.isRunning = False
        self.moveToThread(self)
        self.started.connect(self.measure)
        
        # load frequently used constants
        constants = MeasurementConstants()
        self.configs = NiDaqChannels().look_up_table
        self.sampleRate = constants.patchSealSampRate
        self.frequency = constants.patchSealFreq
        self.voltMin = constants.patchSealMinVol
        self.voltMax = constants.patchSealMaxVol
        self.dutycycle = constants.patchSealDuty
        self.readNumber = 100 # should be a multiple of the wavelength
        
        # look up channel configuration of NIDAQ
        self.configs = NiDaqChannels().look_up_table
        self.patchVoltOutChan = None
        self.patchCurOutChan = None
        self.patchVoltInChan = None
        
        # generate wave function from loaded constants
        self.wave = np.zeros(self.readNumber)
        self.setWave(1,0,0)
        
        # operation mode
        self.mode = 'voltageclamp'
    
    
    @property
    def mode(self):
        return self._mode
    
    @mode.setter
    def mode(self, mode):
        self._mode = mode
        print("Sealtestthread operation mode: "+mode)
    
    
    def stop(self):
        self.isRunning = False
        self.quit()
        self.wait()
    
    def setWave(self, inVolGain, diffvoltage, lowervoltage):
        self.voltMin = lowervoltage / inVolGain
        self.voltMax = self.voltMin + diffvoltage / inVolGain

        self.wave = blockWave(self.sampleRate, self.frequency, self.voltMin, self.voltMax, self.dutycycle)
    
    def setTiming(self, writeTask, readTask):
        # Check if they are on the same device
        readDev = self.patchCurOutChan.split("/")[0]
        writeDev = self.patchVoltInChan.split("/")[0]

        # Check if they are on the same device
        if readDev == writeDev:
            writeClock = ("/" + self.patchCurOutChan.split("/")[0] + "/ai/SampleClock")  # Getting the device and its sampleClock
        elif (readDev == self.configs["clock1Channel"].split("/")[1]):  # Checking if readTask is on same device as clock1.
            readTask.export_signals.samp_clk_output_term = self.configs["clock1Channel"]
            writeClock = self.configs["clock2Channel"]
        elif readDev == self.configs["clock2Channel"].split("/")[1]:
            readTask.export_signals.samp_clk_output_term = self.configs["clock2Channel"]
            writeClock = self.configs["clock1Channel"]
        else:
            assert True, "No corresponding clocks defined"
    
        if self.mode == 'voltageclamp':
            readTask.timing.cfg_samp_clk_timing(
                rate=self.sampleRate,
                sample_mode=nidaqmx.constants.AcquisitionType.CONTINUOUS,
                samps_per_chan=self.readNumber,
            )  # Read number is used to determine the buffer size.
            writeTask.timing.cfg_samp_clk_timing(
                rate=self.sampleRate,
                source=writeClock,
                sample_mode=nidaqmx.constants.AcquisitionType.CONTINUOUS,
            )
        elif self.mode == 'zap':
            readTask.timing.cfg_samp_clk_timing(
                rate=self.sampleRate,
                sample_mode=nidaqmx.constants.AcquisitionType.FINITE,
                samps_per_chan=self.sampleNumber,
            )  # Read number is used to determine the buffer size.
            writeTask.timing.cfg_samp_clk_timing(
                rate=self.sampleRate,
                source=writeClock,
                sample_mode=nidaqmx.constants.AcquisitionType.FINITE,
                samps_per_chan=self.sampleNumber,
            )
        else:
            pass
    
    def zap(self):
        # interrupt measurement
        self.stop()
        
        # update attributes for zapping (there should be a function for generating this kind of wave right...?)
        zap_voltage = 1
        zap_voltagegain = 0.1
        zap_duration = 200
        self.sampleRate = 100000
        self.readNumber = 500
        self.voltMax = zap_voltage/zap_voltagegain
        self.voltMin = 0/zap_voltagegain
        self.wave = self.voltMax * np.ones(int(int(zap_duration) * 10 / 100 / 100000 * self.sampleRate))
        self.wave = np.append(self.wave, np.zeros(10))  # give 10*0 at the end of waveform
        self.sampleNumber = len(self.wave)
        
        # change operation mode to zap and execute
        self.mode = 'zap'
        self.start()
        
        # zap cannot be interupted so we press stop and just wait for it to end
        self.stop()
        
        # update the wavefunction for ordinary use and execute
        self.sampleRate = MeasurementConstants().patchSealSampRate
        self.readNumber = 100 # should be a multiple of the wavelength
        self.setWave(0.1, 0.01, 0)
        self.mode = 'voltageclamp'
        
        # resume sealtestthread
        self.start()
    
    @pyqtSlot()
    def measure(self):
        """
        Starts writing a waveform continuously to the patchclamp. While reading
        the buffer periodically.
        """
        self.patchVoltOutChan = self.configs["Vp"]
        self.patchCurOutChan = self.configs["Ip"]
        self.patchVoltInChan = self.configs["patchAO"]

        # DAQ
        with nidaqmx.Task() as writeTask, nidaqmx.Task() as readTask:
            writeTask.ao_channels.add_ao_voltage_chan(self.patchVoltInChan)
            readTask.ai_channels.add_ai_voltage_chan(self.patchVoltOutChan)
            readTask.ai_channels.add_ai_voltage_chan(self.patchCurOutChan)

            self.setTiming(writeTask, readTask)

            reader = AnalogMultiChannelReader(readTask.in_stream)
            writer = AnalogSingleChannelWriter(writeTask.out_stream)

            writer.write_many_sample(self.wave)

            """Reading data from the buffer in a loop. 
            The idea is to let the task read more than could be loaded in the buffer for each iteration.
            This way the task will have to wait slightly longer for incoming samples. And leaves the buffer
            entirely clean. This way we always know the correct numpy size and are always left with an empty
            buffer (and the buffer will not slowly fill up)."""
            if self.mode == 'voltageclamp':
                output = np.zeros([2, self.readNumber])
                writeTask.start()  # Will wait for the readtask to start so it can use its clock
                readTask.start()
                self.isRunning = True
                while self.isRunning:
                    reader.read_many_sample(data=output, number_of_samples_per_channel=self.readNumber)
    
                    # Emiting the data just received as a signal
                    self.measurement.emit(output[0,:], output[1,:])
            elif self.mode == 'zap':
                writeTask.start()  # Will wait for the readtask to start so it can use its clock
                readTask.start()
            else:
                pass
            
            print('sealtest thread stopped')
    



# class SealTestThread(QThread):
#     """ Class for mimiking a patchclamp seal test
#     The structure of this class is comparable to the real seal test thread, but
#     we replace the current and voltage measurement by two numpy random arrays.
#     """
    
#     measurement = pyqtSignal(np.ndarray, np.ndarray)  # Voltage, Current
    
#     def __init__(self):
#         # inherit a QThread to run parallel from the GUI
#         super().__init__()
#         self.isRunning = False
#         self.moveToThread(self)
#         self.started.connect(self.measure)
        
#         # load frequently used constants
#         constants = MeasurementConstants()
#         self.configs = NiDaqChannels().look_up_table
#         self.sampleRate = constants.patchSealSampRate
#         self.frequency = constants.patchSealFreq
#         self.voltMin = constants.patchSealMinVol
#         self.voltMax = constants.patchSealMaxVol
#         self.dutycycle = constants.patchSealDuty
#         self.readNumber = 100 # should be a multiple of the wavelength
        
#         # look up channel configuration of NIDAQ
#         self.configs = NiDaqChannels().look_up_table
#         self.patchVoltOutChan = None
#         self.patchCurOutChan = None
#         self.patchVoltInChan = None
        
#         # generate wave function from loaded constants
#         self.wave = np.zeros(self.readNumber)
#         self.setWave(1,0,0)
        
#         # operation mode
#         self.mode = 'voltageclamp'
    
    
#     @property
#     def mode(self):
#         return self._mode
    
#     @mode.setter
#     def mode(self, mode):
#         self._mode = mode
#         print("Sealtestthread operation mode: "+mode)
    
    
#     def stop(self):
#         self.isRunning = False
#         self.quit()
#         self.wait()
    
#     def setWave(self, inVolGain, diffvoltage, lowervoltage):
#         self.voltMin = lowervoltage / inVolGain
#         self.voltMax = self.voltMin + diffvoltage / inVolGain

#         self.wave = blockWave(self.sampleRate, self.frequency, self.voltMin, self.voltMax, self.dutycycle)
    
#     def zap(self):
#         # interrupt measurement
#         self.stop()
        
#         # update attributes for zapping (there should be a function for generating this kind of wave right...?)
#         zap_voltage = 1
#         zap_voltagegain = 0.1
#         zap_duration = 200
#         self.sampleRate = 100000
#         self.readNumber = 500
#         self.voltMax = zap_voltage/zap_voltagegain
#         self.voltMin = 0/zap_voltagegain
#         self.wave = self.voltMax * np.ones(int(int(zap_duration) * 10 / 100 / 100000 * self.sampleRate))
#         self.wave = np.append(self.wave, np.zeros(10))  # give 10*0 at the end of waveform
#         self.sampleNumber = len(self.wave)
        
#         # change operation mode to zap and execute
#         self.mode = 'zap'
#         self.start()
        
#         # zap cannot be interupted so we press stop and just wait for it to end
#         self.stop()
        
#         # update the wavefunction for ordinary use and execute
#         self.sampleRate = MeasurementConstants().patchSealSampRate
#         self.readNumber = 100 # should be a multiple of the wavelength
#         self.setWave(0.1, 0.01, 0)
#         self.mode = 'voltageclamp'
        
#         # resume sealtestthread
#         self.start()
    
#     @pyqtSlot()
#     def measure(self):
#         """
#         Starts writing a waveform continuously to the patchclamp. While reading
#         the buffer periodically.
#         """
#         print('sealtest thread started')
        
#         self.patchVoltOutChan = self.configs["Vp"]
#         self.patchCurOutChan = self.configs["Ip"]
#         self.patchVoltInChan = self.configs["patchAO"]
        
#         if self.mode == 'voltageclamp':
#             self.isRunning = True
#             while self.isRunning:
#                 output = np.random.rand(2,self.readNumber)
#                 output[0,:] *= np.concatenate((np.ones(25),np.zeros(25),np.ones(25),np.zeros(25)))*1
#                 output[1,:] *= np.linspace(2,-2,100)*10**-2
#                 QThread.msleep(10)
    
#                 # Emiting the data just received as a signal
#                 self.measurement.emit(output[0,:], output[1,:])
#         elif self.mode == 'zap':
#             print('ZAPPPPP!')
#             QThread.msleep(200)
#         else:
#             pass
        
#         print('sealtest thread stopped')
    