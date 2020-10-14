# -*- coding: utf-8 -*-
"""
Created on Tue Dec 17 23:40:26 2019

@author: Meng
"""

from __future__ import division
from PyQt5 import QtWidgets
from PyQt5.QtCore import Qt, pyqtSignal, QRectF, QPoint, QRect, QObject
from PyQt5.QtGui import QColor, QPen, QPixmap, QIcon, QTextCursor, QFont

from PyQt5.QtWidgets import (QWidget, QButtonGroup, QLabel, QSlider, QSpinBox, QDoubleSpinBox, QGridLayout, QPushButton, QGroupBox, 
                             QLineEdit, QVBoxLayout, QHBoxLayout, QComboBox, QMessageBox, QTabWidget, QCheckBox, QRadioButton, 
                             QFileDialog, QProgressBar, QTextEdit, QStyleFactory, QStackedWidget)

import pyqtgraph as pg
from IPython import get_ipython
import sys
import numpy as np
from skimage.io import imread
from skimage.transform import rotate
import threading
import os
if __name__ == "__main__":
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname+'/../')
import copy
import time
from datetime import datetime, date
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import plotly.express as px
from NIDAQ.constants import HardwareConstants
import NIDAQ.WaveformWidget
from ScreeningWidget.EvolutionScanningThread import ScanningExecutionThread # This is the thread file for execution.
from SampleStageControl.stage import LudlStage
from NIDAQ.DAQoperator import DAQmission

import GalvoWidget.PMTWidget
import NIDAQ.AOTFWidget
import ThorlabsFilterSlider.FilterSliderWidget
import InsightX3.TwoPhotonLaserUI
import StylishQT


class Mainbody(QWidget):
    
    # waveforms_generated = pyqtSignal(object, object, list, int)
    #%%
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        os.chdir('./')# Set directory to current folder.
        self.setFont(QFont("Arial"))
        
        self.setWindowIcon(QIcon('./Icons/screening.png'))
        self.setWindowTitle("Gorgonzola")
        self.layout = QGridLayout(self)
        
        self.RoundQueueDict = {}        
        self.WaveformQueueDict = {}
        self.CamOperationDict = {}
        self.PhotocycleDict = {}
        self.RoundQueueDict['InsightEvents'] = []
        self.RoundQueueDict['FilterEvents'] = []
        
        self.RoundCoordsDict = {}
        self.WaveformQueueDict_GalvoInfor = {}
        self.GeneralSettingDict = {}
        self.FocusCorrectionMatrixDict = {}
        self.FocusStackInfoDict = {}
        self.popnexttopimgcounter = 0

        self.quick_start_location = [r"M:\tnw\ist\do\projects\Neurophotonics\Brinkslab\Data\Octoscope\2020-8-26 Screening Lenti Archon\2020-08-26_11-28-50_Pipeline.npy"]
        
        #**************************************************************************************************************************************
        #-----------------------------------------------------------GUI for Quick start--------------------------------------------------------
        #**************************************************************************************************************************************
        self.Quick_startContainer = StylishQT.roundQGroupBox("Quick start")
        self.Quick_startContainerLayout = QGridLayout()
        
        self.prefixtextbox = QtWidgets.QLineEdit(self)
        self.prefixtextbox.setPlaceholderText('Prefix')
        self.prefixtextbox.setFixedWidth(80)
        self.prefixtextbox.returnPressed.connect(self.set_prefix)
        self.Quick_startContainerLayout.addWidget(self.prefixtextbox, 0, 0)
        
        self.OpenSettingWidgetButton = QPushButton('New pipeline', self)
        # self.OpenSettingWidgetButton.setCheckable(True)
        self.Quick_startContainerLayout.addWidget(self.OpenSettingWidgetButton, 0, 1)
        self.OpenSettingWidgetButton.clicked.connect(self.showPipelineConfigWidget)  
        
        self.QuickStartButton_1 = QPushButton('Config 1', self)
        self.Quick_startContainerLayout.addWidget(self.QuickStartButton_1, 1, 0)
        self.QuickStartButton_1.clicked.connect(lambda: self.quick_start(0))
        
        self.openScreenAnalysisMLWidgetButton = QPushButton('Screen Analysis ML', self)
        self.Quick_startContainerLayout.addWidget(self.openScreenAnalysisMLWidgetButton, 2, 0)
        self.openScreenAnalysisMLWidgetButton.clicked.connect(self.openScreenAnalysisMLWidget)
        
        # self.Quick_startContainer.setFixedWidth(400)
        # self.Quick_startContainer.setFixedHeight(300)
        self.Quick_startContainer.setLayout(self.Quick_startContainerLayout)

        #**************************************************************************************************************************************
        #-----------------------------------------------------------GUI for GeneralSettings----------------------------------------------------
        #**************************************************************************************************************************************
        self.GeneralSettingContainer = StylishQT.roundQGroupBox("Tanto Tanto")
        self.GeneralSettingContainerLayout = QGridLayout()
        
        self.saving_prefix = ''
        self.savedirectorytextbox = QtWidgets.QLineEdit(self)
        self.savedirectorytextbox.setFixedWidth(300)
        self.savedirectorytextbox.returnPressed.connect(self.update_saving_directory)
        self.GeneralSettingContainerLayout.addWidget(self.savedirectorytextbox, 0, 1)
        
        self.toolButtonOpenDialog = QtWidgets.QPushButton('Saving directory')
        self.toolButtonOpenDialog.setStyleSheet("QPushButton {color:white;background-color: pink; border-style: outset;border-radius: 10px;border-width: 2px;font: bold 14px;padding: 6px}"
                                                "QPushButton:pressed {color:yellow;background-color: pink; border-style: outset;border-radius: 3px;border-width: 2px;font: bold 14px;padding: 1px}"
                                                "QPushButton:hover:!pressed {color:gray;background-color: pink; border-style: outset;border-radius: 10px;border-width: 2px;font: bold 14px;padding: 6px}")

        self.toolButtonOpenDialog.setObjectName("toolButtonOpenDialog")
        self.toolButtonOpenDialog.clicked.connect(self._open_file_dialog)
        
        self.GeneralSettingContainerLayout.addWidget(self.toolButtonOpenDialog, 0, 0)
        
        ButtonConfigurePipeline = StylishQT.generateButton()
        ButtonConfigurePipeline.clicked.connect(self.ConfigGeneralSettings)
#        ButtonConfigurePipeline.clicked.connect(self.GenerateFocusCorrectionMatrix)
        
        ButtonExePipeline = StylishQT.runButton()
        ButtonExePipeline.clicked.connect(self.ExecutePipeline)
        
        ButtonSavePipeline = StylishQT.saveButton()
        ButtonSavePipeline.clicked.connect(self.Savepipeline)
        
        # Pipeline import
        self.LoadPipelineAddressbox = QLineEdit(self)    
        self.LoadPipelineAddressbox.setFixedWidth(300)
        self.GeneralSettingContainerLayout.addWidget(self.LoadPipelineAddressbox, 1, 1)
        
        self.BrowsePipelineButton = QPushButton('Browse pipeline', self)
        self.BrowsePipelineButton.setStyleSheet("QPushButton {color:white;background-color:rgb(143,191,224); border-style: outset;border-radius: 10px;border-width: 2px;font: bold 14px;padding: 6px}"
                                                "QPushButton:pressed {color:red;background-color: white; border-style: outset;border-radius: 10px;border-width: 2px;font: bold 14px;padding: 6px}"
                                                "QPushButton:hover:!pressed {color:gray;background-color:rgb(143,191,224); border-style: outset;border-radius: 10px;border-width: 2px;font: bold 14px;padding: 6px}")        
        
        self.GeneralSettingContainerLayout.addWidget(self.BrowsePipelineButton, 1, 0) 
        
        self.BrowsePipelineButton.clicked.connect(self.GetPipelineNPFile)
        
        self.GeneralSettingContainerLayout.addWidget(QLabel('Configure focus correction first.'), 1, 2)
        
        self.ImportPipelineButton = QPushButton('Load', self)
        self.ImportPipelineButton.setStyleSheet("QPushButton {color:white;background-color: rgb(191,216,189); border-style: outset;border-radius: 10px;border-width: 2px;font: bold 14px;padding: 6px}"
                                                "QPushButton:pressed {color:red;background-color: white; border-style: outset;border-radius: 10px;border-width: 2px;font: bold 14px;padding: 6px}"
                                                "QPushButton:hover:!pressed {color:gray;background-color: rgb(191,216,189); border-style: outset;border-radius: 10px;border-width: 2px;font: bold 14px;padding: 6px}")        

        self.GeneralSettingContainerLayout.addWidget(self.ImportPipelineButton, 1, 3)
        self.ImportPipelineButton.clicked.connect(self.LoadPipelineFile)
        
        self.GeneralSettingContainerLayout.addWidget(ButtonConfigurePipeline, 0, 3)        
        self.GeneralSettingContainerLayout.addWidget(ButtonExePipeline, 0, 5)
        self.GeneralSettingContainerLayout.addWidget(ButtonSavePipeline, 0, 4)    
        self.GeneralSettingContainer.setLayout(self.GeneralSettingContainerLayout)
        
        #**************************************************************************************************************************************
        #-----------------------------------------------------------GUI for Billboard display------------------------------------------------------
        #**************************************************************************************************************************************
        self.ImageDisplayContainer = QGroupBox()
        self.ImageDisplayContainerLayout = QGridLayout()        
        
        # AOTFWidgetInstance = NIDAQ.AOTFWidget.AOTFWidgetUI()
        # self.ImageDisplayContainerLayout.addWidget(AOTFWidgetInstance, 0, 0, 1, 1)
        
        # FilterSliderWidgetInstance = ThorlabsFilterSlider.FilterSliderWidget.FilterSliderWidgetUI()
        # self.ImageDisplayContainerLayout.addWidget(FilterSliderWidgetInstance, 1, 0, 1, 1)
                
        self.ConsoleTextDisplay = QTextEdit()
        self.ConsoleTextDisplay.setFontItalic(True)
        self.ConsoleTextDisplay.setPlaceholderText('Notice board from console.')
        # self.ConsoleTextDisplay.setMaximumHeight(200)
        # self.ConsoleTextDisplay.setFixedWidth(200)
        self.ImageDisplayContainerLayout.addWidget(self.ConsoleTextDisplay, 0, 1, 2, 1)
        
        # self.ImageDisplayContainerLayout.addWidget(ToolWidgetsContainer, 2, 0, 1, 1)
        
        self.ImageDisplayContainer.setLayout(self.ImageDisplayContainerLayout)
        # self.ImageDisplayContainer.setMinimumHeight(400)
        # self.ImageDisplayContainer.setMinimumWidth(100)
     
    #--------------------------------------------------------------------------------------------------------------------------------------------
        #**************************************************************************************************************************************
        #-----------------------------------------------------------Pipeline configure widget--------------------------------------------------
        #**************************************************************************************************************************************
        self.PipelineConfigureWidget = QWidget()
        self.PipelineConfigureWidget.layout = QGridLayout()
        

        #**************************************************************************************************************************************
        #-----------------------------------------------------------GUI for PiplineContainer---------------------------------------------------
        #**************************************************************************************************************************************

        self.PipelineContainer = StylishQT.roundQGroupBox("Pipeline settings")
        self.PipelineContainerLayout = QGridLayout()
        
        self.RoundOrderBox = QSpinBox(self)
        self.RoundOrderBox.setMinimum(1)
        self.RoundOrderBox.setMaximum(1000)
        self.RoundOrderBox.setValue(1)
        self.RoundOrderBox.setSingleStep(1)
        self.RoundOrderBox.setMaximumWidth(30)
        self.PipelineContainerLayout.addWidget(self.RoundOrderBox, 0, 1)
        self.PipelineContainerLayout.addWidget(QLabel("Round sequence:"), 0, 0)
                
        ButtonAddRound = StylishQT.addButton()
        ButtonDeleteRound = StylishQT.stop_deleteButton()
        
        self.PipelineContainerLayout.addWidget(ButtonAddRound, 0, 2)
        ButtonAddRound.clicked.connect(self.AddFreshRound)
        ButtonAddRound.clicked.connect(self.GenerateScanCoords)
        
        self.PipelineContainerLayout.addWidget(ButtonDeleteRound, 0, 3)
        ButtonDeleteRound.clicked.connect(self.DeleteFreshRound)
        
        ButtonClearRound = StylishQT.cleanButton()        
        self.PipelineContainerLayout.addWidget(ButtonClearRound, 0, 4)
        ButtonClearRound.clicked.connect(self.ClearRoundQueue)
        
        self.ScanRepeatTextbox = QSpinBox(self)
        self.ScanRepeatTextbox.setMinimum(1)
        self.ScanRepeatTextbox.setValue(1)
        self.ScanRepeatTextbox.setMaximum(100000)
        self.ScanRepeatTextbox.setSingleStep(1)
        self.PipelineContainerLayout.addWidget(self.ScanRepeatTextbox, 0, 7)
        self.PipelineContainerLayout.addWidget(QLabel("Meshgrid:"), 0, 6)  
        
        self.OpenTwoPLaserShutterCheckbox = QCheckBox("Open shutter first")
        self.OpenTwoPLaserShutterCheckbox.setStyleSheet('color:blue;font:bold "Times New Roman"')
        self.OpenTwoPLaserShutterCheckbox.setChecked(True)
        self.PipelineContainerLayout.addWidget(self.OpenTwoPLaserShutterCheckbox, 0, 8)  

        #**************************************************************************************************************************************
        #-----------------------------------------------------------GUI for StageScanContainer-------------------------------------------------
        #**************************************************************************************************************************************    
        ScanContainer = QWidget()     
        ScanSettingLayout = QGridLayout() #Layout manager
        ScanContainer.layout = ScanSettingLayout
        
        self.ScanStepsNumTextbox = QSpinBox(self)
        self.ScanStepsNumTextbox.setMinimum(1)
        self.ScanStepsNumTextbox.setMaximum(100000)
        self.ScanStepsNumTextbox.setValue(10)
        self.ScanStepsNumTextbox.setSingleStep(1)
        ScanSettingLayout.addWidget(self.ScanStepsNumTextbox, 0, 1)
        ScanSettingLayout.addWidget(QLabel("Stage scanning step number:"), 0, 0)  

        self.ScanstepTextbox = QSpinBox(self)
        self.ScanstepTextbox.setMaximum(20000)
        self.ScanstepTextbox.setValue(1650)
        self.ScanstepTextbox.setSingleStep(500)
        ScanSettingLayout.addWidget(self.ScanstepTextbox, 0, 7)
        ScanSettingLayout.addWidget(QLabel("Step size:"), 0, 6)
        
        self.AutoFocusGapTextbox = QSpinBox(self)
        self.AutoFocusGapTextbox.setMinimum(1)
        self.AutoFocusGapTextbox.setMaximum(100000)
        self.AutoFocusGapTextbox.setValue(0)
        self.AutoFocusGapTextbox.setSingleStep(5)

        ScanSettingLayout.addWidget(self.AutoFocusGapTextbox, 0, 5)
        ScanSettingLayout.addWidget(QLabel("Auto focus grid steps:"), 0, 4)
        
        self.FocusStackNumTextbox = QSpinBox(self)
        self.FocusStackNumTextbox.setMinimum(1)
        self.FocusStackNumTextbox.setMaximum(20000)
        self.FocusStackNumTextbox.setValue(1)
        self.FocusStackNumTextbox.setSingleStep(1)
        ScanSettingLayout.addWidget(self.FocusStackNumTextbox, 1, 5)
        ScanSettingLayout.addWidget(QLabel("Focus stack number:"), 1, 4)
        
        self.FocusStackStepTextbox = QDoubleSpinBox(self)
        self.FocusStackStepTextbox.setMinimum(0)
        self.FocusStackStepTextbox.setMaximum(10000)
        self.FocusStackStepTextbox.setDecimals(6)
        self.FocusStackStepTextbox.setValue(0.002)
        self.FocusStackStepTextbox.setSingleStep(0.001)  
        ScanSettingLayout.addWidget(self.FocusStackStepTextbox, 1, 7)
        ScanSettingLayout.addWidget(QLabel("Focus stack step(mm):"), 1, 6)   
        
        ScanContainer.setLayout(ScanSettingLayout)
        
        #**************************************************************************************************************************************
        #-----------------------------------------------------------GUI for Laser/filter-------------------------------------------------
        #**************************************************************************************************************************************  
        TwoPLaserContainer = QGroupBox()        
        TwoPLaserSettingLayout = QGridLayout() #Layout manager
        
        self.TwoPLaserFilterCheckbox = QCheckBox("Insight/Filter event")
        self.TwoPLaserFilterCheckbox.setStyleSheet('color:blue;font:bold "Times New Roman"')
        TwoPLaserSettingLayout.addWidget(self.TwoPLaserFilterCheckbox, 0, 0)
        
        self.TwoPLaserWavelengthbox = QSpinBox(self)
        self.TwoPLaserWavelengthbox.setMinimum(680)
        self.TwoPLaserWavelengthbox.setMaximum(1300)
        self.TwoPLaserWavelengthbox.setSingleStep(100)
        self.TwoPLaserWavelengthbox.setValue(1280)
        TwoPLaserSettingLayout.addWidget(self.TwoPLaserWavelengthbox, 0, 1)
        
        self.TwoPLaserShutterCombox = QComboBox()
        self.TwoPLaserShutterCombox.addItems(['No shutter event', 'Open', 'Close'])
        TwoPLaserSettingLayout.addWidget(self.TwoPLaserShutterCombox, 1, 1)

        #--------filter------------
        NDfilterlabel = QLabel("ND filter:")
        TwoPLaserSettingLayout.addWidget(NDfilterlabel, 0, 3)
        NDfilterlabel.setAlignment(Qt.AlignRight)
        self.NDfilterCombox = QComboBox()
        self.NDfilterCombox.addItems(['1', '2', '2.3', '2.5', '3', '0.5'])
        TwoPLaserSettingLayout.addWidget(self.NDfilterCombox, 0, 4)
        
        Emifilterlabel = QLabel("Emission filter:")
        TwoPLaserSettingLayout.addWidget(Emifilterlabel, 1, 3)
        Emifilterlabel.setAlignment(Qt.AlignRight)
        self.EmisfilterCombox = QComboBox()
        self.EmisfilterCombox.addItems(['Arch', 'eGFP', 'Citrine'])
        TwoPLaserSettingLayout.addWidget(self.EmisfilterCombox, 1, 4)
        
        ButtonDelEvent = QPushButton('Delete event', self)
        TwoPLaserSettingLayout.addWidget(ButtonDelEvent, 1, 5) 
        ButtonDelEvent.clicked.connect(self.DelFilterEvent)
        ButtonDelEvent.clicked.connect(self.DelInsightEvent)
        
        TwoPLaserContainer.setLayout(TwoPLaserSettingLayout)
                
        #--------------------------------------------------------------------------------------------------------------------------------------
        self.RoundGeneralSettingTabs = QTabWidget()
        self.RoundGeneralSettingTabs.addTab(ScanContainer,"Scanning settings")
        self.RoundGeneralSettingTabs.addTab(TwoPLaserContainer,"Pulse laser/Filter settings")

        self.PipelineContainerLayout.addWidget(self.RoundGeneralSettingTabs, 2, 0, 1, 10)
        
        
        self.WaveformOrderBox = QSpinBox(self)
        self.WaveformOrderBox.setMinimum(1)
        self.WaveformOrderBox.setMaximum(1000)
        self.WaveformOrderBox.setValue(1)
        self.WaveformOrderBox.setSingleStep(1)
        self.WaveformOrderBox.setMaximumWidth(30)
        self.PipelineContainerLayout.addWidget(self.WaveformOrderBox, 3, 1)
        self.PipelineContainerLayout.addWidget(QLabel("Waveform/Camera sequence:"), 3, 0)
        
        ButtonAddWaveform = StylishQT.addButton()
        ButtonDeleteWaveform = StylishQT.stop_deleteButton()
        
        ButtonClearWaveform = StylishQT.cleanButton()

        self.PipelineContainerLayout.addWidget(ButtonAddWaveform, 3, 3)
        self.PipelineContainerLayout.addWidget(ButtonDeleteWaveform, 3, 4)
        self.PipelineContainerLayout.addWidget(ButtonClearWaveform, 3, 5)
        
        ButtonAddWaveform.clicked.connect(self.AddFreshWaveform)
        ButtonAddWaveform.clicked.connect(self.AddCameraOperation)
        ButtonAddWaveform.clicked.connect(self.AddPhotocycleOperation)
        
        ButtonDeleteWaveform.clicked.connect(self.DeleteFreshWaveform)
        ButtonDeleteWaveform.clicked.connect(self.DeleteCameraOperation)
        ButtonDeleteWaveform.clicked.connect(self.DeletePhotocycleOperation)
        
        ButtonClearWaveform.clicked.connect(self.ClearWaveformQueue)
        ButtonClearWaveform.clicked.connect(self.CleanCameraOperation)
        ButtonClearWaveform.clicked.connect(self.CleanPhotocycleOperation)
        #--------------------------------------------------------------------------------------------------------------------------------------
        self.EachCoordDwellSettingTabs = QTabWidget()
 
        # =============================================================================
        #         Waveforms tab settings
        # =============================================================================
        waveformTab = QWidget()
        waveformTabLayout = QGridLayout()
        
        self.Waveformer_widget_instance = NIDAQ.WaveformWidget.WaveformGenerator()
        self.Waveformer_widget_instance.WaveformPackage.connect(self.UpdateWaveformerSignal)
        self.Waveformer_widget_instance.GalvoScanInfor.connect(self.UpdateWaveformerGalvoInfor)

        waveformTabLayout.addWidget(self.Waveformer_widget_instance, 2, 0, 2, 9)
        waveformTab.setLayout(waveformTabLayout)
        
        # =============================================================================
        #         Camera tab settings
        # =============================================================================
        CameraDwellTab = QWidget()
        CameraDwellTabLayout = QGridLayout()
        
        self.photocycleChecbox = QCheckBox("Photo cycle")
        self.photocycleChecbox.setStyleSheet('color:Indigo;font:bold "Times New Roman"')
        CameraDwellTabLayout.addWidget(self.photocycleChecbox, 0, 0)  
        

        self.CamTriggerSettingBox = QComboBox()
        self.CamTriggerSettingBox.addItems(["EXTERNAL", "INTERNAL"])
        
        self.CamTriggerActive_SettingBox = QComboBox()
        self.CamTriggerActive_SettingBox.addItems(['EDGE', 'LEVEL', 'SYNCREADOUT'])
        
        CameraDwellTabLayout.addWidget(QLabel("Trigger:"), 2, 0)
        CameraDwellTabLayout.addWidget(self.CamTriggerSettingBox, 2, 1)
        CameraDwellTabLayout.addWidget(self.CamTriggerActive_SettingBox, 2, 2)
        
        self.StreamBufferTotalFrames_spinbox = QSpinBox()
        self.StreamBufferTotalFrames_spinbox.setMaximum(120000)
        self.StreamBufferTotalFrames_spinbox.setValue(0)
        CameraDwellTabLayout.addWidget(self.StreamBufferTotalFrames_spinbox, 2, 4)
        CameraDwellTabLayout.addWidget(QLabel("Buffers:"), 2, 3)
        
        self.CamExposureBox = QDoubleSpinBox(self)
        self.CamExposureBox.setDecimals(6)
        self.CamExposureBox.setMinimum(0)
        self.CamExposureBox.setMaximum(100)
        self.CamExposureBox.setValue(0.001501)
        self.CamExposureBox.setSingleStep(0.001)  
        CameraDwellTabLayout.addWidget(self.CamExposureBox, 2, 6)  
        CameraDwellTabLayout.addWidget(QLabel("Exposure time:"), 2, 5)
        
        #---------------------------Camera ROI settings------------------------
        CameraROIPosContainer = QGroupBox("ROI position")
        CameraROIPosContainer.setStyleSheet("QGroupBox { background-color:#F5F5F5;}")
        CameraROIPosLayout = QGridLayout()
        
        OffsetLabel = QLabel("Offset")
        OffsetLabel.setFixedHeight(30)
        ROISizeLabel = QLabel("Size")
        ROISizeLabel.setFixedHeight(30)
        
        CameraROIPosLayout.addWidget(OffsetLabel, 2, 1)
        CameraROIPosLayout.addWidget(ROISizeLabel, 2, 2)

        self.ROI_hpos_spinbox = QSpinBox()
        self.ROI_hpos_spinbox.setMaximum(2048)
        self.ROI_hpos_spinbox.setValue(0)

        CameraROIPosLayout.addWidget(self.ROI_hpos_spinbox, 3, 1)
        
        self.ROI_vpos_spinbox = QSpinBox()
        self.ROI_vpos_spinbox.setMaximum(2048)
        self.ROI_vpos_spinbox.setValue(0)

        CameraROIPosLayout.addWidget(self.ROI_vpos_spinbox, 4, 1)
        
        self.ROI_hsize_spinbox = QSpinBox()
        self.ROI_hsize_spinbox.setMaximum(2048)
        self.ROI_hsize_spinbox.setValue(2048)

        CameraROIPosLayout.addWidget(self.ROI_hsize_spinbox, 3, 2)
        
        self.ROI_vsize_spinbox = QSpinBox()
        self.ROI_vsize_spinbox.setMaximum(2048)
        self.ROI_vsize_spinbox.setValue(2048)

        CameraROIPosLayout.addWidget(self.ROI_vsize_spinbox, 4, 2)
        
        CameraROIPosLayout.addWidget(QLabel("Horizontal"), 3, 0)
        CameraROIPosLayout.addWidget(QLabel("Vertical"), 4, 0)
        
        CameraROIPosContainer.setLayout(CameraROIPosLayout)
        CameraROIPosContainer.setFixedHeight(105)
        
        CameraDwellTabLayout.addWidget(CameraROIPosContainer, 3, 1, 3, 3)
        
        CameraDwellTab.setLayout(CameraDwellTabLayout)
        
        self.EachCoordDwellSettingTabs.addTab(waveformTab,"Waveforms settings")
        self.EachCoordDwellSettingTabs.addTab(CameraDwellTab,"Camera operations")

        self.PipelineContainerLayout.addWidget(self.EachCoordDwellSettingTabs, 4, 0, 4, 10)    
        
        self.PipelineContainer.setLayout(self.PipelineContainerLayout)
        
        #----------------------------------------------------------------------
        self.PipelineConfigureWidget.layout.addWidget(self.GeneralSettingContainer, 0,0)
        self.PipelineConfigureWidget.layout.addWidget(self.PipelineContainer, 1,0)
        self.PipelineConfigureWidget.setLayout(self.PipelineConfigureWidget.layout)
        
        #**************************************************************************************************************************************
        #-----------------------------------------------------------GUI for Stack widget--------------------------------------------------------
        #**************************************************************************************************************************************
        startupWidget = QWidget()
        
        self.settingStackedWidget =  QStackedWidget()
        self.settingStackedWidget.addWidget(startupWidget)
        self.settingStackedWidget.addWidget(self.PipelineConfigureWidget)
        self.settingStackedWidget.setCurrentIndex(0)
        # self.setFixedWidth(400)
        # self.setFixedHeight(300)
        
        self.layout.addWidget(self.Quick_startContainer, 1, 0, 1, 2)
        
        self.setLayout(self.layout)
        
    def showPipelineConfigWidget(self):
        self.layout.addWidget(self.ImageDisplayContainer, 1, 2, 1, 2)
        self.layout.addWidget(self.settingStackedWidget, 2, 0, 1, 4)  
        
        self.settingStackedWidget.setCurrentIndex(1)

            
    #%%
    """
    #     FUNCTIONS FOR EXECUTION
    
    ----------------Screening routine configuration Structure -----------------
    
    ====RoundQueueDict====                              Dictionary=============
    
      -- key: RoundPackage_{}                           List of operations at each coordinate. {} stands for round sequence number.
            |__ WaveformQueueDict                       Dictionary
                key: WaveformPackage_{}                 Waveforms tuple signal from Waveformer. At each coordinate.
                
            |__ CamOperationDict                        Dictionary
                key: CameraPackage_{}                   Camera operations at each coordinate. {} stands for waveform/camera sequence number.
                
            |__ PhotocycleDict                          Dictionary
                key: PhotocyclePackage_{}               Photocycle experiment information. {} stands for waveform/camera sequence number.
            
      -- key: GalvoInforPackage_{}                   
            |__ WaveformQueueDict_GalvoInfor            Dictionary
                key: GalvoInfor_{}                      Galvo scanning configuration signal from Waveformer. At each coordinate.
                
      -- key: FilterEvents
            |__ List of filter operation strings, in the round.
            
      -- key: InsightEvents
            |__ List of insight laser operation strings, in the round.
        
    ====RoundCoordsDict====                             Dictionary=============
    
      -- key: CoordsPackage_{}
            |__ np.array of scanning coordinates.
            
    ====GeneralSettingDict====                          Dictionary=============
    
      -- key: 'savedirectory'                           screening data saving directory.
      
      -- key: 'FocusCorrectionMatrixDict'               Dictionary
                  |__ key: RoundPackage_{}
                      or RoundPackage_{}_Grid_{}        np.array of pre-calibrated focus positions.
                  
      -- key: 'FocusStackInfoDict'                      Dictionary
                  |__ key: RoundPackage_{}              String specifies 'NumberOfFocus{}WithIncrementBeing{}'.
                  
      -- key: 'Meshgrid'                                int if scanning grid number. meshrepeat
      
      -- key: 'Scanning step'                           Scanning stage step. self.step
      
      -- key: 'StartUpEvents'                           List of strings, like Shutter_Open
    """
    # ==========================================================================================================================================================
    # ------------------------------------------------------------Waveform package functions at each coordinate-------------------------------------------------
    # ==========================================================================================================================================================
    """
    Every time when the 'configure' button in waveformer widget is hit, the 'WaveformPackage' and 'GalvoInfor' signals are sent here.
    """
    def UpdateWaveformerSignal(self, WaveformPackage):
        """
        Capture the newest generated waveform tuple signal from Waveformer, which contains 4 parts in tuple:
        (sampling rate, analogcontainer_array, digitalcontainer_array, recording channel list)
        """
        self.FreshWaveformPackage = WaveformPackage

    def UpdateWaveformerGalvoInfor(self, GalvoInfor):
        self.FreshWaveformGalvoInfor = GalvoInfor
    
    def AddFreshWaveform(self): # Add waveform package for single round.
        CurrentWaveformPackageSequence = self.WaveformOrderBox.value()
        try:
            self.WaveformQueueDict['WaveformPackage_{}'.format(CurrentWaveformPackageSequence)] = self.FreshWaveformPackage
        except AttributeError:
            QMessageBox.warning(self,'Error','Click configure waveform first!',QMessageBox.Ok)
            
        self.WaveformQueueDict_GalvoInfor['GalvoInfor_{}'.format(CurrentWaveformPackageSequence)] = self.FreshWaveformGalvoInfor
        self.normalOutputWritten('Waveform{} added.\n'.format(CurrentWaveformPackageSequence))
        print('Waveform added.')
        
    def DeleteFreshWaveform(self): # Empty the waveform container to avoid crosstalk between rounds.
        CurrentWaveformPackageSequence = self.WaveformOrderBox.value()
        del self.WaveformQueueDict['WaveformPackage_{}'.format(CurrentWaveformPackageSequence)]
        
        del self.WaveformQueueDict_GalvoInfor['GalvoInfor_{}'.format(CurrentWaveformPackageSequence)]
        
    def ClearWaveformQueue(self):
        self.WaveformQueueDict = {}
        self.WaveformQueueDict_GalvoInfor = {}
        
    # ==========================================================================================================================================================
    # --------------------------------------------------------------Camera operation at each coordinate---------------------------------------------------------
    # ==========================================================================================================================================================
    def AddCameraOperation(self):
        CurrentCamPackageSequence = self.WaveformOrderBox.value()
        
        if self.StreamBufferTotalFrames_spinbox.value() != 0:
            CameraOperation = {"Settings": ["trigger_source", self.CamTriggerSettingBox.currentText(), 
                                            "exposure_time", self.CamExposureBox.value(),
                                            "trigger_active", self.CamTriggerActive_SettingBox.currentText(),
                                            "subarray_hsize", self.ROI_hsize_spinbox.value(),
                                            "subarray_vsize", self.ROI_vsize_spinbox.value(),
                                            "subarray_hpos", self.ROI_hpos_spinbox.value(),
                                            "subarray_vpos", self.ROI_vpos_spinbox.value()
                                            ], 
                               "Buffer_number": self.StreamBufferTotalFrames_spinbox.value()}
            
            self.CamOperationDict['CameraPackage_{}'.format(CurrentCamPackageSequence)] = CameraOperation
        else:
            self.CamOperationDict['CameraPackage_{}'.format(CurrentCamPackageSequence)] = {}

    def DeleteCameraOperation(self): # Empty the waveform container to avoid crosstalk between rounds.
        CurrentCamPackageSequence = self.WaveformOrderBox.value()
        del self.CamOperationDict['CameraPackage_{}'.format(CurrentCamPackageSequence)]    
        
    def CleanCameraOperation(self):
        self.CamOperationDict = {}
        
    # ==========================================================================================================================================================
    # --------------------------------------------------------------Photocycle operation at each coordinate-----------------------------------------------------
    # ==========================================================================================================================================================
    def AddPhotocycleOperation(self):
        CurrentPhotocycleSequence = self.WaveformOrderBox.value()
        
        if self.photocycleChecbox.isChecked():
            PhotocycleOperation = [True]
            self.PhotocycleDict['PhotocyclePackage_{}'.format(CurrentPhotocycleSequence)] = PhotocycleOperation
        else:
            self.PhotocycleDict['PhotocyclePackage_{}'.format(CurrentPhotocycleSequence)] = {}   
            
    def DeletePhotocycleOperation(self):
        CurrentPhotocycleSequence = self.WaveformOrderBox.value()
        del self.PhotocycleDict['PhotocyclePackage_{}'.format(CurrentPhotocycleSequence)]    
        
    def CleanPhotocycleOperation(self):
        self.PhotocycleDict = {}
    # ==========================================================================================================================================================
    # --------------------------------------------------------------Settings at each round----------------------------------------------------------------------
    # ==========================================================================================================================================================
    def AddFreshRound(self):
        CurrentRoundSequence = self.RoundOrderBox.value()
        
        WaveformQueueDict = copy.deepcopy(self.WaveformQueueDict) # Here we make the self.WaveformQueueDict private so that other rounds won't refer to the same variable.
        WaveformQueueDict_GalvoInfor = copy.deepcopy(self.WaveformQueueDict_GalvoInfor)
        CamOperationDict = copy.deepcopy(self.CamOperationDict)
        PhotocycleDict = copy.deepcopy(self.PhotocycleDict)
        
        self.RoundQueueDict['RoundPackage_{}'.format(CurrentRoundSequence)] = [WaveformQueueDict, CamOperationDict, PhotocycleDict]
        self.RoundQueueDict['GalvoInforPackage_{}'.format(CurrentRoundSequence)] = WaveformQueueDict_GalvoInfor # Information we need to restore pmt scanning images.
        
        #Configure information for Z-stack
        ZstackNumber = self.FocusStackNumTextbox.value()
        ZstackStep = self.FocusStackStepTextbox.value()
        
        self.FocusStackInfoDict['RoundPackage_{}'.format(CurrentRoundSequence)] = 'NumberOfFocus{}WithIncrementBeing{}'.format(ZstackNumber, ZstackStep)
        
        self.AddFilterEvent()
        
        self.AddInsightEvent()
        
        self.normalOutputWritten('Round{} added.\n'.format(CurrentRoundSequence))
        print('Round added.')
        
    #-----------------------Configure filter event-----------------------------
    def AddFilterEvent(self):
        CurrentRoundSequence = self.RoundOrderBox.value()
        
        if self.TwoPLaserFilterCheckbox.isChecked():
            
            self.RoundQueueDict['FilterEvents'].append('Round_{}_ND_ToPos_{}'.format(CurrentRoundSequence, self.NDfilterCombox.currentText()))
            self.RoundQueueDict['FilterEvents'].append('Round_{}_EM_ToPos_{}'.format(CurrentRoundSequence, self.EmisfilterCombox.currentText()))
            print(self.RoundQueueDict['FilterEvents'])
            self.normalOutputWritten('FilterEvents'+str(self.RoundQueueDict['FilterEvents'])+'\n')
        
    def DelFilterEvent(self):
        CurrentRoundSequence = self.RoundOrderBox.value()
        
        if 'Round_{}_ND_ToPos_{}'.format(CurrentRoundSequence, self.NDfilterCombox.currentText()) in self.RoundQueueDict['FilterEvents']:
            self.RoundQueueDict['FilterEvents'].remove('Round_{}_ND_ToPos_{}'.format(CurrentRoundSequence, self.NDfilterCombox.currentText()))
            self.RoundQueueDict['FilterEvents'].remove('Round_{}_EM_ToPos_{}'.format(CurrentRoundSequence, self.EmisfilterCombox.currentText()))
        print(self.RoundQueueDict['FilterEvents'])
        self.normalOutputWritten(str(self.RoundQueueDict['FilterEvents'])+'\n')
        
    #-----------------------Configure insight event-----------------------------
    def AddInsightEvent(self):
        CurrentRoundSequence = self.RoundOrderBox.value()
        
        if self.TwoPLaserFilterCheckbox.isChecked():
            self.RoundQueueDict['InsightEvents'].append('Round_{}_WavelengthTo_{}'.format(CurrentRoundSequence, self.TwoPLaserWavelengthbox.value()))
            
            if self.TwoPLaserShutterCombox.currentText() != 'No shutter event':
                self.RoundQueueDict['InsightEvents'].append('Round_{}_Shutter_{}'.format(CurrentRoundSequence, self.TwoPLaserShutterCombox.currentText()))
            
        print(self.RoundQueueDict['InsightEvents'])
        self.normalOutputWritten('InsightEvents' + str(self.RoundQueueDict['InsightEvents'])+'\n')
        
    def DelInsightEvent(self):
        CurrentRoundSequence = self.RoundOrderBox.value()
        
        if self.TwoPLaserFilterCheckbox.isChecked():
            self.RoundQueueDict['InsightEvents'].remove('Round_{}_WavelengthTo_{}'.format(CurrentRoundSequence, self.TwoPLaserWavelengthbox.value()))
            
            if self.TwoPLaserShutterCombox.currentText() != 'No shutter event':
                self.RoundQueueDict['InsightEvents'].remove('Round_{}_Shutter_{}'.format(CurrentRoundSequence, self.TwoPLaserShutterCombox.currentText()))
            
        print(self.RoundQueueDict['InsightEvents'])
        self.normalOutputWritten(str(self.RoundQueueDict['InsightEvents'])+'\n')
   
    #-----------------------------Generate Scan Coords-----------------------------
    def GenerateScanCoords(self):
        CurrentRoundSequence = self.RoundOrderBox.value()
        # settings for scanning index
        step = self.ScanstepTextbox.value()
        
        row_start = 0 
        row_end = (self.ScanStepsNumTextbox.value() -1) * step
        
        column_start = 0 
        column_end = (self.ScanStepsNumTextbox.value() -1) * step
        
        # Generate structured array containing scanning coordinates' information.
        AutoFocusGrid_steps = self.AutoFocusGapTextbox.value()
        AutoFocusCoordGap = AutoFocusGrid_steps * step
        
        # Number of coordinates per row
        Coords_number_per_row = (row_end - row_start) / step + 1
        
        # Generate the 
        if AutoFocusGrid_steps != 0:
            AutoFocusGridNum = int(Coords_number_per_row / AutoFocusGrid_steps)
            AutoFocusGridOffsetList = []
            # Auto focus grid coordinates offset list
            for row_offset in range(AutoFocusGridNum):
                for col_offset in range(AutoFocusGridNum):
                    AutoFocusGridOffsetList.append([row_offset * step * AutoFocusGrid_steps, col_offset * step * AutoFocusGrid_steps])
        else:
            AutoFocusGridOffsetList = [[0, 0]]

        # Data type of structured array.
        Coords_array_dtype = np.dtype([('row', 'i4'), ('col', 'i4'), ('auto_focus_flag', 'U10'), ('focus_position', 'f4')])
        
        Coords_array = np.array([], dtype=Coords_array_dtype)
        
        for AutoFocusGridOffset in AutoFocusGridOffsetList:
            AutoFocusOffset_row = AutoFocusGridOffset[0]
            AutoFocusOffset_col = AutoFocusGridOffset[1]
            
            for row_pos in range(row_start, AutoFocusCoordGap, step):
                for col_pos in range(column_start, AutoFocusCoordGap, step):
                    # At each left-top corner of the coordinates grid, place the 
                    # flag for auto focus
                    if col_pos == 0 and row_pos == 0 and AutoFocusGrid_steps != 0:
                        current_coord_array = np.array([(row_pos + AutoFocusOffset_row, col_pos + AutoFocusOffset_col, 'yes', -1)], dtype=Coords_array_dtype)
                    else:
                        current_coord_array = np.array([(row_pos + AutoFocusOffset_row, col_pos + AutoFocusOffset_col, 'no', -1)], dtype=Coords_array_dtype)
                        
                    Coords_array = np.append(Coords_array, current_coord_array)

        self.RoundCoordsDict['CoordsPackage_{}'.format(CurrentRoundSequence)] = Coords_array
        
    def DeleteFreshRound(self):
        CurrentRoundSequence = self.RoundOrderBox.value()
        del self.RoundQueueDict['RoundPackage_{}'.format(CurrentRoundSequence)]
        del self.RoundCoordsDict['CoordsPackage_{}'.format(CurrentRoundSequence)]
        del self.RoundQueueDict['GalvoInforPackage_{}'.format(CurrentRoundSequence)]
        print(self.RoundQueueDict.keys())    
    
    def ClearRoundQueue(self):
        self.WaveformQueueDict = {}
        self.CamOperationDict = {}
        self.PhotocycleDict = {}
        self.RoundQueueDict = {}
        self.RoundQueueDict['InsightEvents'] = []
        self.RoundQueueDict['FilterEvents'] = []
        self.RoundCoordsDict = {}
        self.WaveformQueueDict_GalvoInfor = {}
        self.GeneralSettingDict = {}
        self.FocusStackInfoDict = {}
        
        self.normalOutputWritten('Rounds cleared.\n')
        print('Rounds cleared.')
    #%%
    """
    # =============================================================================
    #     Configure general settings, get ready for execution      
    # =============================================================================
    """
    def ConfigGeneralSettings(self):
        savedirectory = self.savedirectory
        meshrepeat = self.ScanRepeatTextbox.value()
        StageGridOffset = self.ScanStepsNumTextbox.value() * self.ScanstepTextbox.value()
        
        StartUpEvents = []
        if self.OpenTwoPLaserShutterCheckbox.isChecked():
            StartUpEvents.append('Shutter_Open')
        
        # Interpolate in between the focus correction positions
        FocusCorrectionMatrixDict = {}#self.upsize_focus_matrix()
            
        generalnamelist = ['savedirectory', 'FocusCorrectionMatrixDict', 'FocusStackInfoDict', 'StageGridOffset', 'Meshgrid', 'StartUpEvents']
        
        generallist = [savedirectory, FocusCorrectionMatrixDict, self.FocusStackInfoDict, StageGridOffset, meshrepeat, StartUpEvents]
        
        for item in range(len(generallist)):
            self.GeneralSettingDict[generalnamelist[item]] = generallist[item]
#        print(self.GeneralSettingDict['FocusStackInfoDict'])
        self.normalOutputWritten('Rounds configured.\n')
        
        self.show_pipline_infor()
        
    def auto_saving_directory(self):
        self.savedirectory = r'M:\tnw\ist\do\projects\Neurophotonics\Brinkslab\Data\Octoscope\Evolution screening\{}_{}_{}'.format \
                            (date.today(), datetime.now().strftime('%Y-%m-%d_%H-%M-%S'), str(self.prefixtextbox.text()))
        
        os.mkdir(self.savedirectory) # Create the folder
            
    
    def _open_file_dialog(self):
        self.savedirectory = str(QtWidgets.QFileDialog.getExistingDirectory(directory='M:/tnw/ist/do/projects/Neurophotonics/Brinkslab/Data'))
        self.savedirectorytextbox.setText(self.savedirectory)
        self.set_prefix()
    
    def update_saving_directory(self):
        self.savedirectory = str(self.savedirectorytextbox.text())
        
    def set_prefix(self):
        self.saving_prefix = str(self.prefixtextbox.text())
        
    #--------------------------------------------------------------------GenerateFocusCorrectionMatrix-----------------------------------------
    def CaptureFocusCorrectionMatrix(self, CorrectionFomula):
        self.CorrectionFomula = CorrectionFomula
        
    def CaptureFocusDuplicateMethodMatrix(self, CorrectionDictForDuplicateMethod):
        self.FocusDuplicateMethodInfor = CorrectionDictForDuplicateMethod
        
    def ExecutePipeline(self):
        self.Savepipeline()
        
        get_ipython().run_line_magic('matplotlib', 'inline') # before start, set spyder back to inline
        
        self.ExecuteThreadInstance = ScanningExecutionThread(self.RoundQueueDict, self.RoundCoordsDict, self.GeneralSettingDict)
        self.ExecuteThreadInstance.start()
        
    def Savepipeline(self):
        SavepipelineInstance = []
        SavepipelineInstance.extend([self.RoundQueueDict, self.RoundCoordsDict, self.GeneralSettingDict])
        
        np.save(os.path.join(self.savedirectory, self.saving_prefix, datetime.now().strftime('%Y-%m-%d_%H-%M-%S')+'_Pipeline'), SavepipelineInstance)
        
        
    #%%
    """
    # =============================================================================
    #     For save and load file.    
    # =============================================================================
    """
    def GetPipelineNPFile(self):
        self.pipelinenpfileName, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Single File', 'M:/tnw/ist/do/projects/Neurophotonics/Brinkslab/Data',"(*.npy)")
        self.LoadPipelineAddressbox.setText(self.pipelinenpfileName)
        
    def LoadPipelineFile(self):
        temp_loaded_container = np.load(self.pipelinenpfileName, allow_pickle=True)
        self.RoundQueueDict = temp_loaded_container[0]
        self.RoundCoordsDict = temp_loaded_container[1]
        self.GeneralSettingDict = temp_loaded_container[2]

        # Interpolate in between the focus correction positions
        FocusCorrectionMatrixDict = {}#self.upsize_focus_matrix()

        # Refresh the focus correction
        self.GeneralSettingDict['FocusCorrectionMatrixDict'] = FocusCorrectionMatrixDict

        # # if saving directory is re-configured, refresh it, otherwise keep as it is.
        # if len(self.savedirectorytextbox.text()) >= 1:
        #     self.GeneralSettingDict['savedirectory'] = self.savedirectory
        # else:
        self.auto_saving_directory()
        
        self.normalOutputWritten('Pipeline loaded.\n')
        print('Pipeline loaded.')
        
        self.show_pipline_infor()
        
    def show_pipline_infor(self):
        """
        Show general information of the pipeline.

        Returns
        -------
        None.

        """
        self.normalOutputWritten('--------Pipeline general info--------\n')
        for eachround in range(int(len(self.RoundQueueDict)/2-1)):

            #--------------------------------------------------------------
            # show waveform settings
            waveformPackage = self.RoundQueueDict['RoundPackage_'+str(eachround+1)][0]
            camOperationPackage = self.RoundQueueDict['RoundPackage_'+str(eachround+1)][1]
            waveform_sequence = 1
            
            for eachwaveform in waveformPackage:
                try:
                    if len(waveformPackage[eachwaveform][3]) != 0:
                        self.normalOutputWritten('Round {}, sequence {}, recording channels:{}.\n'.format(eachround+1, waveform_sequence, waveformPackage[eachwaveform][3]))
                        print('Round {}, recording channels:{}.'.format(eachround+1, waveformPackage[eachwaveform][3]))#[1]['Sepcification']
#                    else:
#                        self.normalOutputWritten('Round {} No recording channel.\n'.format(eachround+1))
                except:
                    self.normalOutputWritten('No recording channel.\n')
                    print('No recording channel.')
                try:
                    self.normalOutputWritten('Round {}, Analog signals:{}.\n'.format(eachround+1, waveformPackage[eachwaveform][1]['Sepcification']))
                    print('Round {}, Analog signals:{}.'.format(eachround+1, waveformPackage[eachwaveform][1]['Sepcification']))#
                except:
                    self.normalOutputWritten('No Analog signals.\n')
                    print('No Analog signals.')
                try:
                    if len(waveformPackage[2]['Sepcification']) != 0:
                        self.normalOutputWritten('Round {}, Digital signals:{}.\n'.format(eachround+1, waveformPackage[eachwaveform][2]['Sepcification']))
                        self.normalOutputWritten('Lasting time:{} s.\n'.format(len(waveformPackage[eachwaveform][2]['Waveform'][0])/waveformPackage[eachwaveform][0]))
                        
                        print('Lasting time:{} s.\n'.format(len(waveformPackage[eachwaveform][2]['Waveform'][0])/waveformPackage[eachwaveform][0]))
                        print('Round {}, Digital signals:{}.'.format(eachround+1, waveformPackage[eachwaveform][2]['Sepcification']))#
#                    else:
#                        self.normalOutputWritten('Round {} No Digital signals.\n'.format(eachround+1))
                except:
                    self.normalOutputWritten('No Digital signals.\n')
                    print('No Digital signals.')
                waveform_sequence += 1
                self.normalOutputWritten('\n')
                
            for eachcamoperation in camOperationPackage:
                #--------------------------------------------------------------
                # Show camera operations
               
                try:
                    if len(camOperationPackage[eachcamoperation]) != 0:
                        self.normalOutputWritten('Round {}, cam Buffer_number:{}.\n'.format(eachround+1, camOperationPackage[eachcamoperation]['Buffer_number']))
                        print('Round {}, cam Buffer_number:{}.\n'.format(eachround+1, camOperationPackage[eachcamoperation]['Buffer_number']))#
#                    else:
#                        self.normalOutputWritten('Round {} No Digital signals.\n'.format(eachround+1))
                except:
                    self.normalOutputWritten('No camera operations.\n')
                    print('No camera operations.')  
            
            self.normalOutputWritten('-----------end of round-----------\n')
        self.normalOutputWritten('----------------------------------------\n')

    #%%
    """
    # =============================================================================
    #     FUNCTIONS FOR QUICK START
    # =============================================================================
    """
    def quick_start(self, config_number):
        
        # Load pre-saved pipeline
        self.pipelinenpfileName = self.quick_start_location[config_number]
        temp_loaded_container = np.load(self.pipelinenpfileName, allow_pickle=True)
        self.RoundQueueDict = temp_loaded_container[0]
        self.RoundCoordsDict = temp_loaded_container[1]
        self.GeneralSettingDict = temp_loaded_container[2]
        
        self.auto_saving_directory()
        
        self.normalOutputWritten('Pipeline loaded.\n')
        print('Pipeline loaded.')
        
        self.show_pipline_infor()
        
        # Execute
        self.ExecutePipeline()
        
    #---------------------------------------------------------------functions for console display------------------------------------------------------------        
    def normalOutputWritten(self, text):
        """Append text to the QTextEdit."""
        # Maybe QTextEdit.append() works as well, but this is how I do it:
        cursor = self.ConsoleTextDisplay.textCursor()
        cursor.movePosition(QTextCursor.End)
        cursor.insertText(text)
        self.ConsoleTextDisplay.setTextCursor(cursor)
        self.ConsoleTextDisplay.ensureCursorVisible()
        
    def openScreenAnalysisMLWidget(self):
        from ImageAnalysis import EvolutionAnalysisWidget
        
        self.ScreenAnalysisMLWindow = EvolutionAnalysisWidget.MainGUI()
        self.ScreenAnalysisMLWindow.show()
    #%%
if __name__ == "__main__":
    def run_app():
        app = QtWidgets.QApplication(sys.argv)
        QtWidgets.QApplication.setStyle(QStyleFactory.create('Fusion'))
        mainwin = Mainbody()
        mainwin.show()
        app.exec_()
    run_app()