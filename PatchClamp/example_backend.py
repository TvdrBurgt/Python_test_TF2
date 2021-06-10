# -*- coding: utf-8 -*-
"""
Created on Thu Apr 15 11:12:26 2021

@author: tvdrb
"""

import numpy as np

from copy import copy

from PyQt5.QtCore import pyqtSignal, pyqtSlot, QThread, QMutex


class CameraThread(QThread):
    snapsignal = pyqtSignal(np.ndarray)
    livesignal = pyqtSignal(np.ndarray)

    def __init__(self):
        self.isrunning = False
        self.exposure_time = 20
        self.get_frame = np.zeros((2048, 2048))

        super().__init__()
        self.mutex = QMutex()
        self.moveToThread(self)
        self.started.connect(self.acquire)

    def __del__(self):
        self.isrunning = False
        self.quit()
        self.wait()

    @pyqtSlot()
    def acquire(self):
        self.isrunning = True
        print("data acquisition started")
        while self.isrunning:
            self.mutex.lock()
            self.get_frame = np.random.rand(2048, 2048)
            QThread.msleep(self.exposure_time)
            self.livesignal.emit(self.get_frame)
            self.mutex.unlock()
        print("data acquisition stopped")

    def snap(self):
        # Mutex lock makes code execute in order. Code continues when unlocked,
        # so it acts as a que.
        self.mutex.lock()
        last_view = copy(self.get_frame)
        self.mutex.unlock()
        print("snap!")
        self.snapsignal.emit(last_view)

        return last_view


class AutoPatchThread(QThread):
    finished = pyqtSignal()

    def __init__(self, method, camera_handle=None, manipulator_handle=None):
        self.camera = camera_handle
        self.micromanipulator = manipulator_handle
        self.isrunning = False

        super().__init__()
        self.finished.connect(self.__del__)
        self.moveToThread(self)
        if method == "request autofocus":
            self.started.connect(self.autofocus)
        elif method == "request detect":
            self.started.connect(self.detect)
        else:
            pass

    def __del__(self):
        self.isrunning = False
        self.quit()
        self.wait()

    @pyqtSlot()
    def autofocus(self):
        self.isrunning = True
        print("autofocus started")
        loopcount = -1
        while self.isrunning:
            loopcount += 1
            QThread.msleep(1000)
            print(loopcount)
            if loopcount == 5:
                self.isrunning = False
        self.finished.emit()
        print("autofocus stopped")

    @pyqtSlot()
    def detect(self):
        self.isrunning = True
        print("pipette detection started")
        loopcount = -1
        while self.isrunning:
            loopcount += 1
            QThread.msleep(1000)
            print(loopcount)
            if loopcount == 10:
                self.isrunning = False
        print("pipette detection stopped")
        self.finished.emit()


if __name__ == "__main__":
    instance = AutoPatchThread()
