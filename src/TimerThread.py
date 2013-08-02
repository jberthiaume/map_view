#!/usr/bin/env python

import wx
import time
import random
from threading import Thread

class TimerThread(Thread):
    def __init__(self, parent, interval):
        self.stopped = False
        self.parent = parent
        self.interval = interval
        Thread.__init__(self)

    def run(self):
        while not self.stopped:
            time.sleep(self.interval)
            try:                
#                 self.parent.ok = True
                self.DoStuff()
                print "moved robot"
            except AttributeError:
                pass

    def SetInterval(self, interval):
        self.interval = interval
        
    def DoStuff(self):
        wx.CallAfter(self.parent.MoveRobotTo, (random.randrange(1900,2100,1), 
                             random.randrange(1700,1900,1)), (0,1), False)
        pass
                