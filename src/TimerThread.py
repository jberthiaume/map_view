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
#                 self.DoStuff()
#                 print "moved robot"
                
                self.parent.ok = True
            except AttributeError:
                pass

    def SetInterval(self, interval):
        self.interval = interval
        
    def DoStuff(self):
#         wx.CallAfter(self.parent.MoveRobotTo, (random.randrange(1900,2100,1), 
#                              random.randrange(1900,2100,1)), (0,1), False)
        pass
                