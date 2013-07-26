#!/usr/bin/env python

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
                self.parent.q.put( (self.parent.MoveRobotTo, self.parent.ros.r_destination, 
                                         self.parent.ros.r_orient, True) )
#                 print "moved robot"
            except AttributeError:
                pass

    def SetInterval(self, interval):
        self.interval = interval

                