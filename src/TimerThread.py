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
            self.parent.ok = True
            try:
                self.parent.MoveRobotTo( (random.randrange(1950,2050,1),
                                          random.randrange(1950,2050,1)), 
                                         (0,1), False)
                print "moved robot"
            except AttributeError:
                pass

    def SetInterval(self, interval):
        self.interval = interval

                