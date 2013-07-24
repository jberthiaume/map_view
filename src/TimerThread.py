#!/usr/bin/env python

import time
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

    def SetInterval(self, interval):
        self.interval = interval

                