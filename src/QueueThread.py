#!/usr/bin/env python

import time
import threading
from threading import Thread

class QueueThread(Thread):
    def __init__(self, parent):
        self.stopped = False
        self.parent = parent
        Thread.__init__(self)

    def run(self):
        while not self.stopped:
            item = self.parent.q.get()            
            try:
                fn = item[0]
                args = item[1:]
                print str(fn)[13:].split()[0]
                fn(*args)
                
            except TypeError:
                fn = item
                print str(fn)[13:].split()[0]
                fn()