#!/usr/bin/env python

import time
import threading
import Queue
from threading import Thread

class QueueThread(Thread):
    def __init__(self, parent):
        self.stopped = False
        self.parent = parent
        Thread.__init__(self)

    def run(self):
        while not self.stopped:
            try:
                item = self.parent.q.get(timeout=1)
            except Queue.Empty:
                continue     
            try:
                fn = item[0]
                args = item[1:]
                if self.parent.modes['verbose']:
                    print str(fn)[13:].split()[0] + "()"
                fn(*args)
            except TypeError:
                fn = item
                if self.parent.modes['verbose']:
                    print str(fn)[13:].split()[0] + "()"
                fn()
               
            self.parent.q.task_done()