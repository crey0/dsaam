from __future__ import print_function, division, absolute_import
from threading import Lock, Semaphore
from copy import copy
from .time import Time

class MessageQueue:
    def __init__(self, name, time, dt):
        assert(type(time) is Time)
        assert(type(dt) is Time)
        self.time = copy(time)
        self.dt = copy(dt)
        self.name = name
        self.queue = []
        self.qlock = Lock()
        self.qsem = Semaphore(0)

    def push(self, m, time):
        with self.qlock:
            self.queue.append((m,time))
            self.qsem.release()

    def pop(self):
        self.qsem.acquire()
        with self.qlock:
            m, time = self.queue.pop(0)
            self.time = copy(time)
            return m, time, self.dt

    def nextTime(self):
        with self.qlock:
            return self.time + self.dt
    
    def lastTime(self):
        with self.qlock:
            return self.time, self.dt
