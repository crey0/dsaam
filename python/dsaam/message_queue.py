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
            assert time  == self.time, "Time contract breached in : Invalid message"\
                "in queue {}, time expected {} got {} ".format(self.name, self.time, time)
            self.time += self.dt
            return m, time, self.dt

    def nextTime(self):
        return self.time
