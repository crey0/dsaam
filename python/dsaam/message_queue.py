# Copyright © 2018 CNRS
# All rights reserved.

# @author Christophe Reymann

#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:

#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.

#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
#  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
#  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
#  BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

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
