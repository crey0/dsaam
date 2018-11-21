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
from copy import copy
from threading import Lock, Semaphore

class OutMessageFlow:
    def __init__(self, name, time, dt, sinks, qsize, ftype):
        self.name = name
        self.time = time
        self.dt = copy(dt)
        self.sinks = sinks
        self.sink_times = dict([(s.name, [time, Semaphore(qsize)]) for s in sinks])
        self.lock = Lock()
        self.qsize = qsize
        self.ftype = ftype

    def push_time(self, name, time):
        with self.lock:
            t, s = self.sink_times[name]
            assert t == time, "Flow {} : time contract breached, ACK "\
                "of {} indicates messages are not processed in order: was {}, "\
                "update in the past at {}"\
                .format(self.name, name, t, time)

            self.sink_times[name][0] = t + self.dt
            s.release()
                
    def send(self, m, time):
        for _, s in self.sink_times.values():
            s.acquire()
        with self.lock:
            assert self.time == time,\
                "Flow {} : time contract breached for next message, should be "\
                "at {} but is at {}"\
                .format(self.name, self.time, time)

            self.time = time + self.dt
            for s in self.sinks:
                if s.callback is not None:
                    s.callback(m, time)

    def setup_sink(sink):
        self.sinks.append(sink)
        self.sink_times[s.name] =  [self.time, Semaphore(qsize)]
