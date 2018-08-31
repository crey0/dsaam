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
