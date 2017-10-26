from __future__ import print_function, division, absolute_import
from copy import copy
from .message_queue import MessageQueue
from .time import Time

class MessageFlowMultiplexer:
    def __init__(self, flows, time):
        self.time = copy(time)
        self.tmin_next = copy(time)
        self.flows = flows
        self.queues = []
        for flow in flows:
            self.queues.append(MessageQueue(flow.name, flow.time - flow.dt, flow.dt))

    def push(self, flow_index, m, time):
        self.queues[flow_index].push(m, time)

    def next(self):
        tmin = None
        tmin_next = None
        idx = -1
        for i,f in enumerate(self.queues):
            nt = f.nextTime()
            dt = f.dt
            if tmin is None or nt < tmin:
                if tmin is not None:
                    tmin_next = tmin
                if tmin_next is None or nt + dt < tmin_next:
                    tmin_next = nt + dt
                tmin = nt
                idx = i
            elif nt < tmin_next:
                tmin_next = nt

        q = self.queues[idx]
        m = q.pop()
        
        #assert self.flows[idx].time_callback is not None,\
        #    "No time-callback for flow {}".format(self.time, self.flows[idx].name)
        
        self.flows[idx].time_callback(m[1])
        self.tmin_next = tmin_next
        return self.flows[idx].callback(q.name, m, tmin_next)

    def next_time(self):
        return self.tmin_next

    def setup_inflow(self, flow):
        self.flows.append(flow)
        self.queues.append(MessageQueue(flow.name, flow.time - flow.dt, flow.dt))
