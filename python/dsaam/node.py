from __future__ import print_function, division, absolute_import
from copy import copy

from .common import InFlow, OutFlow, Sink
from .time import Time
from .inflows import MessageFlowMultiplexer
from .outflows import OutMessageFlow

class Node:
    def __init__(self, name, time, dt, max_qsize, inflows=[], outflows=[]):
        """
        inflows: (optional) list InFlow tuple each containing: name, dt
        outflows: (optional) list of OutFlow tuple each containing name, dt, list of sinks
        """
        super().__init__()
        assert type(time) is Time, "Expected type Time  for parameter time, got {}".format(type(time))
        assert type(dt) is Time, "Expected type Time  for parameter dt, got {}".format(type(dt))
        self.name = name
        self.time = copy(time)
        self.dt = copy(dt)
        self.default_qsize = max_qsize

        #NTS avoid problems : never use default parameter for list (shared object) 
        inflows = copy(inflows)
        outflows = copy(outflows)
        self.inflows = MessageFlowMultiplexer(inflows, time)
        self.indexes = dict([(f.name, idx) for idx, f in enumerate(inflows)])
        self.outflows = dict([(f.name, OutMessageFlow(f.name, time, f.dt,
                                                      f.sinks,
                                                      max_qsize))
                              for f in outflows])

    #TODO cleanup this mess
    def init(self):
         if len(self.inflows.flows) == 0:
            self.next = lambda: (None, (None, self.time, self.dt),
                                 self.time + self.dt)
            self.inflows.nextTime = lambda: self.time + self.dt
        
    def setup_inflow(self, flow):
        self.inflows.setup_inflow(flow)
        self.indexes[flow.name] = len(self.inflows.flows) - 1

    def setup_outflow(self, flow):
        assert(flow.name not in self.outflows)
        self.outflows[flow.name] = OutMessageFlow(flow.name, self.time, flow.dt,
                                                  flow.sinks,
                                                  self.default_qsize)

    def setup_sink(self, flow_name, sink):
       self.outflows[flow_name].setup_sink(sink)

    def step(self, time):
        assert time >= self.time,\
            "Time contract breached: cannot step into the past, time was {} trying"\
            "to step at {}".format(self.time, time)
#        print("[{}] [{}] Stepping to {}".format(self.time, self.name, time))
        self.time = copy(time)
        
    def push(self, flow, m, time):
        print("[{}] [{}] pushing IN on flow {}".format(self.time, self.name, flow, self.indexes))
        fidx = self.indexes[flow]
        self.inflows.push(fidx, m, time)

    def push_callback(self, flow):
        return lambda m, time: self.push(flow, m, time)

    def time_callback(self, flow, sink):
        return lambda time: self.outflows[flow].push_time(sink, time)
    
    def next(self):
        print("[{}] next".format(self.name))
        return self.inflows.next()

    def send(self, flow, m, time):
        assert time >= self.time,\
            "Time contract breached: sending in the past is forbiden, message at {},"\
            " self time is {}".format(time, self.time)
        assert self.inflows.nextTime() >= time,\
            "Time contract breached: attempt to send outgoing message at {} but expecting "\
            "next incoming message at {}".format(time, self.inflows.nextTime())

        print("[{}] [{}] OUT message on  {} at {}".format(self.time, self.name, flow, time))
        self.outflows[flow].send(m, time)

    def send_callback(self, flow):
        return lambda m, time: self.send(flow, m, time)        
