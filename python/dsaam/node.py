from __future__ import print_function, division, absolute_import
from copy import copy

from .common import InFlow, OutFlow, Sink, FlowType
from .time import Time
from .inflows import MessageFlowMultiplexer
from .outflows import OutMessageFlow
from .binary_heap import BinaryHeap

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
        self.outflows = dict([(f.name, OutMessageFlow(f.name, flow.time, f.dt,
                                                      f.sinks,
                                                      max_qsize))
                              for f in outflows])

    def init(self):
        #TODO cleanup this mess
        if len(self.inflows) == 0:
            self.next = lambda: self.time + self.dt
            self.inflows.next_time = lambda: self.time + self.dt

        self.qnext_out = BinaryHeap(compare=lambda l,r:
                                    l.time > r.time
                                    or (l.time == l.time
                                        and l.ftype == FlowType.OBS
                                        and r.ftype == FlowType.PRED))
        self.qnext_out_handles = {}
        for f in self.outflows.values():
            h = self.qnext_out.push(f)
            self.qnext_out_handles[f.name] = h
        
        
    def setup_inflow(self, flow):
        flow.qsize = [self.default_qsize, flow.qsize][flow.qsize > 0]
        self.inflows.setup_inflow(flow)
        self.indexes[flow.name] = len(self.inflows) - 1

    def setup_outflow(self, flow):
        assert flow.name not in self.outflows,\
            "There is already an outgoing flow with name {}.".format(flow.name)
        assert isinstance(flow.ftype, FlowType),\
            "Flow type must be a dsaam.common.FlowType instance"
        
        flow.qsize = [self.default_qsize, flow.qsize][flow.qsize > 0]
        self.outflows[flow.name] = OutMessageFlow(flow.name, flow.time, flow.dt,
                                                  flow.sinks,
                                                  flow.qsize,
                                                  flow.ftype)

    def setup_sink(self, flow_name, sink):
       self.outflows[flow_name].setup_sink(sink)

    def step(self, time):
        assert time >= self.time,\
            "Time contract breached: cannot step into the past, time was {} trying"\
            "to step at {}".format(self.time, time)
        
        #print("[{}] [{}] Stepping to {}".format(self.time, self.name, time))
        self.time = copy(time)
        
    def push(self, flow, m, time):
        #print("[{}] [{}] pushing IN on flow {}".format(self.time, self.name, flow, self.indexes))
        fidx = self.indexes[flow]
        self.inflows.push(fidx, m, time)

    def push_callback(self, flow):
        return lambda m, time: self.push(flow, m, time)

    def time_callback(self, flow, sink):
        return lambda time: self.outflows[flow].push_time(sink, time)
    
    def next(self):
        next_in = self.inflows.next_time()
        next_out = self.next_out()
        assert next_out is None\
            or (next_out.ftype is FlowType.OBS and next_in <= next_out.time)\
            or (next_out.ftype is FlowType.PRED and next_in < next_out.time),\
            "Time contract breached: attempting to consume next message at {} "\
            "but next outgoing message should be sent at {}."\
            .format(next_in, next_out.time)
        return self.inflows.next()
    
    def next_time(self):
        return self.inflows.next_time()

    def next_out(self):
        return  self.qnext_out.top() if len(self.outflows) != 0 else None 


    def send(self, flow, m, time):
        oflow = self.outflows[flow]
        next_in = self.inflows.next_time()
        assert time >= self.time,\
            "Time contract breached: sending in the past is forbiden, message at {},"\
            " self time is {}".format(time, self.time)
        assert len(self.inflows) == 0\
            or (oflow.ftype is FlowType.PRED and next_in >= time)\
            or (oflow.ftype is FlowType.OBS  and next_in >  time),\
            "Time contract breached: attempt to send outgoing message on flow {} type {} "\
            "at {} but expecting next incoming message at {}"\
            .format(flow, oflow.ftype, time, next_in)

        #print("[{}] [{}] OUT message on  {} at {}".format(self.time, self.name, flow, time))
        oflow.send(m, time)
        self.qnext_out.siftdown(self.qnext_out_handles[oflow.name])

    def send_callback(self, flow):
        return lambda m, time: self.send(flow, m, time)        
