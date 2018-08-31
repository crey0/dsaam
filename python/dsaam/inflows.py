from __future__ import print_function, division, absolute_import
from copy import copy
from .message_queue import MessageQueue
from .binary_heap import BinaryHeap
from .time import Time

class HQValue:
        def __init__(self, flow, queue):
            self.flow = flow
            self.queue = queue
            self.handle = None

class MessageFlowMultiplexer:

    
            
    def __init__(self, flows, time):
        self.time = copy(time)
        self.tmin_next = copy(time)
        mq_greater = lambda a, b: a.queue.nextTime() > b.queue.nextTime()
        self.qheap = BinaryHeap(mq_greater)
        self.queues = []
        for flow in flows:
            self.queues.push(MessageQueue(flow.name, flow.time, flow.dt))
            h = self.qheap.push(HQValue(flow, queue))
            h.value.handle = handle
    
    def push(self, flow_index, m, time):
        self.queues[flow_index].push(m, time)

    def next(self):
        q = self.qheap.top()
        m = q.queue.pop()
        self.qheap.siftdown(q.handle)
        
        #assert self.flows[idx].time_callback is not None,\
        #    "No time-callback for flow {}".format(self.time, self.flows[idx].name)
        
        q.flow.time_callback(m[1])
        self.tmin_next = self.qheap.top().queue.nextTime()
        return q.flow.callback(q.flow.name, m, self.tmin_next)

    def next_time(self):
        return self.tmin_next

    def setup_inflow(self, flow):
        self.queues.append(MessageQueue(flow.name, flow.time, flow.dt))
        h = self.qheap.push(HQValue(flow, self.queues[-1]))
        h.value.handle = h

    def __len__(self):
        return len(self.queues)
        
