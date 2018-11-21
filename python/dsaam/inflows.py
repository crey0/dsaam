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
        
