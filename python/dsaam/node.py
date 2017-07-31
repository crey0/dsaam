from __future__ import print_function, division, absolute_import
from queue import Queue
from heapq import heappop, heappush
from threading import Lock, Semaphore, Thread, Event
from copy import copy
from collections import Sequence

NS_IN_SECOND = 1000000000

class HeapQ():
    def __init__(self):
        self.heap = []

    def push(self):
        heappush(self.heap)
        return self.__len__()

    def pop(self):
        heappop(self.heap)
        return self.__len__()

    def __len__(self):
        return len(self.heap)

    
class Time:
    def __init__(self, sec=0, nanos=0):
        assert(type(sec) is int)
        assert(type(nanos) is int)
        
        self.sec = sec
        self.nanos = nanos

    def __call__(self, other):
        assert(type(other) is Time)
        self.sec = sec
        self.nanos = nanos

    def __add__(self, other):
        nanos = self.nanos % NS_IN_SECOND + other.nanos % NS_IN_SECOND
        carry = nanos // NS_IN_SECOND + self.nanos //  NS_IN_SECOND + other.nanos // NS_IN_SECOND
        return Time(self.sec + other.sec + carry, nanos % NS_IN_SECOND)

    def __sub__(self, other):
        nanos = self.nanos % NS_IN_SECOND - other.nanos % NS_IN_SECOND
        carry = nanos // NS_IN_SECOND + self.nanos //  NS_IN_SECOND - other.nanos // NS_IN_SECOND
        return Time(self.sec - other.sec + carry, nanos % NS_IN_SECOND)

    def __mul__(self, other):
        nanos = self.nanos % NS_IN_SECOND * other
        carry = nanos // NS_IN_SECOND + self.nanos // NS_IN_SECOND
        return Time(self.sec * other + carry, nanos % NS_IN_SECOND)
    
    def __lt__(self, other):
        return self.sec < other.sec or (self.sec == other.sec and self.nanos < other.nanos)

    def __le__(self, other):
        return self.__lt__(other) or self.__eq__(other)

    def __eq__(self, other):
        return self.sec == other.sec and self.nanos == other.nanos

    def __ne__(self, other):
        return not self.__eq__(other)

    def __gt__(self, other):
        return not self.__le__(other)

    def __ge__(self, other):
        return not self.__lt__(other)

    def __repr__(self):
        return "Time(sec={}, nanos={})".format(self.sec, self.nanos)

    def __str__(self):
        return "{}:{}".format(self.sec, self.nanos)



class NamedMutableSequence(Sequence):
    __slots__ = ()

    def __init__(self, *a, **kw):
        slots = self.__slots__
        for k in slots:
            setattr(self, k, kw.get(k))

        if a:
            for k, v in zip(slots, a):
                setattr(self, k, v)

    def __str__(self):
        clsname = self.__class__.__name__
        values = ', '.join('%s=%r' % (k, getattr(self, k))
                           for k in self.__slots__)
        return '%s(%s)' % (clsname, values)

    __repr__ = __str__

    def __getitem__(self, item):
        return getattr(self, self.__slots__[item])

    def __setitem__(self, item, value):
        return setattr(self, self.__slots__[item], value)

    def __len__(self):
        return len(self.__slots__)

def namedlist(name, members):
    if isinstance(members, str):
        members = members.split()
    members = tuple(members)
    return type(name, (NamedMutableSequence,), {'__slots__': members})

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
    
        
InFlow = namedlist('InFlow', ['name', 'dt', 'time_callback'])
OutFlow = namedlist('OutFlow', ['name','dt', 'sinks'])
Sink = namedlist('Sink', ['name', 'callback'])

class MessageFlowMultiplexer:
    def __init__(self, flows, time):
        self.time = copy(time)
        self.tmin_next = copy(time)
        self.flows = flows
        self.queues = []
        for flow in flows:
            self.queues.append(MessageQueue(flow.name, Time() - flow.dt, flow.dt))

    def push(self, flow, m, time):
        self.queues[flow].push(m, time)

    def pop(self):
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

        if idx < 0 or idx >= len(self.queues):
            print("ERROOOR Bad index {} len={}".format(idx,len(self.queues) ))
            return "None", None, None
        q = self.queues[idx]
        m = q.pop()
        if self.flows[idx].time_callback is None:
            print("[{}] ERROOOR NO callback for flow {}".format(self.time, self.flows[idx].name))
            return q.name, None, None
        self.flows[idx].time_callback(m[1])
        self.tmin_next = tmin_next
        return q.name, m, tmin_next

    def nextTime(self):
        return self.tmin_next
            
class OutMessageFlow:
    def __init__(self, name, time, dt, sinks, max_qsize):
        self.name = name
        self.time = time-dt
        self.dt = copy(dt)
        self.sinks = sinks
        self.sink_times = dict([(s.name, [time-dt, Semaphore(max_qsize)]) for s in sinks])
        #self.min_time = copy(time)
        self.lock = Lock()
        self.max_qsize = max_qsize

    def push_time(self, name, time):
        with self.lock:
            t, s = self.sink_times[name]
            assert(t < time)
            self.sink_times[name][0] = copy(time)
            #if time < self.min_time:
            #    self.min_time = copy(time)
            s.release()
                
    def send(self, m, time):
        for _, s in self.sink_times.values():
            s.acquire()
        with self.lock:
            assert(self.time + self.dt <= time)
            #assert(self.min_time + self.dt * self.max_qsize >= time)
            self.time = time
            for s in self.sinks:
                if s.callback is not None:
                    s.callback(m, time)

class Node:
    def __init__(self, name, time, dt, inflows, outflows, max_qsize):
        """
        inflows: list InFlow tuple each containing: name, dt
        outflows: list of OutFlow tuple each containing name, dt, list of sinks
        """
        super().__init__()
        assert(type(time) is Time)
        assert(type(dt) is Time)
        self.name = name
        self.time = copy(time)
        self.dt = copy(dt)
        self.inflows = MessageFlowMultiplexer(inflows, time)
        if len(inflows) == 0:
            self.next = lambda: (None, (None, self.time, self.dt), self.time + self.dt)
            self.inflows.nextTime = lambda: self.time + self.dt

        self.outflows = dict([(f.name, OutMessageFlow(f.name, time, f.dt, f.sinks, max_qsize))
                              for f in outflows])
        self.indexes = dict([(f.name, idx) for idx, f in enumerate(inflows)])

    def push(self, flow, m, time):
        print("[{}] [{}] pushing IN on flow {}".format(self.time, self.name, flow, self.indexes))
        fidx = self.indexes[flow]
        self.inflows.push(fidx, m, time)

    def push_callback(self, flow):
        return lambda m, time: self.push(flow, m, time)

    def time_callback(self, flow, sink):
        return lambda time: self.outflows[flow].push_time(sink, time)
    
    def next(self):
        name, m, tmin_next = self.inflows.pop()
        if m is None:
            print("[{}] [{}] ERROOOR happened in node".format(self.time, self.name))
            exit(0)
        print("[{}] [{}] IN message from {} (at {}, next at {})"
              .format(self.time, self.name, name, m[1], tmin_next))
        return name, m, tmin_next
        

    def send(self, flow, m, time):
        assert(time >= self.time)
        if self.inflows.nextTime() < time:
            raise AssertionError("[{}] [{}] Attempt to send at {} but next is at {}"
                                 .format(self.time, self.name, time, self.inflows.nextTime()))
        print("[{}] [{}] OUT message on  {}".format(time, self.name, flow))
        self.outflows[flow].send(m, time)

    def send_callback(self, flow):
        return lambda m, time: self.send(flow, m, time)        

    def step(self, time):
        assert(time >= self.time)
        self.time = copy(time)
    
class OneThreadNode():
    def __init__(self, node, *args, **kwargs):
        self.node = node
        self.send = self.node.send
        self.send_callback = self.node.send_callback
        
        self.stop_event = Event()
        self.worker = Thread(name=self.name, target=self, daemon=True)
        if "callback" in kwargs:
            cb = kwargs["callback"]
            self.process = cb
        if "init" in kwargs:
            init = kwargs["init"]
            self.init = init
        

    @property
    def name(self):
        return self.node.name

    @property
    def dt(self):
        return self.node.dt

    @property
    def time(self):
        return self.node.time

    def start(self):
        self.worker.start()
        
    def stop(self):
        self.stop_event.set()

    def join(self, timeout=0):
        return self.worker.join(timeout)

    def isAlive(self):
        return self.worker.isAlive()
        
    def __call__(self):
        self.init()
        while not self.stop_event.is_set():
            v = self.node.next()
            t = self.process(*v)
            if t is not None:
                self.node.step(t)
            

    def init(self):
        raise NotImplementedError("Please implement process method in your subclass")
    
    def process(self, name, m, tmin_next):
        raise NotImplementedError("Please implement process method in your subclass")
