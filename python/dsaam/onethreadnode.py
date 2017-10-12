from __future__ import print_function, division, absolute_import

from threading import Thread, Event

class OneThreadNode():
    def __init__(self, node, *args, **kwargs):
        self.node = node
        
        #import needed node setup methods
        self.send = self.node.send
        self.send_callback = self.node.send_callback
        self.push_callback = self.node.push_callback
        self.time_callback = self.node.time_callback
        self.setup_inflow = self.node.setup_inflow
        self.setup_outflow = self.node.setup_outflow
        self.setup_sink = self.node.setup_sink
        
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
        self.node.init()
        self.init()
        while not self.stop_event.is_set():
            t = self.node.next()
            if t is not None:
                self.node.step(t)
            

    def init(self):
        raise NotImplementedError("Please implement process method in your subclass")
