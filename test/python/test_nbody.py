from dsaam.node import Node, InFlow, OutFlow, Sink, Time, FlowType
from dsaam.onethreadnode import OneThreadNode

import numpy as np
from copy import deepcopy
from threading import Event
from queue import Queue
import matplotlib
import matplotlib.pyplot as plt
import signal, os
from time import sleep
from time import time as systime

stop_event = Event()
except_event = Event()
excepts = Queue()

def exception_collect(fun):
    def _fun(*args, **kwargs):
        try:
            return fun(*args, **kwargs)
        except BaseException as e:
            excepts.put(e)
            except_event.set()
            stop_event.set()
            raise e
    return _fun

def handler_sigint(signum, frame):
    print("[master] Caught SIGINT : setting stop event.")
    stop_event.set()

G = 1.5 * 1.80 * 1e-11
C = 8 * 1e-3
eps = 1e-3
colors = ['red', 'green', 'blue', 'yellow']
colors_display = {'red':'firebrick',
                  'green':'darkolivegreen',
                  'blue':'midnightblue',
                  'yellow':'goldenrod'}
class Body:
    def __init__(self, name, p0, v0, mass, radius, color):
        self.name = name
        self.p = np.array(p0, dtype=float)
        self.v = np.array(v0, dtype=float)
        self.mass = float(mass)
        self.radius = float(radius)
        self.color = color

    def acceleration(self, p_others, m_others):
        if p_others.shape[0] == 0:
            return 0.
        d =  p_others - self.p[None, :]
        d1 = -1*np.sign(d)*(1-np.abs(d))
        f = np.abs(d1) < np.abs(d)
        d[f] = d1[f]
        denom = np.sum(np.abs(d) ** 3 + eps, axis=1)
        a = G * np.sum( (m_others / denom)[:, None] * d, axis=0)
        return a

    def integrate(self, p_others, m_others, dt):
        a = self.acceleration(p_others, m_others)
        self.p = (self.p + self.v * dt) % 1
        self.v = self.v + a * dt
        #max speed
        n = np.linalg.norm(self.v, ord=2)
        if n > C:
            self.v = self.v / n * C
    
    def state(self):
        return self.p, self.v
        
class System:
    def __init__(self, bodies):
        self.bodies = bodies

    def updateState(self, idx, s):
        p, v = s
        b = self.bodies[idx]
        if p is not None:
            b.p = p
        if v is not None:
            b.v = v
        
class SystemOne(System):
    def __init__(self, theone, bodies):
        self.theone = theone
        self.bodies = bodies
        self.m_others = np.array([b.mass for b in bodies.values() if b.name != self.theone])
        
    def oneState(self):
        return self.bodies[self.theone].state()

    def integrateOne(self, dt):
        p_others = np.array([b.p for b in self.bodies.values() if b.name != self.theone])
        self.bodies[self.theone].integrate(p_others, self.m_others, dt)

class SystemDrawer:
    def __init__(self, system, size, scale, headless=False):
        self.system = system
        self.size = size
        self.scale = scale
        self.headless = headless
        self.figure = None
        self.scatter = None
        self.bodies = {c: [b for b in self.system.bodies.values() if b.color==c]
                       for c in colors}

    def scatter_data(self):
       pos = [np.reshape([b.p  for b in self.bodies[c]], (-1, 2)) for c in colors]
       size = [np.reshape([b.radius for b in self.bodies[c]], (-1,)) for c in colors]
       return pos, size

    def init(self):
        global colors
        if self.headless:
            plt.ioff()
            from matplotlib.figure import Figure
            from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
            self.figure = Figure()
            canvas  = FigureCanvas(self.figure)
        else:
            self.figure = plt.figure(figsize=self.size)
        self.axes = self.figure.add_axes([0, 0, 1, 1],frameon=True)
        pos, size = self.scatter_data()
        self.scatter = [self.axes.scatter(pos[c][:, 0], pos[c][:, 1], size[c],
                                          color=colors_display[color])
                        for c, color in enumerate(colors)]
        self.axes.set_xlim(0,1)
        self.axes.set_ylim(0,1)
        #self.axes.grid(True)
        self.axes.set_aspect('equal')

        if not self.headless:
            plt.ion()
            plt.show(block=False)

        
    def draw(self):
        global colors
        # Update the scatter collection, with the new sizes and positions.
        pos, size = self.scatter_data()
        for s, c in zip(self.scatter, range(len(colors))):
            s.set_sizes(size[c])
            s.set_offsets(pos[c])
        self.figure.canvas.draw()
   
class SystemOneNode(OneThreadNode):
    def __init__(self, systemone, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.systemone = deepcopy(systemone)

    @exception_collect
    def init(self):
        stateOne = self.systemone.oneState()
        name = self.systemone.theone
        self.send(name, stateOne, self.time)

    @exception_collect
    def process(self, name, m, tmin_next):
        #print("[{}] [{}] Process {} next at {}"\
        #      .format(self.time, self.name, name, tmin_next))
        idx = name
        state, time, dt = m
        if idx is not None:
            self.systemone.updateState(idx, state)
        
        newTime = self.time
        while (tmin_next >= newTime + self.dt):
            #print("[{}] [{}] Computing next step {} -- > {}"\
            #      .format(self.time, self.name, newTime, newTime+self.dt))
            dt_secf = float(self.dt.sec) + float(self.dt.nanos)*1e-9
            self.systemone.integrateOne(dt_secf)
            stateOne = self.systemone.oneState()
            name = self.systemone.theone
            newTime = newTime + self.dt
            self.send(name, stateOne, newTime)

        #if newTime > self.time:
        #    print("[{}] [{}] Stepping at {}".format(self.time, self.name, newTime))
        return newTime
        
            

class DrawerNode(OneThreadNode):
    def __init__(self, systemdrawer,*args, **kwargs):
        super().__init__(*args, **kwargs)
        self.systemdrawer = deepcopy(systemdrawer)

    @exception_collect
    def init(self):
        self.systemdrawer.init()

    @exception_collect
    def process(self, name, m, tmin_next):
        idx = name
        state, time, dt = m
        self.systemdrawer.system.updateState(idx, state)

        newTime = self.time
        while tmin_next >= newTime + self.dt:
            newTime = newTime + self.dt

        if newTime > self.time:
            print("[{}] [{}] redrawing picture".format(newTime, self.name))
            self.systemdrawer.draw()

        return newTime

def test_nbody(auto=True):
    global stop_event

    stop_time = Time(10)
    
    colors = ['red', 'green', 'blue', 'yellow']
    #nred, ngreen, nblue, nyellow = 1, 7, 11, 1
    nred, ngreen, nblue, nyellow = 1, 2, 3, 5
    #idx = {'red': range(0, nred),
    #       'green':range(nred, nred+ngreen),
    #       'blue': range(nred+ngreen ,nred+ngreen+nblue),
    #       'yellow':range(nred+ngreen+nblue, nred+ngreen+nblue+nyellow)}

    idx = {'red': ['red-{}'.format(i) for i in range(nred)],
           'green':['green-{}'.format(i) for i in range(ngreen)],
           'blue': ['blue-{}'.format(i) for i in range(nblue)],
           'yellow':['yellow-{}'.format(i) for i in range(nyellow)]}
    red_p, red_v = np.random.random((nred,2)), np.zeros((nred,2))
    red_p[0] = [0.5, 0.5]
    green_p, green_v = np.random.random((ngreen,2)), \
                       0.1 * C * np.random.random((ngreen,2)) - 0.05*C
    blue_p, blue_v = np.random.random((nblue,2)), \
                     0.1 * C * np.random.random((nblue,2))
    yellow_p, yellow_v = np.random.random((nyellow,2)), \
                         0.1 * C * np.random.random((nyellow,2)) - 0.05*C

#    red_m, green_m, blue_m, yellow_m = \
 #       (1+np.random.random(nred))*100000,\
 #       (1+np.random.random(ngreen))*100,\
 #       (1+np.random.random(nblue))*0.01,\
 #       np.ones(nyellow)*0.0001,\
    red_m, green_m, blue_m, yellow_m = \
        (1+np.random.random(nred))*10000,\
        (1+np.random.random(ngreen))*1000,\
        (1+np.random.random(nblue))*100,\
        np.ones(nyellow)*100,\

    red_r, green_r, blue_r, yellow_r = \
        red_m*0.1,\
        green_m*0.1,\
        blue_m*0.1,\
        yellow_m*0.1\

    
    
    bodies = {'red':{idx: Body(idx, red_p[i], red_v[i], red_m[i], red_r[i],
                        'red')
                    for i, idx in enumerate(idx['red'])},
              'green':{idx: Body(idx, green_p[i], green_v[i], green_m[i], green_r[i],
                        'green')
                    for i, idx in enumerate(idx['green'])},
              'blue':{idx: Body(idx, blue_p[i], blue_v[i], blue_m[i], blue_r[i],
                        'blue')
                    for i, idx in enumerate(idx['blue'])},
              'yellow':{idx: Body(idx, yellow_p[i], yellow_v[i], yellow_m[i], yellow_r[i],
                        'yellow')
                    for i, idx in enumerate(idx['yellow'])}}
    
    effectors = {'red': {}, 'green': {}, 'blue': {}, 'yellow': {}, 'drawer': {}}
    effectors['red'].update(bodies['red'])
    effectors['red'].update(bodies['yellow'])
    effectors['green'].update(bodies['red'])
    effectors['green'].update(bodies['green'])
    effectors['blue'].update(bodies['green'])
    effectors['blue'].update(bodies['blue'])
    effectors['yellow'].update(bodies['red'])
    effectors['yellow'].update(bodies['green'])
    effectors['yellow'].update(bodies['blue'])
    effectors['drawer'].update(bodies['red'])
    effectors['drawer'].update(bodies['green'])
    effectors['drawer'].update(bodies['blue'])
    effectors['drawer'].update(bodies['yellow'])
                               
    def build_systemone(color, i):
        bsys = {}
        bsys.update(effectors[color])
        if i not in bsys:
            bsys[i] = bodies[color][i]
        return SystemOne(i, bsys)
            
    system_ones = {c: {i: build_systemone(c, i) for i in idx[c]}
               for c in colors
    }

    all_bodies = {}
    for b in bodies.values():
        all_bodies.update(b)    
    system_drawer = SystemDrawer(System(all_bodies), (10, 10), 1, auto)

    dt_colors = {'red':Time(0,int(1e8)), 'green':Time(0,int(1e8)),
                 'blue':Time(0,int(1e8)), 'yellow':Time(0,int(1e8))}
    dt_draw = Time(1, int(1e8))
    
    def build_systemone_node(s, time, dt, max_qsize):
        name = s.bodies[s.theone].name
        node = SystemOneNode(s, Node(name, time, dt, max_qsize))
        return node

    time = Time(0)
    max_qsize = 10
    
    system_one_nodes = {s.theone: build_systemone_node(s, time,
                                                       dt_colors[c], max_qsize)
                            for c in colors for s in system_ones[c].values()}

    system_drawer_node = DrawerNode(system_drawer,
                                    Node('drawer', time, dt_draw, max_qsize))
    
    def get_node(name):
        if name == 'drawer':
            node = system_drawer_node
        else:
            node = system_one_nodes[name]
        return node
    
    #setup flows
    all_nodes = list(system_one_nodes.values())+[system_drawer_node]
    for node in all_nodes:
        #setup outflows
        if node.name != 'drawer':
            node_color = all_bodies[node.name].color
            sinks=[Sink(sb.name,
                        callback=get_node(sb.name).push_callback(node.name))
                   for sb in all_bodies.values()
                   if node.name in effectors[sb.color] and sb.name != node.name]
            sinks.append(Sink('drawer', callback=get_node('drawer').push_callback(node.name)))
            node.setup_outflow(OutFlow(node.name, time,
                                       dt_colors[node_color],
                                       0, # qsize
                                       sinks,
                                       FlowType.PRED))
        else:
            node_color = 'drawer'

        #setup inflows
        for ni, b in effectors[node_color].items():
            if ni != node.name:
                node.setup_inflow(
                    InFlow(ni, time, dt_colors[b.color],
                           qsize = 0,
                           callback=node.process,
                           time_callback=get_node(ni).time_callback(ni,
                                                                    node.name)))


    signal.signal(signal.SIGINT, handler_sigint)
    
    print("[master] Starting threads.")
    #start threads
    for node in system_one_nodes.values():
        node.start()
    system_drawer_node.start()

    #wait on stop signal or simulate at least 10 seconds if auto is set
    if not auto:
        stop_event.wait()
    else:
        while system_drawer_node.time < stop_time\
              and not stop_event.is_set():
            sleep(0.1)
            print("[master] {} < {} = {}".format(system_drawer_node.time,Time(10),
                                                 system_drawer_node.time < Time(10)))
    print("[master] Stopping threads.")
    
    #stop all threads
    for node in system_one_nodes.values():
        print("[master] Stopping node {}".format(node.name))
        node.stop()
    print("[master] Stopping node {}".format(system_drawer_node.name))
    system_drawer_node.stop()

    print("[master] Joining threads...")
    #join all threads
    remaining = list(system_one_nodes.values())
    stime = systime()
    while len(remaining) > 0 and not systime() - stime > 1:
        node = remaining.pop(0)
        node.join(0.001)
        if node.isAlive():
            remaining.append(node)
    system_drawer_node.join(0.1)

    if(except_event.is_set()):
        print("[master] Failure: {} exception(s) collected".format(excepts.qsize()))
    else:
        print("[master] Done")

    assert(not except_event.is_set())
              
if __name__ == "__main__":
    test_nbody(auto=False)
