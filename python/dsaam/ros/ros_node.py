from __future__ import absolute_import, print_function, division
from ..node import Node, InFlow, OutFlow, Sink, Time
import rospy
from rospy.topics import SubscribeListener
from std_msgs.msg import Header
import importlib
import string
from threading import Semaphore

class CountSubListener(SubscribeListener):
    def __init__(self, num_peers):
        self.semaphore = Semaphore(0)
        self.num_peers = num_peers
     	
    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        print("Peer subscribed to {} (n={})".format(topic_name, self.num_peers))
        self.semaphore.release()

    def peer_unsubscribe(self, topic_name, num_peers):
        raise RuntimeError("No unsubscription allowed for topic {}".format(topic_name))

    def wait(self):
        for _ in range(self.num_peers):
            self.semaphore.acquire()

class RosNode(Node):

    def __init__(self, name, start_time, dt, default_qsize):
    
        # ROS pubs and subs dicts
        self.publishers = {}
        self.subscribers = {}
        self.sublisteners = []

        # init parent
        super().__init__(name, start_time, dt, default_qsize)        


    def setup_subscriber(self, name, m_class, callback, start_time, dt, queue_size=0):
        #if queue_size is zero use default queue size
        queue_size = [queue_size, self.default_qsize] [queue_size <= 0]
        
        #create subscriber listener for message processed subscribtion from publisher
        subl = CountSubListener(num_peers=1)
        self.sublisteners.append(subl)

        #setup inflow on node
        inflow = InFlow(name, start_time, dt, queue_size, callback,
            time_callback=self.ros_in_time_callback(name, self.name,
                                                    queue_size, subl))
        self.setup_inflow(inflow)

        #setup corresponding ROS subscriber
        self.subscribers[name] = \
            rospy.Subscriber('/'+name, m_class, callback=self.ros_push_callback(name))
        
        
    def setup_publisher(self, name, m_class, start_time, dt, queue_size = 0, sinks = None):
        #if queue_size is zero use default queue size
        queue_size = [queue_size, self.default_qsize] [queue_size <= 0]
        
        #If sinks is None set to empty list
        sinks = [sinks, []][sinks is None]
        
        #setup corresponding ROS publisher
        subl = CountSubListener(num_peers=len(sinks))
        self.sublisteners.append(subl)
        self.publishers[name] = \
            rospy.Publisher('/'+name, m_class, subscriber_listener=subl,
                            queue_size=queue_size)
        
        #subscribe to callbacks on message processed by subscribers
        for s in sinks:
            subname = "/" + name + "/time/" + s
            self.subscribers[subname] =\
                rospy.Subscriber(subname, Header, callback=self.ros_out_time_callback(name, s))

        #create sinks
        sinks = [Sink(s) for s in sinks]
        if len(sinks) > 0:
            sinks[0].callback = self.ros_send_callback(name)

        #setup flow on node
        outflow = OutFlow(name, start_time, dt, queue_size, sinks)
        self.setup_outflow(outflow)


    def ros_init(self, init_node=True):
        #init node
        if(init_node):
            rospy.init_node(self.name)
        
        # wait for all sinks to be subscribed
        for s in self.sublisteners:
            s.wait()

        #print("[{}] Init DONE".format(self.name))
            
    def ros_send_callback(self, flow):
        pub = self.publishers[flow]
        def __cb(m, time):
            #print("[{}] OUT ROS message on flow {} / {}".format(self.name, flow, time))
            h = Header()
            h.stamp = rospy.Time(time.sec, time.nanos)
            m.header = h
            pub.publish(m)
        return __cb

    def ros_in_time_callback(self, flow, name, max_qsize, sub_listen):
        pname = "/" + flow + "/time/" + name
        if pname not in self.publishers:
            self.publishers[pname] = rospy.Publisher(pname, Header, subscriber_listener=sub_listen,
                                                     queue_size=max_qsize)
        tpub = self.publishers[pname]
        def __cb(time):
            #print("[{}] IN TIME for {} / {}".format(name, flow, time))
            h = Header()
            h.stamp = rospy.Time(time.sec, time.nanos)
            tpub.publish(h)
        return __cb

    def ros_out_time_callback(self, flow, sink):
        cb = self.time_callback(flow, sink)
        def __cb(h):
            time = Time(h.stamp.secs, h.stamp.nsecs)
            #print("[{}] OUT TIME from {} for {} / {}".format(self.name, sink, flow, time))
            cb(time)
        return __cb
            
    def ros_push_callback(self, flow):
        cb = self.push_callback(flow)
        def __cb(m):
            #print("[{}] IN ROS message on flow {}".format(self.name, flow))
            stamp = m.header.stamp
            #dt = m.header.dt
            cb(m,
               Time(stamp.secs, stamp.nsecs))
               #Time(dt.secs, dt.nsecs))
        return __cb


def get_class(name_string):
    m_name, c_name = name_string.rsplit('.', maxsplit=1)
    module = importlib.import_module(m_name)
    m_class = getattr(module, c_name)
    return m_class
    
def make_rosnode_from_params(name):
    # get global node parameters
    max_qsize = rospy.get_param('max_qsize')
    start_time = Time(nanos=rospy.get_param('start_time'))
    dt = Time(nanos=rospy.get_param('dt'))
    node = RosNode(name, start_time, dt, max_qsize)
    return node
    
def setup_rosnode_from_params(node, get_callback):
    # setup print()ublishers
    start_time = node.time
    
    p_out = rospy.get_param("outflows")      
    for o in p_out:
        m_class = get_class(o['message_class'])
        sinks = o['sinks']
        node.setup_publisher(o['name'], m_class,
                             start_time, Time(nanos=o['dt']),
                             0, # qsize
                             sinks)

    # setup subscribers
    p_in  = rospy.get_param("inflows")        
    for i in p_in:
        m_class = get_class(i['message_class'])
        callback = get_callback(i['name'], i['message_class'])
        node.setup_subscriber(i['name'], m_class, callback,
                              start_time, Time(nanos=i['dt']),
                              qsize=0)

    node.ros_init()

