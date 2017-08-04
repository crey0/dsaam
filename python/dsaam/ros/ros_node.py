from __future__ import absolute_import, print_function, division
from ..node import Node, InFlow, OutFlow, Sink, Time
import rospy
from rospy.topics import SubscribeListener
from std_msgs.msg import Header
import importlib
import string
from threading import Semaphore

def get_class(name_string):
    m_name, c_name = name_string.rsplit('.', maxsplit=1)
    module = importlib.import_module(m_name)
    m_class = getattr(module, c_name)
    return m_class

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

    def __init__(self, name):
        # get global node parameters
        max_qsize = rospy.get_param('max_qsize')
        time = Time(nanos=rospy.get_param('start_time'))
        dt = Time(nanos=rospy.get_param('dt'))

        # TODO: setup management service

        # ROS pubs and subs dicts
        self.publishers = {}
        self.subscribers = {}
        sublisteners = []
        
        # setup outflows
        p_out = rospy.get_param("outflows")      
        ofs = []
        for o in p_out:
            m_class = get_class(o['message_class'])
            subl = CountSubListener(num_peers=len(o['sinks']))
            sublisteners.append(subl)
            self.publishers[o['name']] = \
                rospy.Publisher('/'+o['name'], m_class, subscriber_listener=subl,
                                queue_size=max_qsize)
            sink_names = o['sinks']
            sinks = []
            for s in sink_names:
                sinks.append(Sink(name=s, callback=None))
                subname = "/" + name + "/time/" + s
                self.subscribers[subname] =\
                    rospy.Subscriber(subname, Header, callback=self.ros_out_time_callback(name, s))
            sinks[0].callback = self.ros_send_callback(o['name'])
            ofs.append(OutFlow(name=o['name'], dt=Time(nanos=o['dt']), sinks=sinks))

        # setup inflows
        p_in  = rospy.get_param("inflows")        
        ifs = []
        subl = CountSubListener(num_peers=len(p_in))
        sublisteners.append(subl)
        for i in p_in:
            m_class = get_class(i['message_class'])
            self.subscribers[i['name']] = \
                rospy.Subscriber('/'+i['name'], m_class, callback=self.ros_push_callback(i['name']))
            
            ifs.append(InFlow(name=i['name'], dt=Time(nanos=i['dt']),
                time_callback=self.ros_in_time_callback(i['name'], name, max_qsize, subl)))

        # init parent
        super().__init__(name, time, dt, ifs, ofs, max_qsize)

        #init node
        rospy.init_node(name)
        
        # wait for all sinks to be subscribed
        for s in sublisteners:
            s.wait()

        print("[{}] Init DONE".format(name))
        
    def ros_send_callback(self, flow):
        pub = self.publishers[flow]
        def __cb(m, time):
            print("[{}] OUT ROS message on flow {}".format(self.name, flow))
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
            h = Header()
            h.stamp = rospy.Time(time.sec, time.nanos)
            tpub.publish(h)
        return __cb

    def ros_out_time_callback(self, flow, sink):
        cb = self.time_callback(flow, sink)
        def __cb(h):
            time = Time(h.stamp.secs, h.stamp.nanos)
            cb(time)
        return __cb
            
    def ros_push_callback(self, flow):
        cb = self.push_callback(flow)
        def __cb(m):
            print("[{}] IN ROS message on flow {}".format(self.name, flow))
            stamp = m.header.stamp
            dt = m.header.dt 
            cb(m,
               Time(stamp.secs, stamp.nsecs))
               #Time(dt.secs, dt.nsecs))
        return __cb
