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

from __future__ import absolute_import, print_function, division
from ..node import Node, InFlow, OutFlow, Sink, Time, FlowType
import rospy
from rospy.topics import SubscribeListener
from std_msgs.msg import Header
import importlib
import string
from threading import Semaphore
from copy import copy

class CountSubListener(SubscribeListener):
    def __init__(self, peers = None, num_peers = 0):
        self.peers = copy(peers)
        if self.peers is None:
            self.num_peers = num_peers
        else:
            self.num_peers = len(peers)
        self.semaphore = Semaphore(0)
     	
    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        #hack to get the subscriber node id, in order to perform checks
        peer_id = peer_publish.__closure__[0].cell_contents._connection.endpoint_id[1:]
        if self.peers is not None and not peer_id in self.peers: # peer not awaited
            print("Peer /{} subscribed to {} (unawaited)".format(peer_id, topic_name))
            return
        
        print("Peer /{} subscribed to {} (n={})".format(peer_id, topic_name, self.num_peers))
        self.semaphore.release()

    def peer_unsubscribe(self, topic_name, num_peers):
        raise RuntimeError("No unsubscription allowed for topic {}".format(topic_name))

    def wait(self):
        for _ in range(self.num_peers):
            self.semaphore.acquire()

class RosNode(Node):

    def __init__(self, name, start_time, dt, default_qsize, global_name=None):
    
        # ROS pubs and subs dicts
        self.publishers = {}
        self.subscribers = {}
        self.sublisteners = []
        self.global_name = [global_name, name][global_name is None]
        
        # init parent
        super().__init__(name, start_time, dt, default_qsize)        
        

    def setup_subscriber(self, node_name, flow_name, m_class, callback,
                         start_time, dt, queue_size=0):
        #if queue_size is zero use default queue size
        queue_size = [queue_size, self.default_qsize] [queue_size <= 0]

        #setup inflow on node
        inflow = InFlow(flow_name, start_time, dt, queue_size, callback,
            time_callback=self.ros_in_time_callback(node_name, flow_name, queue_size))
        self.setup_inflow(inflow)

        #setup corresponding ROS subscriber
        self.subscribers[flow_name] = \
            rospy.Subscriber('/'+flow_name, m_class, callback=self.ros_push_callback(flow_name))
        
        
    def setup_publisher(self, flow_name, m_class,
                        start_time, dt, queue_size = 0, sinks = None, ftype=FlowType.PRED):
        #if queue_size is zero use default queue size
        queue_size = [queue_size, self.default_qsize] [queue_size <= 0]
        
        #If sinks is None set to empty list
        sinks = [sinks, []][sinks is None]
        
        #setup corresponding ROS publisher
        subl = CountSubListener(sinks)
        self.sublisteners.append(subl)
        self.publishers[flow_name] = \
            rospy.Publisher('/'+flow_name, m_class, subscriber_listener=subl,
                            queue_size=queue_size)
        
        #subscribe to callbacks on message processed by subscribers
        for s in sinks:
            subname = "/" + flow_name + "/time/" + s
            self.subscribers[subname] =\
                rospy.Subscriber(subname, Header,
                                 callback=self.ros_out_time_callback(flow_name, s))

        #create sinks
        sinks = [Sink(s, callback=None) for s in sinks]
        if len(sinks) > 0:
            sinks[0].callback = self.ros_send_callback(flow_name)

        #setup flow on node
        outflow = OutFlow(flow_name, start_time, dt, queue_size, sinks, ftype)
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
            h.frame_id = m.header.frame_id
            m.header = h
            pub.publish(m)
        return __cb

    def ros_in_time_callback(self, node, flow, max_qsize):
        pname = "/" + flow + "/time/" + self.global_name
        if pname not in self.publishers:
            #create subscriber listener for message processed subscribtion from publisher
            sub_listen = CountSubListener(peers=[node])
            self.sublisteners.append(sub_listen)
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
    node = RosNode(name.split("/")[-1], start_time, dt, max_qsize, global_name=name)
    return node
    
def setup_rosnode_from_params(node, get_callback):
    # setup publishers
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
        node.setup_subscriber(i['from'], i['name'], m_class, callback,
                              start_time, Time(nanos=i['dt']),
                              queue_size=0)

    node.ros_init()

