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

from dsaam_test.test_nbody import stop_event, except_event, exception_collect, excepts,\
    Body, System, SystemOne, SystemDrawer, SystemOneNode, DrawerNode
import rospy
from dsaam.ros.ros_node import Time, make_rosnode_from_params, setup_rosnode_from_params
from geometry_msgs.msg import QuaternionStamped, PointStamped
import numpy as np
from time import sleep

autotest = False
stop_time = None

def rosbridge(node):
    process = node.process
    send = node.send

    @exception_collect
    def _process(name, m, tmin_next):
        s, time, dt = m
        if name.endswith("position"):
            state = (np.array([s.point.x, s.point.y]), None)
        elif name.endswith("speed"):
            state = (None, np.array([s.quaternion.z, s.quaternion.w]))
        else:
            raise RuntimeError("Unknown message "+name)
        return process(name.split("/")[0], (state, time, dt), tmin_next)

    @exception_collect
    def _send(name, state, time):
        p = PointStamped()
        p.point.x = state[0][0]
        p.point.y = state[0][1]
        send(name + "/position", p, time)
        s = QuaternionStamped()
        s.quaternion.z = state[1][0]
        s.quaternion.w = state[1][1]
        send(name + "/speed", s, time)
        
        
    node.process = _process
    node.send = _send
    return node
        
def shutdown_hook():
    stop_event.set()

def make_ros_nbody_node():
    global autotest, stop_time
    
    name_global = rospy.get_param("name")
    name = name_global.split("/")[-1]
    print('[{}] Building node'.format(name))
    autotest = rospy.get_param('/autotest')
    if autotest:
        stop_time = Time(sec=rospy.get_param('/stop_time'))
        
    ifs = rospy.get_param('inflows')
    bodies = {}
    for i in ifs:
        i_name = i['from'].split("/")[-1]
        i_p = rospy.get_param('/' + i_name + '/body')
        bodies[i_name] = Body(i_name,
                              i_p['position'], i_p['speed'], i_p['mass'], i_p['radius'],
                              i_p['color'])

    ros_node = make_rosnode_from_params(name_global)
    if name.startswith('drawer'):
        system = System(bodies)
        size = rospy.get_param('size')
        scale = rospy.get_param('scale')
        s_drawer = SystemDrawer(system, size, scale, headless=autotest)
        node = rosbridge(DrawerNode(s_drawer, ros_node))
    else:
        p = rospy.get_param('body')
        position, speed, mass, radius, color = p['position'], p['speed'], p['mass'], p['radius'],\
                                               p['color']
        bodies[name] = Body(name,
                            position, speed, mass, radius, color)
        s_one = SystemOne(name, bodies)
        node = rosbridge(SystemOneNode(s_one, ros_node))

    setup_rosnode_from_params(ros_node, lambda *args:  node.process)
    
    return node



if __name__== '__main__':
    rospy.on_shutdown(shutdown_hook)
    node = make_ros_nbody_node()
    print('[{}] Starting node'.format(node.name))
    node.start()
    if not autotest:
        stop_event.wait()
    else:
        while node.time < stop_time\
              and not stop_event.is_set():
            sleep(0.1)
    if not rospy.is_shutdown():
        rospy.signal_shutdown("Internal error during execution")
    node.join(0.1)
    print('[{}] Node stopped'.format(node.name))

    assert not except_event.is_set(), "Failure with {} exceptions".format(excepts.qsize())
