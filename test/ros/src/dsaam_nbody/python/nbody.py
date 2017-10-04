from dsaam_test.test_nbody import stop_event, except_event, exception_collect, excepts,\
    Body, System, SystemOne, SystemDrawer, SystemOneNode, DrawerNode
import rospy
from dsaam.ros.ros_node import RosNode, Time
from geometry_msgs.msg import QuaternionStamped, PointStamped
import numpy as np
from time import sleep

autotest = False

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
    global autotest
    
    name = rospy.get_param("name")
    print('[{}] Building node'.format(name))
    autotest = rospy.get_param('/autotest')

    ifs = rospy.get_param('inflows')
    bodies = {}
    for i in ifs:
        i_name = i['from']
        i_p = rospy.get_param('/' + i_name + '/body')
        bodies[i_name] = Body(i_name,
                              i_p['position'], i_p['speed'], i_p['mass'], i_p['radius'],
                              i_p['color'])

    ros_node = RosNode(name)
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

    return node



if __name__== '__main__':
    rospy.on_shutdown(shutdown_hook)
    node = make_ros_nbody_node()
    print('[{}] Starting node'.format(node.name))
    node.start()
    if not autotest:
        stop_event.wait()
    else:
        while node.time < Time(10)\
              and not stop_event.is_set():
            sleep(0.1)
    if not rospy.is_shutdown():
        rospy.signal_shutdown("Internal error during execution")
    node.join(0.1)
    assert not except_event.is_set(), "Failure with {} exceptions".format(excepts.qsize())
