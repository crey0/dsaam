from ..test_nbody import stop_event, except_event, exception_collect,\
    Body, System, SystemOne, SystemDrawer, SystemOneNode, DrawerNode
import rospy
from .ros_node import RosNode
from geometry_msgs.msg import PointStamped

def rosbridge(node):
    process = node.process
    send = node.send

    @exception_collect
    def _process(name, m, tmin_next):
        s, time, dt = m
        state = [s.point.x, s.point.y]
        process(name, (state, time, dt), tmin_next)

    @exception_collect
    def _send(name, state, time):
        s = PointStamped()
        s.point.x = state[0]
        s.point.y = state[1]
        send(name, s, time)
        
    node.process = _process
    node.send = _send
    return node
        
def shutdown_hook():
    stop_event.set()

def make_ros_nbody_node():
    name = rospy.get_name()
    print('[{}] Building node'.format(name))
    autotest = rospy.get_param('/autotest')

    ifs = rospy.get_param('inflows')
    bodies = {}
    for i in ifs:
        i_name = i['name']
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
        ros_node = RosNode(name)
        node = rosbridge(SystemOneNode(s_one, ros_node))

    return node



if __name__== '__main__':
    rospy.on_shutdown(shutdown_hook)
    node = make_ros_nbody_node()
    print('Starting node')
    node.start()
    stop_event.wait()
    if not rospy.is_shutdown():
        rospy.init_shutdown("Internal error during execution")
    node.join(0.1)
    assert not except_event.is_set(), "Failure with {} exceptions".format(excepts.qsize())
