#include<dsaam/ros/ros_node.hpp>
#include<dsaam_nbody/nbody_common.hpp>

using RosNode = Node<RosTransport>;
using Time = RosNode::time_type;
void main()
{
  Time t, dt;
  RosNode("gloubi", t, dt);
}
