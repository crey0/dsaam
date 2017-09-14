#include<dsaam/ros/ros_node.hpp>
#include<dsaam/onethreadnode.hpp>
#include<dsaam_nbody/nbody_common.hpp>

using TTransport = dsaam::ros::RosTransport;
class OneBSystemNode : public OneBSystemNodeBase<TTransport>
{
public:
  OneBSystemNode(std::vector<Body> & bodies, string &name, ros::Time &time, ros::Time &dt,
		 unsigned int max_qsize, const ros::Time &stop_time)
    : OneBSystemNodeBase<TTransport>(bodies, name, time, dt, max_qsize, stop_time)
  {
  }

  message_callback_type pCallback(const string &b_name)
  {
    Body & b = system.get_body(b_name);
    return std::bind(&p_callback, std::ref(b),
		     std::placeholders::_1, std::placeholders::_2);
  }

  message_callback_type vCallback(const string &b_name)
  {
    Body & b = system.get_body(b_name);
    return std::bind(&v_callback,  std::ref(b),
		     std::placeholders::_1, std::placeholders::_2);
  }

protected:
  void send_state(const dsaam::Time & t) override
  {
    mpointer m = \
      mpointer(new PMessage(system.self().p, t));
    send_p(std::move(m));
     
    
    m = mpointer(new VMessage(system.self().v, t));
    send_v(std::move(m));
  }
};

using Time = RosNode::time_type;
void main()
{
  Time t, dt;
  RosNode("gloubi", t, dt);
}
