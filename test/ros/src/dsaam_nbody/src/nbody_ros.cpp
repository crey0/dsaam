#include<dsaam/ros/ros_node.hpp>
#include<dsaam/time.hpp>
#include<dsaam/onethreadnode.hpp>
#include<dsaam_nbody/nbody_common.hpp>
#include<boost/bind.hpp>
#include<geometry_msgs/QuaternionStamped.h>
#include<geometry_msgs/PointStamped.h>
#include <XmlRpcValue.h>

using pm_type = geometry_msgs::PointStamped;
using vm_type = geometry_msgs::QuaternionStamped;

void p_callback(Body &b, const boost::shared_ptr<const pm_type> & pm, const ros::Time&)
{
  b.p[0] = pm->point.x;
  b.p[1] = pm->point.y;
}

void v_callback(Body &b, const boost::shared_ptr<const vm_type> & pm, const ros::Time&)
{
  b.v[0] = pm->quaternion.z;
  b.v[1] = pm->quaternion.w;
}



using TTransport = dsaam::ros::RosTransport;
class OneBSystemNode : public OneBSystemNodeBase<TTransport>
{
public:
  using TTransport::time_type;
  OneBSystemNode(const std::vector<Body> & bodies, const string &name,
		 const time_type &time, const time_type &dt, unsigned int max_qsize,
		 const time_type &stop_time)
    : OneBSystemNodeBase<TTransport>(bodies, name, time, dt, max_qsize, stop_time)
  {
  }

  auto pCallback(const string &b_name) -> \
    boost::function<void(const boost::shared_ptr<const pm_type> &, const ros::Time&)>
  {
    Body & b = system.get_body(b_name);
    return std::bind(&p_callback, std::ref(b),
		     std::placeholders::_1, std::placeholders::_2);
  }

  auto vCallback(const string &b_name)  -> \
    boost::function<void(const boost::shared_ptr<const vm_type> &, const ros::Time&)>
  {
    Body & b = system.get_body(b_name);
    return std::bind(&v_callback,  std::ref(b),
		     std::placeholders::_1, std::placeholders::_2);
  }

protected:
  void send_state(const time_type & t) override
  {
    mpointer m {new dsaam::ros::ROSMessagePointerHolder{new vm_type()}};
    pm_type &p = const_cast<pm_type&>(m->get_ref<pm_type>());
    p.header.stamp = t;
    p.point.x = system.self().p[0];
    p.point.y = system.self().p[1];
    send_p(std::move(m));
     
    
    m = mpointer(new dsaam::ros::ROSMessagePointerHolder{new vm_type()});
    vm_type &v = const_cast<vm_type&>(m->get_ref<vm_type>());
    v.header.stamp = t;
    v.quaternion.z = system.self().p[0];
    v.quaternion.w = system.self().p[1];
    send_v(std::move(m));
  }
private:
};

using time_type = OneBSystemNode::time_type;

time_type from_string(const string &nanos_s)
{
  unsigned long nanos = std::stoul(nanos_s);
  return time_type((int)(nanos / dsaam::Time::NS_IN_SECOND),
		   (int)(nanos % dsaam::Time::NS_IN_SECOND));
}

int main()
{
  std::vector<Body> bodies;

  string name = ros::this_node::getName();

  string param;
  assert(ros::param::get("start_time", param));
  time_type t = from_string(param);
  assert(ros::param::get("dt", param));
  time_type dt = from_string(param);
  assert(ros::param::get("stop_time", param));
  time_type stop_time = from_string(param);

  assert(ros::param::get("max_qsize",param));
  size_t max_qsize = std::stoul(param);

  XmlRpc::XmlRpcValue ptree;
  assert(ros::param::get("inflows", ptree));
  for(auto &_pt : ptree)
    {
      auto pt = _pt.second;
      string b_name = pt["name"];
      XmlRpc::XmlRpcValue bodyp;
      assert(ros::param::get("/"+b_name+"/body", bodyp));
      Array2d p_0,v_0;
      p_0[0] = double(bodyp["position"][0]);
      p_0[1] = double(bodyp["position"][1]);
      p_0[0] = double(bodyp["speed"][0]);
      p_0[1] = double(bodyp["speed"][1]);

      bodies.emplace_back(b_name, p_0, v_0,
			  double(bodyp["mass"]), double(bodyp["radius"]),
			  string(bodyp["color"]));
    }
  auto node = OneBSystemNode(bodies, name, t, dt, max_qsize, stop_time);

  //init subs
  for(auto &_pt : ptree)
    {
      auto pt = _pt.second;
      string o_name = pt["name"];
      time_type o_dt = from_string(string(pt["dt"]));
      std::vector<string> sinks;
      for(auto &s : pt["sinks"])
	{
	  sinks.push_back(string(s.second));
	}
      node.setup_publisher<geometry_msgs::PointStamped>(o_name+"/p", t, o_dt, sinks);
      node.setup_publisher<geometry_msgs::PointStamped>(o_name+"/s", t, o_dt, sinks);
    }
  
  //init pubs
  assert(ros::param::get("outflows", ptree));
  for(auto &_pt : ptree)
    {
      auto pt = _pt.second;
      string f_name = string(pt["name"]);
    }
  

  //finish init ROS transport, wait for incoming subscriptions
  node.init_ros();

  return 0;
}
