/*
Copyright © 2018 CNRS
All rights reserved.

@author Christophe Reymann

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
#include<dsaam/ros/ros_node.hpp>
#include <map>
#include <boost/assign/list_inserter.hpp>

using namespace boost::assign;

double to_double(const ros::Duration &t)
{
  return t.toSec();
}

#include<dsaam/time.hpp>
#include<dsaam/onethreadnode.hpp>
#include<dsaam_nbody/nbody_common.hpp>
#include<boost/bind.hpp>
#include<geometry_msgs/QuaternionStamped.h>
#include<geometry_msgs/PointStamped.h>
#include <XmlRpcValue.h>


#ifdef logic_assert
#undef logic_assert
#endif

#define logic_assert(test) if(!(test)) throw std::logic_error("Assertion failed: " #test)

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
		 const time_type &time, const time_type &dt, size_t max_qsize,
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
  virtual void send_state(const time_type & t) override
  {
    //std::cout << "Sending state at "<< t << std::endl;
    mpointer m {new dsaam::ros::ROSMessagePointerHolder{new vm_type()}};
    pm_type &p = const_cast<pm_type&>(m->get_ref<pm_type>());
    p.header.stamp = t;
    p.point.x = system.self().p[0];
    p.point.y = system.self().p[1];
    send_p(std::move(m));
     
    
    m = mpointer(new dsaam::ros::ROSMessagePointerHolder{new vm_type()});
    vm_type &v = const_cast<vm_type&>(m->get_ref<vm_type>());
    v.header.stamp = t;
    v.quaternion.z = system.self().v[0];
    v.quaternion.w = system.self().v[1];
    send_v(std::move(m));
  }
private:
};

using time_type = OneBSystemNode::time_type;

time_type from_nanos(const unsigned long &nanos)
{
  return time_type((int)(nanos / dsaam::Time::NS_IN_SECOND),
		   (int)(nanos % dsaam::Time::NS_IN_SECOND));
}
time_type from_secs(const int &secs)
{
  return time_type((int)secs,
		   (int)0);
}
time_type from_string(const string &nanos_s)
{
  unsigned long nanos = std::stoul(nanos_s);
  return from_nanos(nanos);
}

int main(int argc, char **argv)  
{
  
  ros::init(argc, argv, "__remapped__");
  
  std::vector<Body> bodies;

  string name_global;
  logic_assert(ros::param::get("name", name_global));
  auto pos = name_global.rfind("/");
  string name = name_global.substr(pos+1);
  std::cout << "[" << name << "] Building node" << std::endl;


  int param_int = -1;
  logic_assert(ros::param::get("start_time", param_int));
  time_type t = from_nanos(param_int);
  logic_assert(ros::param::get("dt", param_int));
  time_type dt = from_nanos(param_int);
  logic_assert(ros::param::get("/stop_time", param_int));
  time_type stop_time = from_secs(param_int);
  logic_assert(ros::param::get("max_qsize",param_int));
  size_t max_qsize = param_int;
  logic_assert(max_qsize > 0 );
  
  XmlRpc::XmlRpcValue ptree;

  auto has_body = [&bodies](const string &b_name)
    {
      for(auto &b : bodies)
	{
	  if (b.name == b_name) return true;
	}
      return false;
    };
  
  auto add_body =  [&bodies,&has_body](const string& b_name, XmlRpc::XmlRpcValue &bodyp) -> void
    {
      if (has_body(b_name)) return;
      
      Array2d p_0,v_0;
      p_0[0] = double(bodyp["position"][0]);
      p_0[1] = double(bodyp["position"][1]);
      v_0[0] = double(bodyp["speed"][0]);
      v_0[1] = double(bodyp["speed"][1]);

      bodies.emplace_back(b_name, p_0, v_0,
			  double(bodyp["mass"]), double(bodyp["radius"]),
			  string(bodyp["color"]));
    };
  
  logic_assert(ros::param::get("body", ptree));
  add_body("/"+name, ptree);
	   
  logic_assert(ros::param::get("inflows", ptree));
  for(int i=0; i < ptree.size(); i++)
    {
      auto pt = ptree[i];
      string b_name = string(pt["from"]);
      b_name = b_name.substr(b_name.rfind("/"));
      XmlRpc::XmlRpcValue bodyp;
      logic_assert(ros::param::get("/"+b_name+"/body", bodyp));
      add_body(b_name, bodyp);
     
    }
  OneBSystemNode node{bodies, "/"+name, t, dt, max_qsize, stop_time};

  auto get_body = [&bodies](const string &n) -> const string&
    {
      for(auto &b : bodies)
	if(n.find(b.name) !=  std::string::npos) return b.name;
      throw std::domain_error("Undefined body " + n);
    };
  
  //init subs
  for(int i=0; i < ptree.size(); i++)
    {
      auto pt = ptree[i];
      string i_name = "/" + string(pt["name"]);
      string n_name = string(pt["from"]);
      time_type i_dt = from_nanos(int(pt["dt"]));
      const string &in_body_name = get_body(i_name);
      
      string type = pt["message_class"];
      if(type == "geometry_msgs.msg.PointStamped")
	{
	  node.setup_subscriber<geometry_msgs::PointStamped>(n_name, i_name, name_global,
							     t, i_dt,
				node.pCallback(in_body_name));
	}
      else if(type == "geometry_msgs.msg.QuaternionStamped")
	{
	  node.setup_subscriber<geometry_msgs::QuaternionStamped>(n_name, i_name, name_global,
								  t, i_dt,
				node.vCallback(in_body_name));
	} 
      else
	{
	  throw std::domain_error("Unknown message type" + type);
	}
    }
  
  //init pubs
  logic_assert(ros::param::get("outflows", ptree));
  for(int i=0; i < ptree.size(); i++)
    {
      auto pt = ptree[i];
      string o_name = "/" + string(pt["name"]);
      time_type o_dt = from_nanos(int(pt["dt"]));
      std::vector<string> sinks;
      auto pt_sinks = pt["sinks"];
      for(int s=0; s < pt_sinks.size(); s++)
	{
	  sinks.push_back(string(pt_sinks[s]));
	}
      string type = pt["message_class"];
      if(type == "geometry_msgs.msg.PointStamped")
	{
	  node.setup_publisher<geometry_msgs::PointStamped>(o_name, t, o_dt, sinks);
	}
      else if(type == "geometry_msgs.msg.QuaternionStamped")
	{
	  node.setup_publisher<geometry_msgs::QuaternionStamped>(o_name, t, o_dt, sinks);
	}
      
      else
	{
	  throw std::domain_error("Unknown message type" + type);
	}
    }
  

  
  //finish init ROS transport, wait for incoming subscriptions
  node.init_ros();
  //start node
  std::cout << "[" << node.name << "] Starting node" << std::endl;
  node.start();
  if( stop_time > time_type() )
    {
	  node.join();
	  ros::shutdown();
    }
  ros::waitForShutdown();
  std::cout << "[" << node.name << "] Node stopped" << std::endl;

  return 0;
}
