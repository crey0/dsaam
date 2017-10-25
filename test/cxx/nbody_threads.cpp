/***

Nbody test

 ***/
#include <dsaam/onethreadnode.hpp>
#include <dsaam/string_utils.hpp>
#include <dsaam/time.hpp>

#include "nbody_common.hpp"

class MessageBase
{
public:
  MessageBase(const dsaam::Time &time) : time(time) {}
public:
  const dsaam::Time time;
};

using mpointer = dsaam::ThreadTypes<MessageBase, dsaam::Time>::message_cptr;

struct FMessageTime
{
  static const dsaam::Time& time(mpointer b)
  {
    return b->time;
  }
};

class PMessage : public MessageBase
{
public:
  PMessage(const Array2d & p, const dsaam::Time & t)
    : MessageBase(t), p(p) {}
public:
  Array2d p;
};

class VMessage : public MessageBase
{
public:
  VMessage(const Array2d & v, const dsaam::Time & t)
    : MessageBase(t), _filler_("filler"), v(v) {}
public:
  string _filler_;
  Array2d v;
};



void p_callback(Body &b, const mpointer & pm, const dsaam::Time &)
{
  if(!pm) return;
  auto pmp = static_cast<const PMessage *>(pm.get());
  b.p = pmp->p;
}
  
void v_callback(Body & b, const mpointer & pm,  const dsaam::Time &)
{
  if(!pm) return;
  auto pmv = static_cast<const VMessage *>(pm.get());
  b.v = pmv->v;
}


using TTransport = dsaam::ThreadTransport<MessageBase, dsaam::Time, FMessageTime>;
using InFlow = TTransport::InFlow;
using OutFlow = TTransport::OutFlow;
using Sink = TTransport::Sink;

class OneBSystemNode : public OneBSystemNodeBase<TTransport>
{
public:
  OneBSystemNode(std::vector<Body> & bodies, string &name, dsaam::Time &time, dsaam::Time &dt,
		 unsigned int max_qsize, const dsaam::Time &stop_time)
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
    //std::cout << dsaam::to_string("[",t,"] ",
    //				  "[",system.self().name ,"] ",
    //				  "Sending state\n");
    mpointer m = \
      mpointer(new PMessage(system.self().p, t));
    send_p(std::move(m));
     
    
    m = mpointer(new VMessage(system.self().v, t));
    send_v(std::move(m));
  }
};

int main()
{
  string colors[] = {"red", "green", "blue", "yellow"};

  unsigned int n = 4;
  string names[] = {"red_0", "green_0", "blue_0", "yellow_0"};
  
  std::vector<string> ins[] = {{"yellow_0"},
			       {"red_0"},
			       {"green_0"},
			       {"red_0", "green_0", "blue_0"}};
  
  
  Array2d positions[] = {{{0.5,  0.5}},
		       {{0.3,  0.3}},
		       {{0.2,  0.2}},
		       {{0.15, 0.15}}};
  
  Array2d speeds[] =  {{{0.0,   0.0}},
		     {{-0.1,  0.1}},
		     {{-0.1,  0.1}},
		     {{-0.1,  0.1}}};

  double masses[] = {1000., 100., 10., 1.};
  
  double radii[] = {100., 10., 1., 0.1};
  

  dsaam::Time start_time(0);
  dsaam::Time dts[] = {{1}, {2}, {3}, {4}};

  unsigned int max_qsize = 10;
  
  //construct all bodies
  std::vector<Body> all_bodies;
  for(size_t i=0; i<n; i++)
    {
      all_bodies.push_back(Body(names[i], positions[i],
				speeds[i], masses[i], radii[i], colors[i]));
    }

  //  auto get_node_idx = [&](string name) -> size_t
  // {for(size_t i=0; i<n; i++) if (names[i] == name) return i;
  // throw std::domain_error(name);}; 
  
  //construct all Nodes
  std::deque<OneBSystemNode> nodes;
  for(size_t i=0; i<n; i++)
    {      
      //std::vector<OutFlow> outflows = {{b.name + "/position", start_time, dts[i], sinks},
      //				       {b.name + "/speed", start_time, dts[i], sinks}};
      nodes.emplace_back(all_bodies,
			 names[i], start_time, dts[i],
			 max_qsize, dsaam::Time(10000));
      auto bp = names[i] + "/position";
      auto bs = names[i] + "/speed";
      nodes.back().setup_outflow({bp, start_time, dts[i]});
      nodes.back().setup_outflow({bs, start_time, dts[i]});
      
    }

  auto get_node = [&](string name) -> OneBSystemNode&
    {for(auto & n : nodes) if (n.name == name) return n;
     throw std::domain_error(name);}; 
	  

  //link Nodes together
   for(size_t i=0; i<n; i++)
    {
      auto &n = nodes[i];
      auto bp = n.name + "/position";
      auto bs = n.name + "/speed";
      for(size_t j=0; j < ins[i].size(); j++)
	{
	  string j_name = ins[i][j];
	  auto & nj = get_node(j_name);
	  auto jp = j_name + "/position";
	  auto js = j_name + "/speed";

	  n.setup_inflow({jp,  start_time, nj.dt, n.pCallback(j_name)});
	  nj.setup_sink(jp, {jp, n}),
	    
	  n.setup_inflow({js, start_time, nj.dt, n.vCallback(j_name)});
	  nj.setup_sink(js, {js, n});
	  
	  //n.set_inflow_callbacks(jp, n.pCallback(j_name), nj.time_callback(jp, n.name));
	  //n.set_inflow_callbacks(js, n.vCallback(j_name), nj.time_callback(js, n.name));

	}
      
    }

  //start Nodes
  for(auto &n : nodes) n.start();

  //wait for Nodes to stop
  for(auto &n : nodes) n.join();

  //stop
  return 0;
}
