/***
    node.hpp

***/
#ifndef DSAAM_NODE_HPP
#define DSAAM_NODE_HPP

#include<dsaam/common.hpp>
#include<dsaam/inflows.hpp>
#include<dsaam/outflows.hpp>
#include<deque>

namespace dsaam
{
  class Node
  {
  public:
    typedef std::shared_ptr<class MessageBase const > mpointer;
    typedef std::function<void(const mpointer &)> push_callback_type;

    Node(string &name, Time &time, Time &dt, std::vector<InFlow> &inflows,
	 std::vector<OutFlow> &outflows, unsigned int max_qsize) :
      name(name), _time(time), _dt(dt),
      inflows(inflows, time, max_qsize), outflows()
    {
      for(OutFlow flow : outflows)
	{
	  this->outflows.emplace_back(flow.name, _time, flow.dt,
				      flow.sinks, max_qsize);
	}
    }

    ~Node(){}

    void next()
    {
      this->inflows.next();
    }

    ::dsaam::send_callback_type send_callback(unsigned int flow)
    {
      using std::placeholders::_1;
      return std::bind(&Node::_send, this, &outflows[flow], _1);
    }
    
    ::dsaam::send_callback_type send_callback(const string & flow)
    {
      return send_callback(_out_flow_index(flow));
    }

    ::dsaam::time_callback_type time_callback(unsigned int flow, unsigned int sink)
    {
      using std::placeholders::_1;
      return std::bind(&OutMessageFlow<mpointer>::time_callback, &outflows[flow], sink, _1);
    }

    ::dsaam::time_callback_type time_callback(const string &flow, const string &sink)
    {
      unsigned int iflow = _out_flow_index(flow);
      return time_callback(iflow, _out_flow_sink_index(iflow, sink));
    }
    push_callback_type push_callback(unsigned int flow)
    {
      using std::placeholders::_1;
      return std::bind(&MessageFlowMultiplexer<mpointer>::push, &inflows, flow, _1);
    }

    push_callback_type push_callback(const string &flow)
    {
      return push_callback(_in_flow_index(flow));
    }

    void stepTime(const Time & t)
    {
      logic_assert(t >= _time,
		   to_string("Time contract breached, stepping back in time from ",_time," to ",t));
      _time = t;
    }

    const Time & time() const
    {
      return _time;
    }

    const Time & nextAt() const
    {
      return inflows.nextTime();
    }

    const Time & dt() const
    {
      return _dt;
    }

    void set_inflow_callbacks(const string & ifname,
			      const ::dsaam::message_callback_type &m_cb,
			      const ::dsaam::time_callback_type &t_cb)
    {
      inflows.set_flow_callbacks(_in_flow_index(ifname), m_cb, t_cb);
    }

    void set_outflow_callback(const string & ofname,
			      const string & sinkname,
			      const ::dsaam::send_callback_type &cb)
    {
      unsigned int odx = _out_flow_index(ofname);
      outflows[odx].set_sink_callback(_out_flow_sink_index(odx, sinkname), cb);
    }

  private:

    void _send(OutMessageFlow<mpointer> * outflow, const mpointer & m)
    {
      logic_assert(m->time >= time(),
		   to_string("Time contract breached : sending message with time ", m->time,
			     " in the future  (current time is ", time()));
      logic_assert(m->time <= inflows.nextTime(),
		   to_string("Time contract breached: sending message at ",
			     m->time, " before arrival of next message at ", inflows.nextTime()));
      outflow->send(m);
    }
      
    unsigned int _out_flow_index(const string & name) const
    {
      for(unsigned int i=0; i<outflows.size(); i++)
	{
	  if(outflows[i].name == name) return i;
	}
      throw  std::domain_error("No outflow "+name+" defined on this node");
    }

    unsigned int _out_flow_sink_index(unsigned int flow, const string & name) const
    {
      return outflows[flow].subscriber_index(name);
    }

    unsigned int _in_flow_index(const string & name)
    {
      return inflows.flow_index(name);
    }
    
  public:
    const string name;
  private:
    Time _time;
    Time _dt;
    MessageFlowMultiplexer<mpointer> inflows;
    std::deque<OutMessageFlow<mpointer>> outflows;  
  };


}

#endif //DSAAM_NODE_HPP
