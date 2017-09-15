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

  template<class T>
  class Node : public T
  {
  public:
    using transport_type = T;
    using typename transport_type::message_type;
    using typename transport_type::time_type;
    using typename transport_type::message_cptr;
    template<class F>
    using function_type = typename transport_type::template function_type<F>;

    using message_callback_type = ::dsaam::message_callback_type<message_cptr, time_type, function_type>;
    using send_callback_type = ::dsaam::send_callback_type<message_cptr, function_type>;
    using InFlow = typename transport_type::InFlow;
    using OutFlow = typename transport_type::OutFlow;
    using Sink = typename transport_type::Sink;

    Node(const string &name, const time_type &time, size_t default_qsize = 0) :
      name(name), _time(time), default_qsize(default_qsize),
      inflows(time, default_qsize), outflows()
    {}
    
    void next()
    {
      this->inflows.next();
    }

    send_callback_type send_callback(unsigned int flow)
    {
      using std::placeholders::_1;
      return std::bind(&Node::_send, this, &outflows[flow], _1);
    }
    
    send_callback_type send_callback(const string & flow)
    {
      return send_callback(_out_flow_index(flow));
    }

    void stepTime(const time_type & t)
    {
      logic_assert(t >= _time,
		   to_string("Time contract breached, stepping back in time from ",_time," to ",t));
      _time = t;
    }

    const time_type & time() const
    {
      return _time;
    }

    const time_type & nextAt() const
    {
      return inflows.nextTime();
    }
    
    virtual void setup_inflow(InFlow&& flow) override
    {
      inflows.setup_inflow(std::move(flow));
      transport_type::setup_inflow(inflows.inflows.back());
    }

    virtual void setup_outflow(OutFlow&& flow) override 
    {
      flow.qsize = flow.qsize > 0 ? flow.qsize : default_qsize;
      outflows.emplace_back(std::move(flow));
      transport_type::setup_outflow(outflows.back());
 
    }

    virtual void setup_sink(const string &outflow, Sink && sink) override
    {
      OutFlow & flow = outflows.at(_out_flow_index(outflow));
      flow.setup_sink(std::move(sink));
      transport_type::setup_sink(outflow, flow.sinks.back());
    }


  protected:
    virtual send_callback_type push_callback(size_t flow) override
    {
      using std::placeholders::_1;
      return std::bind(&MessageFlowMultiplexer<message_cptr, time_type, transport_type, InFlow>::push,
		       &inflows, flow, _1);
    }

    virtual send_callback_type push_callback(const string &flow) override
    {
      return push_callback(_in_flow_index(flow));
    }
    
  private:

    void _send(OutFlow * outflow, const message_cptr & m)
    {
      logic_assert(transport_type::time(m) >= time(),
		   to_string("Time contract breached : sending message with time ",
			     transport_type::time(m),
			     " in the future  (current time is ", time()));
      logic_assert(transport_type::time(m) <= inflows.nextTime(),
		   to_string("Time contract breached: sending message at ",
			     transport_type::time(m), " before arrival of next message at ",
			     inflows.nextTime()));
      logic_assert(transport_type::time(m) == outflow->time,
		   to_string("Time contract breached : Invalid message time expected ",
			     outflow->time + outflow->dt, " got ", transport_type::time(m)));
      outflow->time = outflow->time + outflow->dt;
      
      outflow->send(m);
    }
      
    unsigned int _out_flow_index(const string & name) const
    {
      for(unsigned int i=0; i<outflows.size(); i++)
	{
	  if(outflows[i].name == name) return i;
	}
      throw  std::domain_error("No outflow "+name+" defined on this node ("+this->name+")");
    }

    unsigned int _out_flow_sink_index(unsigned int flow, const string & name) const
    {
      unsigned int i = 0;
      for(auto &s : outflows[flow].sinks)
	{
	  if (s.name == name) return i;
	  i++;
	}
      throw  std::domain_error("No sink " + name + " defined on outflow " + this->name);
    }

    unsigned int _in_flow_index(const string & name)
    {
      return inflows.flow_index(name);
    }
    
  public:
    const string name;
  private:
    time_type _time;
    size_t default_qsize;
    MessageFlowMultiplexer<message_cptr, time_type, transport_type, InFlow> inflows;
    std::deque<OutFlow> outflows;  
  };


}

#endif //DSAAM_NODE_HPP
