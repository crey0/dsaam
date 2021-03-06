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
#ifndef DSAAM_NODE_HPP
#define DSAAM_NODE_HPP

#include<dsaam/common.hpp>
#include<dsaam/inflows.hpp>
#include<dsaam/outflows.hpp>
#include<dsaam/time.hpp>
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


    struct greater_outflow
    {
      bool operator()(const OutFlow & left, const OutFlow& right)
      {
	return left.time > right.time
	  || (left.time == right.time
	      && left.ftype == FlowType::OBS && right.ftype == FlowType::PRED);
      };
    };

    using OutFlowHandle = typename binary_heap<OutFlow, greater_outflow>::handle_type;


    Node(const string &name, const time_type &time, size_t default_qsize = 0) :
      name(name), _time(time), default_qsize(default_qsize),
      inflows(time, default_qsize), outflows() {}
    
    void next()
    {
      logic_assert(outflows.size() == 0 ||
		   ((nextOut().ftype == FlowType::OBS && nextAt() <= nextOut().time)
		    || (nextOut().ftype == FlowType::PRED && nextAt() < nextOut().time)),
		   to_string("Time contract breached: attempting to consume next message at ",
			     nextAt()," but next outgoing message should be sent before at ",
			     nextOut().time, " on flow ", nextOut().name));
      this->inflows.next();
    }

    send_callback_type send_callback(unsigned int flow)
    {
      using std::placeholders::_1;
      return std::bind(&Node::_send, this, outflows[flow], _1);
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

    const OutFlow & nextOut() const
    {
      return next_out.top();
    }
    
    virtual void setup_inflow(InFlow&& flow) override
    {
      inflows.setup_inflow(std::move(flow));
      transport_type::setup_inflow(inflows.inflows.back());
    }

    virtual void setup_outflow(OutFlow&& flow) override 
    {
      flow.qsize = flow.qsize > 0 ? flow.qsize : default_qsize;
      auto h = next_out.push(std::move(flow));
      outflows.push_back(h);
      transport_type::setup_outflow(h->value);
    }

    virtual void setup_sink(const string &outflow, Sink && sink) override
    {
      OutFlow & flow = outflows.at(_out_flow_index(outflow))->value;
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

    void _send(OutFlowHandle handle, const message_cptr & m)
    {
      auto & outflow = handle->value; 
      logic_assert(transport_type::time(m) >= time(),
		   to_string("Time contract breached : sending message with time ",
			     transport_type::time(m),
			     " in the future  (current time is ", time()));
      logic_assert(transport_type::time(m) == outflow.time,
		   to_string("Time contract breached : Invalid message time expected ",
			     outflow.time, " got ", transport_type::time(m)));
      logic_assert(inflows.size() == 0
		   || ((outflow.ftype == FlowType::PRED && transport_type::time(m) <= nextAt())
		       || (outflow.ftype == FlowType::OBS && transport_type::time(m) < nextAt())),
		   to_string("Time contract breached: sending message at ",
			     transport_type::time(m), " before arrival of next message at ",
			     inflows.nextTime()));
      
      outflow.time = outflow.time + outflow.dt;     
      outflow.send(m);
      next_out.update(handle);
      
    }
      
    unsigned int _out_flow_index(const string & name) const
    {
      for(unsigned int i=0; i<outflows.size(); i++)
	{
	  if(outflows[i]->value.name == name) return i;
	}
      throw  std::domain_error("No outflow "+name+" defined on this node ("+this->name+")");
    }

    unsigned int _out_flow_sink_index(unsigned int flow, const string & name) const
    {
      unsigned int i = 0;
      for(auto &s : outflows[flow]->value.sinks)
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
    std::vector<OutFlowHandle> outflows;
    binary_heap<OutFlow, greater_outflow> next_out;
    
  };


}

#endif //DSAAM_NODE_HPP
