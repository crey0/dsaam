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
#ifndef DSAAM_INFLOWS_HPP
#define DSAAM_INFLOWS_HPP

#include<dsaam/queue.hpp>
#include<dsaam/binary_heap.hpp>
#include<deque>

namespace dsaam
{
  template<class M, class T, class FMT>
  class MessageQueue : public Queue<M>
  {
  public:
    MessageQueue(const T &start_time,const T &dt, unsigned int max_size)
      : Queue<M>(max_size), nextTime(start_time), dt(dt) {}

    M pop()
    {
      M m = Queue<M>::pop();
      logic_assert(FMT::time(m) == nextTime,
		   to_string("Time contract breached : Invalid message time expected ",
			     nextTime, " got ", FMT::time(m)));
      nextTime = nextTime + dt;
      return m;
    }

    const T & nextAt() const
    {
      return nextTime; 
    }
      
  private:
    T nextTime;
    T dt;
  };

  template<typename M, typename T, typename FMT>
  inline bool operator<(const MessageQueue<M, T, FMT> &l, const MessageQueue<M, T, FMT> &r)
  { return l.nextAt() < r.nextAt();}

  template<typename M, typename T, typename FMT>
  inline bool operator>(const MessageQueue<M, T, FMT> &l, const MessageQueue<M, T, FMT> &r)
  { return r < l;}
  
  template<class M, class T, class FMT, class I>
  class MessageFlowMultiplexer
  {
    using InFlow = I;
    class heap_data;
    typedef  binary_heap<heap_data, std::greater<heap_data>> heap_type;
    
    class heap_data
    {
    public:
      heap_data(InFlow &flow, MessageQueue<M, T, FMT> & queue) : flow(flow), queue(queue) {}
      InFlow &flow;
      MessageQueue<M, T, FMT> & queue;
      typename heap_type::handle_type handle;

      bool operator>(const heap_data& r) const { return queue > r.queue;}
      bool operator<(const heap_data& r) const { return queue > r.queue;}
    };


  public:
    MessageFlowMultiplexer(const T &time, unsigned int default_qsize):
      inflows(), time(time), default_qsize(default_qsize), heap(), queues()
    {}

    size_t size()
    {
      return queues.size();
    }
    
    void setup_inflow(InFlow&& flow)
    {
      auto p = std::unique_ptr<MessageQueue<M, T, FMT>>(
		    new MessageQueue<M, T, FMT>(time, flow.dt,
						flow.qsize>0?flow.qsize:default_qsize));
      queues.push_back(std::move(p));
      inflows.push_back(std::move(flow));
      
      size_t index = queues.size()-1;
      typename heap_type::handle_type h = heap.push(heap_data(inflows.back(),
						    *this->queues[index]));
      (*h).value.handle = h;
      
    }
    
    unsigned int flow_index(const string & name) const
    {
      unsigned int i = 0;
      for(auto s : inflows)
	{
	  if (s.name == name) return i;
	  i++;
	}
      throw  std::domain_error("No inflow " + name + " defined on this node");
 
    }
  
    void push(unsigned int flow_index, const M & message)
    {
      //std::cout << to_string("[",std::this_thread::get_id(),"] pushing on flow ",flow_index,
      //			     "/",&queues[flow_index], "\n");
      queues[flow_index]->push(message);
    }

    void next()
    {
      heap_data & q = const_cast<heap_data &>(heap.top());
      //std::cout << to_string("[",std::this_thread::get_id(),"] [",time,"] popping on queue ",q.flow.name,"\n");
      auto m = q.queue.pop();
      heap.siftdown(q.handle);
      q.flow.time_callback(FMT::time(m));
      q.flow.callback(std::move(m), nextTime());
    }

    const T & nextTime() const
    {
      auto & q = heap.top();
      //std::cout << to_string("[",time,"] next message InFlow ",q.flow.name, " at ", q.queue.nextAt(), "\n");
   
      return q.queue.nextAt();
    }

  public:
    std::deque<InFlow> inflows;
  private:
    T time;
    unsigned int default_qsize;
    heap_type heap;
    std::vector<std::unique_ptr<MessageQueue<M, T, FMT>>> queues;
   
    
  };
}

#endif
