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
#ifndef DSAAM_OUTFLOWS_HPP
#define DSAAM_OUTFLOWS_HPP
#include<dsaam/common.hpp>
#include<dsaam/binary_heap.hpp>
#include<iostream>
namespace dsaam
{
  template<class M, class T, template <class> class F, class FMT>
  class OutMessageFlow : public OutFlow<M, T, F>
  {
    struct heap_data;
    typedef binary_heap<struct heap_data> heap_type;
    struct heap_data
    {
      heap_data(unsigned int v = 0) : v(v), h() {}
      heap_data(unsigned int v, const typename heap_type::handle_type & h) : v(v), h(h) {}
      bool operator>(const heap_data& r) const { return v > r.v;}
      bool operator<(const heap_data& r) const { return v < r.v;}
      
      unsigned int v;
      typename heap_type::handle_type h;
    };

    
    
  public:
    OutMessageFlow(string name, T start_time, T dt, std::vector<Sink>& sinks,
		   size_t max_qsize, const send_callback_type<M,F> &cb,
		   FlowType ftype)
      :  OutFlow<M, T, F>(name, start_time, dt, sinks, max_qsize, cb, ftype),
	 heap(), heap_handles()
    {
      for(size_t i = 0; i < this->sinks.size(); i++)
	{
	  typename heap_type::handle_type h = heap.push(0);
	  h->value.h = h;
	  heap_handles.push_back(h);
	}
    }

    OutMessageFlow(OutMessageFlow &&other) : OutFlow<M,T,F>(other)
    {
      this->heap = std::move(other.heap);
      this->heap_handles = std::move(other.heap_handles);
    }

    void time_callback(unsigned int subscriber, const T &)
    {
      unsigned int top;
      {
	std::lock_guard<std::mutex> lk(this->m);
      
        typename heap_type::handle_type & h = heap_handles[subscriber];
	assert(h->value.v > 0);
	h->value.v -= 1;
	heap.siftdown(h);
	top = heap.top().v;
      }
      if (top < this->qsize)
	cv.notify_one();
    }
    
    void send(const M & message)
    {
    
      std::unique_lock<std::mutex> lk(this->m);
      cv.wait(lk, [this]{return heap.top().v < this->qsize;});
      for(auto h : heap_handles)
	{
	  h->value.v += 1;
	}
      lk.unlock();
      cv.notify_one();

      OutFlow<M,T,F>::send(message);
      //std::cout << "Sent message on flow " << this->name << " time " << this->time << std::endl;
      //this->time  = this->time + this->dt;
    }
    
    void setup_sink(Sink&&)
    {
      throw std::runtime_error("Method not implemented");
    }
    
  private:
    heap_type heap;
    std::vector<typename heap_type::handle_type> heap_handles;
    std::mutex m;
    std::condition_variable cv;
  };
}

#endif
