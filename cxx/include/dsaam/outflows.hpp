#ifndef DSAAM_OUTFLOWS_HPP
#define DSAAM_OUTFLOWS_HPP
#include<dsaam/common.hpp>
#include<dsaam/binary_heap.hpp>

namespace dsaam
{
  template<class M, class T, class FMT>
  class OutMessageFlow
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
    OutMessageFlow(string name, T start_time, T dt, std::vector<Sink<M>> &sinks,
		   unsigned int max_qsize)
      :  name(name), max_qsize(max_qsize), next_time(start_time), dt(dt), sinks(sinks),
	 heap(), heap_handles()
    {
      for(size_t i = 0; i < sinks.size(); i++)
	{
	  typename heap_type::handle_type h = heap.push(0);
	  h->value.h = h;
	  heap_handles.push_back(h);
	}
    }

    unsigned int subscriber_index(const string & name) const
    {
      unsigned int i = 0;
      for(auto &s : sinks)
	{
	  if (s.name == name) return i;
	  i++;
	}
      throw  std::domain_error("No sink " + name + " defined on outflow " + this->name);
 
    }


    void time_callback(unsigned int subscriber, const T &)
    {
      unsigned int top;
      {
	std::lock_guard<std::mutex> lk(this->m);
      
        typename heap_type::handle_type & h = heap_handles[subscriber];
	assert(h->value.v > 0);
	h->value.v -= 1;
	heap.decrease(h);
	top = heap.top().v;
      }
      if (top < max_qsize)
	cv.notify_one();
    }
    
    void send(const M & message)
    {
      logic_assert(FMT::time(message) == next_time,
		   to_string("Time contract breached : Invalid message time expected ",
			     time, " got ", FMT::time(message)));
    
      std::unique_lock<std::mutex> lk(this->m);
      cv.wait(lk, [this]{return heap.top().v < max_qsize;});
      for(auto h : heap_handles)
	{
	  h->value.v += 1;
	}
      lk.unlock();
      cv.notify_one();

      for(Sink<M> s : sinks)
	{
	  if(s.send_callback) s.send_callback(message);         
	}
      
      next_time  = next_time + dt;
    }

    void set_sink_callback(unsigned int sink_idx, const ::dsaam::send_callback_type<M> & cb)
    {
      sinks[sink_idx].send_callback = cb;
    }

  public:
    const string name;

  private:
    unsigned int max_qsize;
    T next_time;
    T dt;
    std::vector<Sink<M>> sinks;
    heap_type heap;
    std::vector<typename heap_type::handle_type> heap_handles;
    std::mutex m;
    std::condition_variable cv;
  };
}

#endif
