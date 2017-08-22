#ifndef DSAAM_OUTFLOWS_HPP
#define DSAAM_OUTFLOWS_HPP
#include<dsaam/common.hpp>
#include<dsaam/binary_heap.hpp>

namespace dsaam
{
  template<class T>
  class OutMessageFlow
  {
    struct heap_data;
    typedef binary_heap<struct heap_data> heap_type;
    typedef struct heap_data
    {
      heap_data(unsigned int v = 0) : v(v), h() {}
      heap_data(unsigned int v, const typename heap_type::handle_type & h) : v(v), h(h) {}
      bool operator>(const heap_data& r) const { return v > r.v;}
      bool operator<(const heap_data& r) const { return v < r.v;}
      
      unsigned int v;
      typename heap_type::handle_type h;
    }heap_data;

    
    
  public:
    OutMessageFlow(string name, Time start_time, Time dt, std::vector<Sink> &sinks,
		   unsigned int max_qsize)
      :  name(name), max_qsize(max_qsize), next_time(start_time), dt(dt), sinks(sinks),
	 heap(), heap_handles()
    {
      for(auto s : sinks)
	{
	  typename heap_type::handle_type h = heap.push(0);
	  h->value.h = h;
	  heap_handles.push_back(h);
	}
    }

    unsigned int subscriber_index(const string & name) const
    {
      unsigned int i = 0;
      for(auto s : sinks)
	{
	  if (s.name == name) return i;
	  i++;
	}
      throw  std::domain_error("No sink " + name + " defined on outflow " + this->name);
 
    }


    void time_callback(unsigned int subscriber, const Time &)
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
    
    void send(const T & message)
    {
      logic_assert(message->time == next_time,
		   to_string("Time contract breached : Invalid message time expected ",
			     time, " got ", message->time));
    
      std::unique_lock<std::mutex> lk(this->m);
      cv.wait(lk, [this]{return heap.top().v < max_qsize;});
      for(auto h : heap_handles)
	{
	  h->value.v += 1;
	}
      lk.unlock();
      cv.notify_one();

      for(Sink s : sinks)
	{
	  if(s.send_callback) s.send_callback(message);         
	}
      
      next_time  = next_time + dt;
    }

    void set_sink_callback(unsigned int sink_idx, const ::dsaam::send_callback_type & cb)
    {
      sinks[sink_idx].send_callback = cb;
    }

  public:
    const string name;

  private:
    unsigned int max_qsize;
    Time next_time;
    Time dt;
    std::vector<Sink> sinks;
    heap_type heap;
    std::vector<typename heap_type::handle_type> heap_handles;
    std::mutex m;
    std::condition_variable cv;
  };
}

#endif
