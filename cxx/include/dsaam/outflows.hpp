#ifndef DSAAM_OUTFLOWS_HPP
#define DSAAM_OUTFLOWS_HPP
#include<dsaam/common.hpp>
#include<dsaam/binary_heap.hpp>

namespace dsaam
{
  template<class M, class T, template <class> class F, class FMT>
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
    OutMessageFlow(string name, T start_time, T dt, OutFlow<M,T,F>& outflow,
		   unsigned int max_qsize)
      :  name(name), max_qsize(max_qsize), next_time(start_time), dt(dt), outflow(outflow),
	 heap(), heap_handles()
    {
      for(size_t i = 0; i < outflow.sinks.size(); i++)
	{
	  typename heap_type::handle_type h = heap.push(0);
	  h->value.h = h;
	  heap_handles.push_back(h);
	}
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
    
      std::unique_lock<std::mutex> lk(this->m);
      cv.wait(lk, [this]{return heap.top().v < max_qsize;});
      for(auto h : heap_handles)
	{
	  h->value.v += 1;
	}
      lk.unlock();
      cv.notify_one();

      outflow.send(message);
      
      next_time  = next_time + dt;
    }
    
  public:
    const string name;

  private:
    unsigned int max_qsize;
    T next_time;
    T dt;
    OutFlow<M,T,F> outflow;
    heap_type heap;
    std::vector<typename heap_type::handle_type> heap_handles;
    std::mutex m;
    std::condition_variable cv;
  };
}

#endif
