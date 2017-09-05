#ifndef DSAAM_QUEUE_HPP
#define DSAAM_QUEUE_HPP


#include <vector>
//#include <iostream>

//#include <dsaam/string_utils.hpp>
#include <dsaam/semaphore.hpp>

namespace dsaam
{
 
  template<typename M>
  class Queue
  {
  public:
    Queue(unsigned int max_size)
      : max_size(max_size), head(0), tail(0), n_full_to_pop(0, max_size), n_free_to_push(max_size),
	buffer(max_size)
    {
      if(max_size == 0) throw  std::length_error("Queue min size is 1");
    }

    ~Queue() {}
 

    void push(M &&e)
    {
      n_free_to_push.decrease();//blocking
      //std::cout << to_string("[",std::this_thread::get_id(),"] Queue::push(",this,") idx=",tail,"\n");
      buffer[tail] = std::forward<M>(e);
      tail = (tail + 1) % max_size;
      n_full_to_pop.increase();
    }

    M&& pop()
    {
      n_full_to_pop.decrease();//blocking
      //std::cout << to_string("[",std::this_thread::get_id(),"] Queue::pop(",this,") idx=",head,"\n");

      M && e = std::move(buffer[head]);
      head = (head + 1) % max_size;
      n_free_to_push.increase();
      return std::move(e);
    }
  private:
    unsigned int max_size;
    unsigned int head;
    unsigned int tail;
    BoundedSemaphore n_full_to_pop;
    BoundedSemaphore n_free_to_push;
    std::vector<M> buffer;
  };
    
}
#endif //DSAAM_QUEUE_HPP
