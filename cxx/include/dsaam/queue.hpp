#ifndef DSAAM_QUEUE_HPP
#define DSAAM_QUEUE_HPP

#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <type_traits>
#include <dsaam/exceptions.hpp>
#include <dsaam/string_utils.hpp>
#include <iostream>

namespace dsaam
{
  template<bool isBounded>
  class _Semaphore
  {
  private:
    typedef struct NoCheck
    {
      NoCheck(unsigned int) {}
      
      inline void operator()(int)
      {
	;
      }
    }NoCheck;

    typedef struct BoundedCheck
    {
      BoundedCheck(unsigned int bound) : bound(bound) {}
      
      inline void operator()(unsigned int value)
      {
	logic_assert(value <= bound,
		     to_string("Bounded semaphore has exceeded bounds ",value,">",bound));
      }
      unsigned int bound;
    }BoundedCheck;
      
    typedef typename std::conditional<isBounded, BoundedCheck, NoCheck>::type CheckType;
    
  public:
    _Semaphore(unsigned int tokens, unsigned int bound)
      : tokens(tokens), bounded_check(bound) {}
    _Semaphore(unsigned int tokens)
      : tokens(tokens), bounded_check(tokens) {}

    void increase()
    {
      {
	std::lock_guard<std::mutex> lk(this->m);
        ++tokens;
        bounded_check(tokens);
      }
      cv.notify_one();
    }

    void decrease()
    {
      std::unique_lock<std::mutex> lk(this->m);
      cv.wait(lk, [this]{return tokens > 0;});
      --tokens;
      lk.unlock();
      cv.notify_one();
    }

    unsigned int count()
    {
      return tokens;
    }
    
  private:    
    unsigned int tokens;
    CheckType bounded_check;
    std::mutex m;
    std::condition_variable cv;
    
  };
  
  typedef _Semaphore<false> Semaphore;
  typedef _Semaphore<true> BoundedSemaphore;
  
  template<typename M>
  class Queue
  {
  public:
    Queue(unsigned int max_size)
      : max_size(max_size), head(0), tail(0), n_full_to_pop(0, max_size), n_free_to_push(max_size),
	buffer(max_size)
    {
      if(max_size == 0) throw  std::length_error("Queue min size is 1");
      std::cout << "Queue::Queue(" << this << ", max_size=" << max_size << ")" << std::endl;
    }

    ~Queue() {std::cout << "Queue::~Queue(" << this << ", max_size=" << max_size << ")" << std::endl; }
 

    void push(M &&e)
    {
      n_free_to_push.decrease();//blocking
      std::cout << to_string("[",std::this_thread::get_id(),"] Queue::push(",this,") idx=",tail,"\n");
      buffer[tail] = std::forward<M>(e);
      tail = (tail + 1) % max_size;
      n_full_to_pop.increase();
    }

    M&& pop()
    {
      n_full_to_pop.decrease();//blocking
      std::cout << to_string("[",std::this_thread::get_id(),"] Queue::pop(",this,") idx=",head,"\n");

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
