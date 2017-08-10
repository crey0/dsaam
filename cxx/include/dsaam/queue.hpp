#ifndef DSAAM_QUEUE_HPP
#define DSAAM_QUEUE_HPP
#include <mutex>
#include <condition_variable>
#include <vector>
#include <type_traits>
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
      
      inline void operator()(int value)
      {
	assert(value <= bound);
	return value <= bound;
      }
      unsigned int bound;
    }BoundedCheck;
      
    typedef typename std::conditional<isBounded, BoundedCheck, NoCheck>::type CheckType;
    
  public:
    _Semaphore(unsigned int tokens)
      : tokens(tokens), bounded_check(tokens)
    {
    }

    void increase()
    {
      {
	std::lock_guard<std::mutex>(this->m);
	tokens +=1;
        bounded_check(tokens);
      }
      cv.notify_one();
    }

    void decrease()
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait(lk, [this]{return tokens > 0;});
      tokens -=1;
      lk.unlock();
      cv.notify_one();
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
      : max_size(max_size), head(0), tail(0), n_full_to_pop(0), n_free_to_push(max_size),
	buffer(max_size) {}

    ~Queue() {}

    void push(M &&e)
    {
      n_free_to_push.decrease();//blocking
      buffer[tail] = std::move(e);
      tail = (tail + 1) % max_size;
      n_full_to_pop.increase();
    }

    M&& pop()
    {
      n_full_to_pop.decrease();//blocking
      M && e = std::move(buffer[head]);
      head = (head - 1) % max_size;
      n_free_to_push.increase();
      return std::move(e);
    }
  private:
    unsigned int max_size;
    unsigned int head;
    unsigned int tail;
    Semaphore n_full_to_pop;
    Semaphore n_free_to_push;
    std::vector<M> buffer;
  };
    
}
#endif //DSAAM_QUEUE_HPP
