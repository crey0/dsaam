#ifndef DSAAM_SEMAPHORE_HPP
#define DSAAM_SEMAPHORE_HPP

#include <mutex>
#include <condition_variable>
#include <type_traits>

#include <dsaam/exceptions.hpp>
#include <dsaam/string_utils.hpp>


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
  
}

#endif
