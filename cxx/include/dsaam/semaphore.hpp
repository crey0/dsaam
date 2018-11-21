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
