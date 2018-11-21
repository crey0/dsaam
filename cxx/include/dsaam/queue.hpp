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
#ifndef DSAAM_QUEUE_HPP
#define DSAAM_QUEUE_HPP


#include <vector>
//#include <iostream>
//#include <thread>

//include <dsaam/string_utils.hpp>
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
 

    void push(const M &e)
    {
      n_free_to_push.decrease();//blocking
      //std::cout << to_string("[",std::this_thread::get_id(),"] Queue::push(",this,") idx=",tail,"\n");
      buffer[tail] = e;
      tail = (tail + 1) % max_size;
      n_full_to_pop.increase();
    }

    M pop()
    {
      n_full_to_pop.decrease();//blocking
      //std::cout << to_string("[",std::this_thread::get_id(),"] Queue::pop(",this,") idx=",head,"\n");

      M e = std::move(buffer[head]);
      head = (head + 1) % max_size;
      n_free_to_push.increase();
      return e;
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
