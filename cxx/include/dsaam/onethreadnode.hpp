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
#ifndef DSAAM_ONETHREADNODE_HPP
#define DSAAM_ONETHREADNODE_HPP

#include<thread>
#include<dsaam/common.hpp>
#include<dsaam/node.hpp>
#include<iostream>

namespace dsaam
{
  template<class M, class T>
  struct ThreadTypes
  {
    using message_type=M;
    using message_cptr=std::shared_ptr<const M>;
    using time_type=T;
    template <class F>
    using function_type = std::function<F>;
    using send_callback_type = ::dsaam::send_callback_type<message_cptr, function_type>;
 
    

  };
  
  template<class M, class T, class FMT>
  class ThreadTransport : public ThreadTypes<M, T>, public FMT
  {
  public:
    using typename ThreadTypes<M,T>::message_type;
    using typename ThreadTypes<M,T>::message_cptr;
    using typename ThreadTypes<M,T>::time_type;
    template<class F>
    using function_type = typename ThreadTypes<M,T>::template function_type<F>;
    using typename ThreadTypes<M,T>::send_callback_type;

    struct Sink : ::dsaam::Sink
    {
      Sink(const string &flow_name, Node<ThreadTransport> & node)
	: ::dsaam::Sink(node.name)
      {
	send = ((ThreadTransport & )node).push_callback(flow_name);
      }
      send_callback_type send;
    };

    struct OutFlow :  ::dsaam::OutFlow<message_cptr, time_type, function_type, Sink>
    {
      OutFlow(string name, const time_type &time, const time_type &dt,
	      const std::vector<Sink> &sinks = {}, size_t qsize = 0)
	:  ::dsaam::OutFlow<message_cptr, time_type, function_type, Sink>(name,time,dt,sinks, qsize)
      {
	//using std::placeholders::_1;
	//this->send = std::bind(&OutFlow::_send, this, _1);
      }

      void send(const message_cptr & m)
      {
	for(auto &s : this->sinks)
	  {
	    s.send(m);
	  }
      }      
    };

    struct InFlow : ::dsaam::InFlow<message_cptr, time_type, function_type>
    {
      InFlow(string name,
	     const time_type& time, const time_type &dt,
	     const message_callback_type<message_cptr,time_type,function_type> &callback,
	     size_t qsize=0)
	: ::dsaam::InFlow<message_cptr, time_type, function_type>(name, time, dt, qsize, callback)
      {}

      void time_callback(const time_type&)
      {
      }
    };

    
    virtual void setup_inflow(InFlow&)
    {
    }
    
    virtual void setup_outflow(OutFlow&)
    {      
    }

    virtual void setup_sink(const string &, Sink &)
    {
    }
    
  protected:
    virtual void setup_inflow(InFlow&&) = 0;
    virtual void setup_outflow(OutFlow&&) = 0;
    virtual void setup_sink(const string &, Sink &&) = 0;
    
    virtual typename ThreadTypes<M,T>::send_callback_type push_callback(size_t flow) = 0;
    virtual typename ThreadTypes<M,T>::send_callback_type push_callback(const string & name) = 0;
    
    
  };
  
  template<class T>
  class OneThreadNode : public Node<T>
  {
  public:
    using mpointer=typename Node<T>::message_cptr;
    using InFlow=typename Node<T>::InFlow;
    using OutFlow=typename Node<T>::OutFlow;
    using time_type=typename Node<T>::time_type;
    
    OneThreadNode(const string &name, const time_type &time, const time_type &dt, size_t max_qsize)
      : Node<T>(name,time,max_qsize), dt(dt), stopped(false),
	_thread() {} 

    ~OneThreadNode()
    {
      //Prevents destruction of joinable thread which is illegal
      stop();
      if(_thread.joinable()) join();
    }
    
    void start()
    {
      if(!_thread.joinable()) _thread = std::thread(&OneThreadNode::run, this);
    }

    void join()
    {
      _thread.join();
    }
  
    void stop()
    {
      stopped = true;
    }

    virtual void init() = 0;

    virtual void step(const time_type &) = 0;

  private:
    
    void run()
    {
      init();
      time_type t = this->time() + dt;
      while (!stopped)
	{
	  this->next();
	  //std::cout << to_string("[",time(),"] [",name,"] nextAt=", nextAt(), " TEST STEP to t=",t) << std::endl;
	  while(t <= this->nextAt() && !stopped)
	    {
	      //std::cout << to_string("[",time(),"] [",name,"] nextAt=", nextAt(), " STEPPING to t=",t) << std::endl;
	      step(t); this->stepTime(t); t = t + dt;
	    }
	}
      //std::cout << to_string("[",this->time(),"] [",this->name,"] STOP \n");
    }


    
  public:
    const time_type dt;
  private:
    bool stopped;
    std::thread _thread;
  };

}

#endif
