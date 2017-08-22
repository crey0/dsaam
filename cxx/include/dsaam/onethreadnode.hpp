#ifndef DSAAM_ONETHREADNODE_HPP
#define DSAAM_ONETHREADNODE_HPP

#include<thread>
#include<dsaam/common.hpp>
#include<dsaam/node.hpp>
#include<iostream>

namespace dsaam
{
  template<class M, class T, class FMT>
  class OneThreadNode : public Node<M, T, FMT>
  {
  public:
    using mpointer=typename Node<M, T, FMT>::mpointer;
    using InFlow=typename Node<M, T, FMT>::InFlow;
    using OutFlow=typename Node<M, T, FMT>::OutFlow;
    
    OneThreadNode(string &name, T &time, T &dt, std::vector<InFlow> &inflows,
		  std::vector<OutFlow> &outflows, unsigned int max_qsize)
      : Node<M,T,FMT>(name,time,dt,inflows,outflows,max_qsize), stopped(false),
	_thread() {} 
    
    void start()
    {
      if(!_thread.joinable()) _thread = std::thread(&OneThreadNode::run, this);
    }
  
    void stop()
    {
      stopped = true;
    }

    void join()
    {
      _thread.join();
    }

    virtual void init() = 0;

    virtual void step(const T &) = 0;

  private:
    void run()
    {
      init();
      T t = this->time() + this->dt();
      while (!stopped)
	{
	  this->next();
	  //std::cout << to_string("[",time(),"] [",name,"] nextAt=", nextAt(), " TEST STEP to t=",t) << std::endl;
	  while(t <= this->nextAt() && !stopped) {
	    //std::cout << to_string("[",time(),"] [",name,"] nextAt=", nextAt(), " STEPPING to t=",t) << std::endl;
	    step(t); this->stepTime(t); t = t + this->dt(); }
	}
      std::cout << to_string("[",this->time(),"] [",this->name,"] STOP \n");
    }

  private:
    bool stopped;
    std::thread _thread;
  };

}

#endif
