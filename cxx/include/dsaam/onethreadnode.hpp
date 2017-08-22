#ifndef DSAAM_ONETHREADNODE_HPP
#define DSAAM_ONETHREADNODE_HPP

#include<thread>
#include<dsaam/common.hpp>
#include<dsaam/node.hpp>
#include<iostream>

namespace dsaam
{
  class OneThreadNode : public Node
  {
  public:
    OneThreadNode(string &name, Time &time, Time &dt, std::vector<InFlow> &inflows,
		  std::vector<OutFlow> &outflows, unsigned int max_qsize)
      : Node(name,time,dt,inflows,outflows,max_qsize), stopped(false),
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

    virtual void step(const Time &) = 0;

  private:
    void run()
    {
      init();
      Time t = time() + dt();
      while (!stopped)
	{
	  next();
	  //std::cout << to_string("[",time(),"] [",name,"] nextAt=", nextAt(), " TEST STEP to t=",t) << std::endl;
	  while(t <= nextAt() && !stopped) {
	    //std::cout << to_string("[",time(),"] [",name,"] nextAt=", nextAt(), " STEPPING to t=",t) << std::endl;
	    step(t); stepTime(t); t = t + dt(); }
	}
      std::cout << to_string("[",time(),"] [",name,"] STOP \n");
    }

  private:
    bool stopped;
    std::thread _thread;
  };

}

#endif
