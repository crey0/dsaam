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
#ifndef DSAAM_ROS_NODE_HPP
#define DSAAM_ROS_NODE_HPP
#include<ros/ros.h>

//Horrible hack. Maybe introduce a time and a duration type in DSAAM (seems standard)?
namespace ros
{
  inline Time operator+(const Time& t1, const Time& t2)
  {
    return t1 + ros::Duration(t2.sec, t2.nsec);
  }
}
#include<ros/message_traits.h>
#include<std_msgs/Header.h>
#include<dsaam/node.hpp>
#include<unordered_set>

namespace dsaam { namespace ros
{
  
  class CountSubListener
  {
  public:
    CountSubListener(const std::vector<string> &peers)
      : CountSubListener(peers, peers.size())
    {}
    
    CountSubListener(size_t num_peers)
      : CountSubListener(std::vector<string>(), num_peers)
    {}

    void peer_subscribe(const ::ros::SingleSubscriberPublisher & sub)
    {
      if(peers.size() > 0 && peers.count(sub.getSubscriberName().substr(1)) == 0)
	{
	  std::cout << "Peer " << sub.getSubscriberName() << " subscribed to "
		    <<  sub.getTopic() << " (unawaited)" << std::endl;
	  return;
	}
	
      std::cout << "Peer " << sub.getSubscriberName() << " subscribed to "
		<<  sub.getTopic() << " (n=" << num_peers << ")" << std::endl;
      sub_sem.increase();
    }

    ::ros::SubscriberStatusCallback peer_subscribe_callback()
    {
      return _peer_sub_cb;
    }

    void peer_unsubscribe(const ::ros::SingleSubscriberPublisher & sub)
    {
      throw std::runtime_error(to_string("No unsubscription allowed for topic ",sub.getTopic(),
					 " requested by ",sub.getSubscriberName(),"\n"));
    }

    ::ros::SubscriberStatusCallback peer_unsubscribe_callback()
    {
      return  _peer_unsub_cb;
    }

    void wait()
    {
      
      //std::cout << "Waiting on " << num_peers << " peers " << std::endl;
      for(size_t i=0; i< num_peers; i++)
	{
	  sub_sem.decrease();
	}
      //std::cout << "Waiting DONE" << std::endl;
    }

  private:
    CountSubListener(const std::vector<string> &peers, size_t num_peers)
      : peers(peers.begin(), peers.end()), num_peers(num_peers), sub_sem(0)
      {	
	//TODO change this ?
	_peer_sub_cb =  boost::bind(&CountSubListener::peer_subscribe, this, _1);
	_peer_unsub_cb = boost::bind(&CountSubListener::peer_unsubscribe, this, _1);
      }
    
  private:
    ::ros::SubscriberStatusCallback _peer_sub_cb;
    ::ros::SubscriberStatusCallback _peer_unsub_cb;
    std::unordered_set<string> peers;
    size_t num_peers;
    Semaphore sub_sem;
    
  };
  
  struct ROSMessagePointerHolder;
  using message_type = ROSMessagePointerHolder;
  using message_cptr = boost::shared_ptr<const message_type>;
  using time_type = ::ros::Time;
  template<class F>
  using function_type = boost::function<F>;

  using void_cptr = boost::shared_ptr<const void>;

  template<class M>
  const time_type& message_timestamp(const void_cptr& m)
  {
    return *::ros::message_traits::TimeStamp<M>::pointer(*static_cast<const M*>(m.get()));
  }

  
  struct ROSMessagePointerHolder
  {
    template<class M> ROSMessagePointerHolder(boost::shared_ptr<const M> && m)
      : m(m), timestamp_fun(&message_timestamp<M>)
    {}

    template<class M> ROSMessagePointerHolder(const boost::shared_ptr<const M> & m)
      : m(m), timestamp_fun(&message_timestamp<M>)
    {}

    template<class M> ROSMessagePointerHolder(M *m)
    : m(m), timestamp_fun(&message_timestamp<M>)
    {
    }

    const ::ros::Time & time() const
    {
      return timestamp_fun(m);
    }

    template<class P> 
    boost::shared_ptr<const P> get_shared() const
    {
      return boost::static_pointer_cast<const P>(m);
    }

    template<class P> 
    const P& get_ref() const
    {
      return *(static_cast<const P*>(m.get()));
    }

    template<class P>
    static void publish(::ros::Publisher &pub,
				 boost::shared_ptr<const ROSMessagePointerHolder> m)
    {
      pub.publish(m->get_ref<P>());
    }

  private:
    const void_cptr m;
    const ::ros::Time & (*timestamp_fun)(const void_cptr&);
  };

  template<class P1, class P2, class... Args>
  struct message_conversion_callback
  {
    using callback_type = boost::function<void(P2, Args...)>;
    
    message_conversion_callback(const callback_type &callback) : callback(callback) {}

    void operator()(P1 &p1, Args... args)
    {
      P2 p2 = boost::static_pointer_cast<P2>(std::move(p1));
      callback(p2, &args...);
    }
    
    callback_type callback;
  };
  
  struct ros_header_stamp
  {
    using message_type = ::dsaam::ros::message_type;
    using message_cptr= ::dsaam::ros::message_cptr;
    using time_type = ::dsaam::ros::time_type;

    using time_message_type=std_msgs::Header;
    using time_message_cptr=boost::shared_ptr<const time_message_type>;
    
    static const time_type& time(const message_cptr &m)
    {
      return m->time();
    }

    static std_msgs::Header time_message(const time_type &t)
    {
      std_msgs::Header h;
      h.stamp = t;
      return h;
    }

    static const time_type& time_message(const boost::shared_ptr<const time_message_type> &m)
    {
      return m->stamp;
    }
  };
  
  class RosTransport : public ros_header_stamp
  {
  public:

    //required by Node
    using ros_header_stamp::message_type;
    using ros_header_stamp::message_cptr;
    using ros_header_stamp::time_type;
    template<class F>
    using function_type = dsaam::ros::function_type<F>;
    template<class M>
    using shared_cptr_t = boost::shared_ptr<const M>;
    using send_callback_type = ::dsaam::send_callback_type<message_cptr, function_type>;
    using time_callback_type = ::dsaam::time_callback_type<time_type, function_type>;
    using InFlow = ::dsaam::InFlow<message_cptr, time_type, function_type>;
    using Sink = ::dsaam::Sink;
    using OutFlow = ::dsaam::OutMessageFlow<message_cptr, time_type, function_type, ros_header_stamp>;

    using ros_header_stamp::time_message_type;

    RosTransport() : n(), async_spinner(1)
    { 
    }

    void init_ros()
    {
      async_spinner.start();
      for(auto & l : sub_listeners)
	{
	  l->wait();
	}
    }

    template<class S>
    typename std::enable_if<::ros::message_traits::HasHeader<S>::value>::type
    setup_subscriber(const string & node_name, const string & ifname, const string & name,
		     const time_type& time, const time_type& dt, 
		     const dsaam::message_callback_type<shared_cptr_t<S>,
		     time_type, function_type> &m_cb,
		     size_t max_qsize = 0)
    {

      //Setup callback for delivering messages
      auto message_cwrapper = std::bind(&_dispatch_callback<S>, m_cb,
					std::placeholders::_1, std::placeholders::_2);
      auto time_callback = _ros_out_time_callback(node_name, ifname, name, max_qsize);
      //create inflow
      auto flow = InFlow(ifname, time, dt, max_qsize, message_cwrapper, time_callback);
      setup_inflow(std::move(flow));

      //After InFlow has been setup on node, create get push cb and create ros subscriber
      send_callback_type cb_push = this->push_callback(ifname);
      boost::function<void(const boost::shared_ptr<const S>)> cb = \
	std::bind(&RosTransport::_ros_callback<S>, cb_push, std::placeholders::_1);
      subs.push_back(n.subscribe<S>(ifname, max_qsize, cb));
      
    }

    template<class S>
    typename std::enable_if<::ros::message_traits::HasHeader<S>::value>::type
    setup_publisher(const string &ofname,
		    const time_type &time, const time_type& dt,
		    std::vector<string> subscribers,
		    size_t max_qsize = 0,
		    FlowType ftype = FlowType::PRED)
    {      
      auto subcount = std::unique_ptr<CountSubListener>(new CountSubListener(subscribers));
      ::ros::SubscriberStatusCallback connect_cb = subcount->peer_subscribe_callback();
      //&peer_subscribe_dummy;
      pubs.push_back(n.advertise<S>(ofname, max_qsize,
				    connect_cb));
      sub_listeners.push_back(std::move(subcount));
      std::vector<Sink> sinks = {begin(subscribers), end(subscribers)};
      //create ouflow and setup sinks
      send_callback_type cb = std::bind(&ROSMessagePointerHolder::publish<S>,
					std::ref(pubs.back()), std::placeholders::_1);
      OutFlow flow {ofname, time, dt, sinks, max_qsize, cb, ftype};
      setup_outflow(std::move(flow));
    }

    virtual void setup_inflow(InFlow&)
    {
    }
    
    virtual void setup_outflow(OutFlow& flow)
    {
      auto name = flow.name;
      for(size_t i = 0; i < flow.sinks.size(); i++)
	{
	  auto &s = flow.sinks[i];
	  auto cb1 =  std::bind(&OutFlow::time_callback, &flow, i , std::placeholders::_1);
	  //std::function<void(const time_message_cptr&)> cb =		
	  //  std::bind(&RosTransport::_ros_receive_time, this, cb1, std::placeholders::_1);
	  auto cb = _ros_receive_time(cb1);
	  subs.push_back(n.subscribe<time_message_type>(name + "/time/" + s.name, flow.qsize, cb));
	}
    }

    virtual void setup_sink(const string &, Sink &)
    {
    }
  protected:
    virtual void setup_inflow(InFlow&&) = 0;
    virtual void setup_outflow(OutFlow&&) = 0;
    virtual void setup_sink(const string &, Sink &&) = 0;
    
  private:

    boost::function<void(const time_type&)> _ros_out_time_callback(const string &node_name,
								   const string &ifname,
								   const string &name,
								   size_t max_qsize)
    {
      auto subcount = std::unique_ptr<CountSubListener>(new CountSubListener({node_name}));
      ::ros::SubscriberStatusCallback connect_cb = subcount->peer_subscribe_callback();
      pubs.push_back(n.advertise<time_message_type>(ifname + "/time/" + name, max_qsize,
						    connect_cb));
      sub_listeners.push_back(std::move(subcount));
      auto &pub = pubs.back();
      using std::placeholders::_1;
      return std::bind(&RosTransport::_ros_publish_time, pub, _1);
    }

    struct _ros_receive_time
    {
      _ros_receive_time(const time_callback_type& cb)
	:cb(cb) {}
      void operator ()(const time_message_cptr &time)
      {
	cb(time_message(time));
      }
      const time_callback_type cb;
    };
    
    static void _ros_publish_time(::ros::Publisher & pub, const time_type &time)
    {
      auto m = time_message(time);
      pub.publish(m);
    }

    template<class S>
    static void _ros_callback(send_callback_type &cb,
			      const shared_cptr_t<S> & m)
    {
      auto mp = boost::shared_ptr<const ROSMessagePointerHolder>
	{new ROSMessagePointerHolder{m}};
      cb(mp);
    }

    template<class S>
    static void _dispatch_callback( dsaam::message_callback_type<shared_cptr_t<S>,
				    time_type, function_type> &cb, const message_cptr & mw,
				    const time_type & nt)
    {
      shared_cptr_t<S> m = mw->get_shared<S>();
      cb(m, nt);
    }

    virtual send_callback_type push_callback(size_t flow) = 0;
    virtual send_callback_type push_callback(const string & name) = 0;
    
  private:
    ::ros::NodeHandle n;
    ::ros::AsyncSpinner async_spinner;
    std::vector<::ros::Subscriber> subs;
    std::vector<::ros::Publisher> pubs;
    std::vector<std::unique_ptr<CountSubListener>> sub_listeners;
  };
  
}}
#endif
