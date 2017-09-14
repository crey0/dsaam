#ifndef DSAAM_ROS_NODE_HPP
#define DSAAM_ROS_NODE_HPP

#include<ros/ros.h>
#include<ros/message_traits.h>
#include<std_msgs/Header.h>
#include <dsaam/node.hpp>

/*TODO:
  P1 : Subscriber callbacks need to be passed with bare method pointers + object
  P2 : SubscriberStatusCallback is a typedef for boost::function !
  P3 : Check if it will give us boost::shared_pointer or std::shared_pointer (unclear)
  when using subscribe ???
  P4 : Finish implementing all the init

*/
namespace dsaam { namespace ros
{
  
  class CountSubListener
  {
  public:
    CountSubListener(size_t num_peers) : num_peers(num_peers), sub_sem(0) {}

    void peer_subscribe(::ros::SingleSubscriberPublisher &)
    {
      sub_sem.decrease();
    }

    ::ros::SubscriberStatusCallback peer_subscribe_callback()
    {
      return boost::bind(&CountSubListener::peer_subscribe, this);
    }

    void peer_unsubscribe(::ros::SingleSubscriberPublisher & sub)
    {
      throw std::runtime_error(to_string("No unsubscription allowed for topic ",sub.getTopic(),
					 " requested by ",sub.getSubscriberName(),"\n"));
	}

    ::ros::SubscriberStatusCallback peer_unsubscribe_callback()
    {
      return boost::bind(&CountSubListener::peer_unsubscribe, this);
    }

    void wait()
    {
      for(size_t i=0; i< num_peers; i++)
	{
	  sub_sem.increase();
	}
    }
    
  private:
    size_t num_peers;
    Semaphore sub_sem;
    
  };
  
  struct ROSMessagePointerHolder;
  using message_type = ROSMessagePointerHolder;
  using message_cptr = boost::shared_ptr<const message_type>;
  using void_cptr = boost::shared_ptr<const void>;
  using time_type = ::ros::Time;
  template<class F>
  using function_type = boost::function<F>;
  
  template<class M>
  const time_type& message_timestamp(const void_cptr& m)
  {
    return *::ros::message_traits::TimeStamp<M>(*static_cast<const M*>(m.get()));
  }

  
  struct ROSMessagePointerHolder
  {
    template<class M> ROSMessagePointerHolder(boost::shared_ptr<const M> && m)
      : m(m), timestamp_fun(&message_timestamp<M>)
    {}

    const ::ros::Time & time() const
    {
      return timestamp_fun(m);
    }

    template<class P>
    P get()
    {
      return boost::static_pointer_cast<P>(m);
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
    using message_type = void;
    using time_message_type=std_msgs::Header;
    using time_message_cptr=boost::shared_ptr<time_message_type>;
    using mpointer=boost::shared_ptr<const ROSMessagePointerHolder>;
    
    static const ::ros::Time& time(const mpointer &m)
    {
      return m->time();
    }

    static std_msgs::Header time_message(const ::ros::Time &t)
    {
      std_msgs::Header h;
      h.stamp = t;
      return h;
    }

    static const ::ros::Time & time_message(const boost::shared_ptr<const time_message_type> &m)
    {
      return m->stamp;
    }
  };
  
  class RosTransport : public ros_header_stamp
  {
  public:

    //required by Node
    using message_type = ::dsaam::ros::message_type;
    using message_cptr = ::dsaam::ros::message_cptr;
    using time_type = time_type;
    template<class F>
    using function_type = boost::function<F>;
    
    using ros_header_stamp = ros_header_stamp;
    using ros_header_stamp::time_message_type;
    using send_callback_type = ::dsaam::send_callback_type<message_cptr, function_type>;
    using time_callback_type = ::dsaam::time_callback_type<time_type, function_type>;
    using InFlow = ::dsaam::InFlow<message_cptr, time_type, function_type>;
    using OutFlow = ::dsaam::OutMessageFlow<message_cptr, time_type, function_type, ros_header_stamp>;
    
    RosTransport()
    { 
    }

    void ros_init()
    {
      for(auto & l : sub_listeners)
	{
	  l->wait();
	}
    }

    template<class S,
	     class = typename std::enable_if<::ros::message_traits::HasHeader<S>::value>::type>
    void setup_subscriber(const string & ifname, const string & name,
			  const time_type& time, const time_type& dt, size_t max_qsize,
			  const dsaam::message_callback_type<S, time_type, function_type> &m_cb)
    {
      //Setup ROS subscriber
      auto cb = this->push_callback(ifname);
      subs.push_back(n.subscribe<S>(ifname, max_qsize, cb));

      //Setup callback for delivering messages
      auto message_cwrapper = message_conversion_callback<S, time_type>(m_cb);
      auto time_callback = _ros_out_time_callback(ifname, name, max_qsize);
      //create inflow
      auto flow = InFlow(ifname, time, dt, max_qsize, message_cwrapper, time_callback);
      setup_inflow(flow);
    }

    template<class S,
	     class  = typename std::enable_if<::ros::message_traits::HasHeader<S>::value>::type>
    void setup_publisher(const string &ofname,
			 const time_type &time, const time_type& dt,
			 std::vector<string> subscribers,
			 size_t max_qsize = 0)
    {
      auto name = ::ros::this_node::getName();
      auto subcount = std::unique_ptr<CountSubListener>(new CountSubListener(subscribers.size()));
      pubs.push_back(n.advertise<S>(ofname, max_qsize,
				    subcount->peer_subscribe_callback()));
      sub_listeners.push_back(std::move(subcount));
      std::vector<Sink> sinks = {begin(subscribers), end(subscribers)};
      //create ouflow and setup sinks
      OutFlow flow {name, time, dt, sinks, max_qsize };
      setup_outflow(flow);
    }

    virtual void setup_inflow(InFlow&)
    {
    }
    
    virtual void setup_outflow(OutFlow& flow)
    {
      auto name = ::ros::this_node::getName();
      for(size_t i = 0; i < flow.sinks.size(); i++)
	{
	  auto &s = flow.sinks[i];
	  auto cb = std::bind<void(const time_type&)>(&RosTransport::_ros_receive_time, this,
						      std::bind(&OutFlow::time_callback, &flow, i ,_1),
						      _1);
	  subs.push_back(n.subscribe<time_type>(name + "/time/" + s.name, flow.qsize, cb));
	}
    }

    virtual void setup_sink(const string &, Sink &)
    {
    }

    
  private:

    boost::function<void(const time_type&)> _ros_out_time_callback(const string &ifname,
								   const string &name,
								   size_t max_qsize)
    {
      auto subcount = std::unique_ptr<CountSubListener>(new CountSubListener(1));
      pubs.push_back(n.advertise<time_message_type>(ifname + "/time/" + name, max_qsize,
						    subcount->peer_subscribe_callback()));
      sub_listeners.push_back(std::move(subcount));
      auto &pub = pubs.back();
      using std::placeholders::_1;
      return std::bind<void(const time_type&)>(&RosTransport::_ros_publish_time, this, pub, _1);
    }

    void _ros_receive_time(const time_callback_type& cb, const time_message_cptr &time)
    {
      cb(time_message(time));
    }
    
    void _ros_publish_time(::ros::Publisher & pub, const time_type &time)
    {
      auto m = time_message(time);
      pub.publish(m);
    }

    virtual send_callback_type push_callback(size_t flow) = 0;
    virtual send_callback_type push_callback(const string & name) = 0;
    
  private:
    ::ros::NodeHandle n;
    std::vector<::ros::Subscriber> subs;
    std::vector<::ros::Publisher> pubs;
    std::vector<std::unique_ptr<CountSubListener>> sub_listeners;
  };
  
}}
#endif
