#ifndef DSAAM_ROS_NODE_HPP
#define DSAAM_ROS_NODE_HPP

#include <dsaam/node.hpp>

#include<ros/ros.h>
#include<std_msgs/Header.h>

/*TODO:
  P0: IDEA: Change library template structure to Node<TRANSPORT_TYPE> with
  transport_type::message_pointer, transport_type::time, transport_type::get_time...
  Then we have Node : public TransportType.
  Is this feasible ???
  P1 : Subscriber callbacks need to be passed with bare method pointers + object
  P2 : SubscriberStatusCallback is a typedef for boost::function !
  P3 : Check if it will give us boost::shared_pointer or std::shared_pointer (unclear)
  when using subscribe ???
  P4 : Finish implementing all the init

*/
namespace dsaam::ros
{

  template<class P1, class P2, class... Args>
  struct message_conversion_callback
  {
    std::function<void(P2, Args...)> callback_type;
    
    message_conversion_callback(callback_type & callback) : callback(callback) {}

    void operator()(P1 &p1, Args... args)
    {
      P2 p2 = std::static_pointer_cast<P2>(p1);
      callback(p2, &args...);
    }
    
    callback_type callback;
  };

	
  class CountSubListener
  {
  public:
    RosSubscriptionWaiter(size_t num_peers) : num_peers(num_peers), sub_sem(0) {}

    void peer_subscribe(ros::SingleSubscriberPublisher & sub)
    {
      sub_sem.release();
    }

    auto peer_subscribe_callback()
    {
      return std::bind(&CountSubListener::peer_subscribe, this);
    }

    void peer_unsubscribe(ros::SingleSubscriberPublisher & sub)
    {
      throw std::runtime_error(to_string("No unsubscription allowed for topic ",sub.getTopic(),
					 " requested by ",sub.getSubscriberName(),"\n")),
	}

    auto peer_unsubscribe_callback()
    {
      return std::bind(&CountSubListener::peer_unsubscribe, this);
    }

    void wait()
    {
      for(int i=0; i< num_peers; i++)
	{
	  sub_sem.acquire();
	}
    }
    
  private:
    size_t num_peers;
    Semaphore sub_sem;
    
  };
  
  struct ros_header_stamp
  {
    using time_message_type=std_msgs::Header;
    using mpointer=boost::shared_ptr<time_message_type>;
    
    static const ros::Time& time(const mpointer &m)
    {
      return m->header.stamp;
    }

    static std_msgs::Header time_message(const ros::Time &t)
    {
      std_msgs::Header h;
      h.stamp = t;
      return h;
    }

    static const ros::Time & time_message(const std::shared_pointer<const std_msgs::Header> &m)
    {
      return m->header.stamp;
    }
  };
  
  template<class M = std_msgs::Header, class T = ros::Time, class FMT = ros_header_stamp>
  class RosNode : public FMT
  {
  public:

    using message_type=M;
    using time_type=T;
    using extract_message_time=FMT;
    
    RosNode(T &time, T &dt, std::vector<InFlow> &inflows,
	    std::vector<OutFlow> &outflows, unsigned int max_qsize)
      : Node(time, dt, inflows, outflows, max_qsize)
    {
      
      size_t sub_count = inflows.size(); 
      
      //Setup outflows
      for(auto &outf : outflows)
	{
	  sub_count += outf.sinks.size()
	    for(auto &s : outf.sinks)
	      {
		auto cb = std::bind<void(const &T)>(&RosNode<F, M, FMT>::_ros_receive_time, this,
						    time_callback(outf.name, s.name), _1);
		subs.push_back(n.subscribe(this->name + "/time/" + sname, max_qsize, cb));
	      }
	}

      sub_listener = std::unique_ptr(new CountSubListener(sub_count));

      
    }

    template<class S, class = std::enable_if<std::is_convertible<S, M>::value > = 0>
    void set_inflow_callback(const string & ifname,
			     const dsaam::message_callback_type<S, T> &m_cb)
    {
      //Setup ROS subscriber
      auto cb = push_callback(ifname);
      subs.push_back(n.subscribe<S>(inf.name, max_qsize, cb));

      //Setup callback for delivering messages
      auto message_cwrapper = message_conversion_callback<S>(m_cb);
      auto time_callback = _ros_out_time_callback(ifname);
      dsaam::Node<M, T, FMT>::set_inflow_callbacks(ifname, message_cwrapper, t_cb);
    }

    template<class S, class = std::enable_if<std::is_convertible<S, M>::value > = 0>
    void set_outflow_callback(const string &ifname)
    {
      pubs.push_back(n.advertise<FMT::time_message_type>(ifname + "/time/" + this->name, max_qsize,
							 sub_listener->peer_subscribe_callback()));
    }
    
    
  private:

    auto _ros_out_time_callback(const string &ifname)
    {
      pubs.push_back(n.advertise<FMT::time_message_type>(ifname + "/time/" + this->name, max_qsize,
							 sub_listener->peer_subscribe_callback()));
      auto &pub = pubs.back();
      using std::placeholders::_1;
      return std::bind<void(const &T)>(&RosNode<F, M, FMT>::_ros_publish_time, this, pub, _1);
    }

    void _ros_receive_time(const time_callback_type& cb, const FMT::time_message_type &time)
    {
      cb(FMT::time_message(time));
    }
    
    void _ros_publish_time(ros::Publiser & pub, const T &time)
    {
      auto m = FMT::time_message(t);
      pub.publish(m);
    }


  private:
    ros::NodeHandle n;
    std::vector<ros::Subscriber> subs;
    std::vector<ros::Publisher> pubs;
    std::unique_ptr<CountSubListener> sub_listeners;
  };

  template<class M = std_msgs::Header, class T = ros::Time, class FMT = ros_header_stamp, class... Args>
  RosNode<M, T, FMT> make_rosnode_from_parameters(string name)
  {
    //TODO read from parameter server
    T time = TODO;
    T dt = TODO;
    unsigned int max_qsize = TODO;
    std::vector<InFlow> inflows(TODO);
    std::vector<OutFlow> outflows(TODO);
  }
  
}

#endif
