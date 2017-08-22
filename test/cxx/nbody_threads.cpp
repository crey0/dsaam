/***

Nbody test

 ***/
#include <dsaam/onethreadnode.hpp>
#include <dsaam/string_utils.hpp>
#include <string>
#include <vector>
#include <cmath>
#include <numeric>

#define EPS 1e-3
#define G 1.5 * 1.80 * 1e-11
#define C 8 * 1e-3

typedef std::string string;
typedef std::array<double, 2>  Array2d;

Array2d operator+(const Array2d &l, const Array2d &r) { return {{l[0]+r[0], l[1]+r[1]}};}
template<class T>
Array2d operator+(const Array2d &l, const T &r) { return {{l[0]+r, l[1]+r}};}
template<class T>
Array2d operator+(const T &l , const Array2d &r) { return r + l;}

Array2d operator-(const Array2d &r) { return {{-r[0], -r[1]}};}
Array2d operator-(const Array2d &l, const Array2d &r) { return l + (-r);}
template<class T>
Array2d operator-(const Array2d &l, const T &r) { return l + (-r);}
template<class T>
Array2d operator-(const T &l , const Array2d &r) { return (-r) + l;}

Array2d operator*(const Array2d &l , const Array2d &r) { return {{l[0]*r[0], l[1]*r[1]}};}
template<class T>
Array2d operator*(const Array2d &l, const T &r) { return {{l[0]*r, l[1]*r}};}
template<class T>
Array2d operator*(const T &l , const Array2d &r) { return r * l;}

template<class T>
Array2d operator%(const Array2d &l, const T &r) { return {{fmod(l[0],r), fmod(l[1],r)}};}

template<class T>
Array2d pow(const Array2d &l, const T &r) { return {{pow(l[0],r), pow(l[1],r)}};}

Array2d sign(Array2d & a) { return {{copysign(1., a[0]), copysign(1., a[1])}}; }
Array2d abs(Array2d & a) { return {{fabs(a[0]), fabs(a[1])}}; }

template<class T>
typename T::value_type
sum(const T& a)
{return std::accumulate<typename T::const_iterator, typename T::value_type>(a.begin(), a.end(), 0.);}

template<class T>
typename T::value_type
norm(const T& a, int b)
{return pow(sum(pow(a,b)), 1.0/b);}

typedef struct Body
{
  Body() = default;
  
  Body(string name, Array2d & p0, Array2d & v0, double mass, double radius, string color)
    : name(name), p(p0), v(v0), mass(mass), radius(radius), color(color)
  {}

  Array2d  acceleration(std::vector<Body> & others)
  {
    Array2d a = {{0.0, 0.0}};
    for(Body & o : others)
      {
	Array2d d = p - o.p;
	Array2d d1 = -1 * sign(d) * (1 - abs(d));
	for(size_t i=0; i<d.size(); i++){ d[i] = fabs(d[i]) < fabs(d1[i]) ? d[i] : d1[i]; }
	double denom = sum(pow(abs(d),3) + EPS);
	a = a + o.mass/denom * d;
      }
    return G * a;
  }
  
  void integrate(std::vector<Body> & others, double dt)
  {
    auto a = acceleration(others);
    p = (p + v*dt) % 1;
    v = v + a * dt;
    double n = norm(v,2);
    if (n > C) v = v * (1/n * C);
  }
  
  string name;
  Array2d  p;
  Array2d  v;
  double mass;
  double radius;
  string color;

}Body;


class OneBodySystem
{
public:

  class PMessage : public dsaam::MessageBase
  {
  public:
    PMessage(const Array2d & p, const dsaam::Time & t)
      : MessageBase(t), p(p) {}
  public:
    Array2d p;
  };

  class VMessage : public dsaam::MessageBase
  {
  public:
    VMessage(const Array2d & v, const dsaam::Time & t)
      : MessageBase(t), _filler_("filler"), v(v) {}
  public:
    string _filler_;
    Array2d v;
  };


  OneBodySystem(const string &name, const std::vector<Body>& all_bodies, const dsaam::Time & dt)
    : dt(dt)
  {
    for(auto b : all_bodies)
      {
	if (b.name != name) bodies.push_back(b);
	else _self = b;
      }
  }

  void pCallback(Body &b, const dsaam::Node::mpointer & pm, const dsaam::Time &)
  {
    auto pmp = static_cast<const PMessage *>(pm.get());
    //std::cout << dsaam::to_string("[", pmp->time, "]", "[",_self.name,"] pos update of ", b.name,
    //				  " next at ",nextAt)
    //	      << std::endl;
    b.p = pmp->p;
  }
  
  void vCallback(Body & b, const dsaam::Node::mpointer & pm,  const dsaam::Time &)
  {
    auto pmv = static_cast<const VMessage *>(pm.get());
    //std::cout << dsaam::to_string("[", pmv->time, "]", "[",_self.name,"] spe update of ", b.name,
    //				  " next at ",nextAt)
    //	      << std::endl;
    b.v = pmv->v;
  }

  void integrate(const dsaam::Time & dt)
  {
    _self.integrate(bodies, double(dt));
  }

  const Body & self()
  {
    return _self;
  }

  Body& get_body(const string &name)
  {
    for(auto &b : bodies) if (b.name == name) return b;
    throw std::domain_error(name);
  }
  
private:
  Body _self;
  const dsaam::Time dt; 
  std::vector<Body> bodies;
};

class OneBSystemNode : public dsaam::OneThreadNode
{
public:
  OneBSystemNode(std::vector<Body> & bodies, string &name, dsaam::Time &time, dsaam::Time &dt,
		 std::vector<dsaam::InFlow> &inflows, std::vector<dsaam::OutFlow> &outflows,
		 unsigned int max_qsize,
		 const dsaam::Time &stop_time)
    : OneThreadNode(name,time,dt,inflows,outflows,max_qsize), system(name, bodies, time),
      stop_time(stop_time)
  {
    send_p = send_callback(name + "/position");
    send_v = send_callback(name + "/speed");
  }

  virtual void init()
  {
    //send init position and speed
    send_state(time());
  }

  virtual void step(const dsaam::Time & to)
  {
    //std::cout << to_string("[",time(),"] [",name,"] Stepping to ",to) << std::endl;
    system.integrate(to - time());
    //send updated position and speed
    send_state(to);
    if(to + dt() > stop_time) stop();

  }

  dsaam::message_callback_type pCallback(const string &b_name)
  {
    Body & b = system.get_body(b_name);
    return std::bind(&OneBodySystem::pCallback, &system, b,
		     std::placeholders::_1, std::placeholders::_2);
  }

   dsaam::message_callback_type vCallback(const string &b_name)
  {
    Body & b = system.get_body(b_name);
    return std::bind(&OneBodySystem::vCallback, &system, b,
		     std::placeholders::_1, std::placeholders::_2);
  }
  
private:
  void send_state(dsaam::Time t)
  {
    dsaam::Node::mpointer m = \
      dsaam::Node::mpointer(new OneBodySystem::PMessage(system.self().p, t));
    send_p(std::move(m));
     
    
    m =  dsaam::Node::mpointer(new OneBodySystem::VMessage(system.self().v, t));
    send_v(std::move(m));
  }

  OneBodySystem system;
  dsaam::Time stop_time;
  dsaam::send_callback_type send_p;
  dsaam::send_callback_type send_v;
};

int main()
{
  string colors[] = {"red", "green", "blue", "yellow"};

  unsigned int n = 4;
  string names[] = {"red_0", "green_0", "blue_0", "yellow_0"};
  
  std::vector<string> ins[] = {{"yellow_0"},
			       {"red_0"},
			       {"green_0"},
			       {"red_0", "green_0", "blue_0"}};
  
  std::vector<string> outs[] = {{"green_0", "yellow_0"},
				{"blue_0", "yellow_0"},
				{"yellow_0"},
				{"red_0"}};
  
  
  Array2d positions[] = {{{0.5,  0.5}},
		       {{0.3,  0.3}},
		       {{0.2,  0.2}},
		       {{0.15, 0.15}}};
  
  Array2d speeds[] =  {{{0.0,   0.0}},
		     {{-0.1,  0.1}},
		     {{-0.1,  0.1}},
		     {{-0.1,  0.1}}};

  double masses[] = {1000., 100., 10., 1.};
  
  double radii[] = {100., 10., 1., 0.1};
  

  dsaam::Time start_time(0);
  dsaam::Time dts[] = {{1}, {2}, {3}, {4}};

  unsigned int max_qsize = 10;
  
  //construct all bodies
  std::vector<Body> all_bodies;
  for(size_t i=0; i<n; i++)
    {
      all_bodies.push_back(Body(names[i], positions[i],
				speeds[i], masses[i], radii[i], colors[i]));
    }

  auto get_node_idx = [&](string name) -> size_t
    {for(size_t i=0; i<n; i++) if (names[i] == name) return i;
     throw std::domain_error(name);}; 
  
  //construct all Nodes
  std::deque<OneBSystemNode> nodes;
  for(size_t i=0; i<n; i++)
    {
      auto &b = all_bodies[i];
      
      std::vector<dsaam::InFlow> inflows;
      for(size_t j=0; j < ins[i].size(); j++)
	{
	  string j_name = ins[i][j];
	  size_t jdx = get_node_idx(j_name);
	  inflows.emplace_back(j_name+"/position", dts[jdx]);
	  inflows.emplace_back(j_name+"/speed", dts[jdx]);
	}
      
      std::vector<dsaam::Sink> sinks;
      for(size_t j=0; j < outs[i].size(); j++)
	{
	  string b_name = outs[i][j];
	  sinks.emplace_back(b_name);
	}
      
      std::vector<dsaam::OutFlow> outflows = {{b.name + "/position", dts[i], sinks},
					      {b.name + "/speed", dts[i], sinks}};
      nodes.emplace_back(all_bodies,
			 names[i], start_time, dts[i],
			 inflows, outflows,
			 max_qsize, dsaam::Time(10000));
    }

  auto get_node = [&](string name) -> OneBSystemNode&
    {for(auto & n : nodes) if (n.name == name) return n;
     throw std::domain_error(name);}; 
	  

  //link Nodes together
   for(size_t i=0; i<n; i++)
    {
      auto &n = nodes[i];
      auto bp = n.name + "/position";
      auto bs = n.name + "/speed";
      for(size_t j=0; j < ins[i].size(); j++)
	{
	  string j_name = ins[i][j];
	  auto & nj = get_node(j_name);
	  auto jp = j_name + "/position";
	  auto js = j_name + "/speed";
	  n.set_inflow_callbacks(jp, n.pCallback(j_name), nj.time_callback(jp, n.name));
	  n.set_inflow_callbacks(js, n.vCallback(j_name), nj.time_callback(js, n.name));

	}
      std::vector<dsaam::Sink> sinks;
      for(size_t j=0; j < outs[i].size(); j++)
	{
	  string j_name = outs[i][j];
	  auto & nj = get_node(j_name);
	  n.set_outflow_callback(bp, j_name, nj.push_callback(bp));
	  n.set_outflow_callback(bs, j_name, nj.push_callback(bs));
	}
    }

  //start Nodes
  for(auto &n : nodes) n.start();

  //wait for Nodes to stop
  for(auto &n : nodes) n.join();

  //stop
  return 0;
}
