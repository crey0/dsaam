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

template<class D> void to_double() {}

class OneBodySystem
{
public:

  OneBodySystem(const string &name, const std::vector<Body>& all_bodies)
  {
    for(auto b : all_bodies)
      {
	if (b.name != name) bodies.push_back(b);
	else _self = b;
      }
  }

  template<class D>
  void integrate(const D & dt, decltype(double(dt)) = 0)
  {
    _self.integrate(bodies, double(dt));
  }

  template<class D>
	   
  auto integrate(const D & dt) -> \
    typename std::enable_if<std::is_same<decltype(to_double(dt)), double>::value>::type
  {
    _self.integrate(bodies, to_double(dt));
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
  std::vector<Body> bodies;
};

template<class TTransport>
class OneBSystemNodeBase : public dsaam::OneThreadNode<TTransport>
{
public:
  using node_type = dsaam::OneThreadNode<TTransport>;
  using typename node_type::time_type;
  using typename node_type::send_callback_type;
  using typename node_type::message_callback_type;

  OneBSystemNodeBase(const std::vector<Body> & bodies, const string &name,
		     const time_type &time, const time_type &dt, size_t max_qsize,
		     const time_type &stop_time)
    : dsaam::OneThreadNode<TTransport>(name,time,dt,max_qsize),
    system(name, bodies), stop_time(stop_time)
  {
  }

  virtual void init()
  {
    //init send callbacks
    send_p = this->send_callback(this->name + "/position");
    send_v = this->send_callback(this->name + "/speed");

    //send init position and speed
    this->send_state(this->time());
  }

  void step(const time_type & to)
  {
    //std::cout << to_string("[",time(),"] [",name,"] Stepping to ",to) << std::endl;
    system.integrate(to - this->time());
    //send updated position and speed
    this->send_state(to);
    if(to + this->dt > stop_time) this->stop();

  }
 
protected:
  virtual void send_state(const time_type &t) = 0;

  OneBodySystem system;
  time_type stop_time;
  send_callback_type send_p;
  send_callback_type send_v;
};
