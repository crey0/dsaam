/***

Nbody test

 ***/
#include <dsaam/node.hpp>
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
  OneBodySystem(const string &name, const std::vector<Body>& all_bodies)
  {
    for(auto b : all_bodies)
      {
	if (b.name != name) bodies.push_back(b);
      }
  }
private:
  std::vector<Body> bodies;
};

int main()
{
  int n = 4;

  string names[] = {"red_0", "green_0", "blue_0", "yellow_0"};
  
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
  
  string colors[] = {"red", "green", "blue", "yellow"};

  
  std::vector<Body> all_bodies;
  for(int i=0; i<n; i++)
    {
      all_bodies.push_back(Body(names[i], positions[i],
				speeds[i], masses[i], radii[i], colors[i]));
    }
  return 0;
}
