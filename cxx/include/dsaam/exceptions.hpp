#ifndef DSAAM_EXCEPTIONS
#define DSAAM_EXCEPTIONS

#include<string>
#include<stdexcept>

#define logic_assert(test, error) if(!(test)) throw std::logic_error(error)

namespace dsaam
{
  //  typedef std::string string;

// inline void logic_except(bool test, const string & error)
//   {
//     if(!test)
//       throw std::logic_error(error);
//   }
}

#endif
