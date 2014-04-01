#ifndef UTILS_H
#define UTILS_H

#define HBAR 1
#define NBAR 0.069614844704911

#include <stdint.h>
#include <sys/time.h>

uint64_t getTimeStamp();

#include <sstream>
#include <string>
#include <vector>

short reverseByteOrder(short v)
{
	v = ((v & 0xFF) << 8) | ((v & 0xFF00) >> 8);
	return v;
}

struct split
{
  enum empties_t { empties_ok, no_empties };
};

template <typename Container>
Container& split(
  Container&                                 result,
  const typename Container::value_type&      s,
  typename Container::value_type::value_type delimiter,
  split::empties_t                           empties = split::empties_ok )
{
  result.clear();
  std::istringstream ss( s );
  while (!ss.eof())
  {
    typename Container::value_type field;
    getline( ss, field, delimiter );
    if ((empties == split::no_empties) && field.empty()) continue;
    result.push_back( field );
  }
  return result;
}

#endif
