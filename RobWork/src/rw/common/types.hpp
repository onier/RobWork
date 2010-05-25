#ifdef _MSC_VER
#include <boost/cstdint.hpp>
typedef boost::int8_t int8_t;
typedef boost::uint8_t uint8_t;
typedef boost::int16_t int16_t;
typedef boost::uint16_t uint16_t;
typedef boost::int32_t int32_t;
typedef boost::uint32_t uint32_t;
typedef boost::int64_t int64_t;
typedef boost::uint64_t uint64_t;

#else
#include <stdint.h>
#endif
