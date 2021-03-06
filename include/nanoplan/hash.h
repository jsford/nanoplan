#ifndef NANOPLAN_HASH_H
#define NANOPLAN_HASH_H

#include <cstdint>
#include <functional>
#include <limits>

#include "utils.h"

// std::rotl is not defined before C++20.
#if __cplusplus <= 201703L
namespace std {
template <typename T, typename S>
typename std::enable_if<std::is_unsigned<T>::value, T>::type const rotl(
    const T n, const S i) {
  const T m = (std::numeric_limits<T>::digits - 1);
  const T c = i & m;
  return (n << c) |
         (n >> ((T(0) - c) &
                m));  // this is usually recognized by the compiler to mean
                      // rotation, also c++20 now gives us rotl directly
}
}  // namespace std
#else
#include <bit>
#endif

namespace nanoplan {

template <typename T>
inline T xorshift(const T& n, int i) {
  return n ^ (n >> i);
}

inline uint32_t distribute(const uint32_t& n) {
  uint32_t p = 0x55555555ul;  // pattern of alternating 0 and 1
  uint32_t c = 3423571495ul;  // random uneven integer constant
  return c * xorshift(p * xorshift(n, 16), 16);
}

inline uint64_t hash(const uint64_t& n) {
  uint64_t p = 0x5555555555555555;       // pattern of alternating 0 and 1
  uint64_t c = 17316035218449499591ull;  // random uneven integer constant
  return c * xorshift(p * xorshift(n, 32), 32);
}

inline void hash_combine(std::size_t& seed) {}

template <typename T, typename... Rest>
inline void hash_combine(std::size_t& seed, const T& v, Rest... rest) {
  seed = std::rotl(seed, std::numeric_limits<std::size_t>::digits / 3) ^
         distribute(std::hash<T>{}(v));
  hash_combine(seed, rest...);
}

}  // namespace nanoplan

#define NANOPLAN_MAKE_STATE_HASHABLE(type, ...)   \
  namespace std {                                 \
  template <>                                     \
  struct hash<type> {                             \
    std::size_t operator()(const type& s) const { \
      std::size_t ret = 0;                        \
      nanoplan::hash_combine(ret, __VA_ARGS__);   \
      return ret;                                 \
    }                                             \
  };                                              \
  }

#endif  // NANOPLAN_HASH_H
