#ifndef NANOPLAN_HASH_H
#define NANOPLAN_HASH_H

#include <functional>

inline void hash_combine(std::size_t& seed) { }

template <typename T, typename... Rest>
inline void hash_combine(std::size_t& seed, const T& v, Rest... rest) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    hash_combine(seed, rest...);
}

#define NANOPLAN_MAKE_STATE_HASHABLE(type, ...) \
namespace std {\
    template<> struct hash<type> {\
        std::size_t operator()(const type &s) const {\
            std::size_t ret = 0;\
            hash_combine(ret, __VA_ARGS__);\
            return ret;\
        }\
    };\
}

#endif // NANOPLAN_HASH_H
