#ifndef NANOPLAN_HASH_MAP_H
#define NANOPLAN_HASH_MAP_H

#include "ext/flat_hash_map.hpp"
#include "utils.h"

namespace nanoplan {

template <class KEY, class VAL>
class HashMap {
 public:
  void put(const KEY& key, const VAL& val) { map[key] = val; }

  VAL at(const KEY& key) {
    if (map.find(key) == map.end()) {
      return std::numeric_limits<VAL>::infinity();
    }
    return map[key];
  }

  bool contains(const KEY& key) const { return map.find(key) != map.end(); }

  std::size_t size() const { return map.size(); }

 private:
  ska::flat_hash_map<KEY, VAL> map;
};

#endif  // NANOPLAN_HASH_H
