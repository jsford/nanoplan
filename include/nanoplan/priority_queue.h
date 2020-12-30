#ifndef NANOPLAN_PRIORITY_QUEUE_H
#define NANOPLAN_PRIORITY_QUEUE_H

#include <algorithm>
#include <functional>
#include <queue>
#include <utility>
#include <vector>

#include "ext/flat_hash_map.hpp"
#include "hash.h"

namespace nanoplan {

template <class VALUE, class PRIORITY = double,
          class COMPARE = std::greater<PRIORITY>>
class PriorityQueue {
 public:
  VALUE top() { return vec.at(0).first; }
  PRIORITY top_priority() { return vec.at(0).second; }

  void pop() {
    set.erase(top());
    std::pop_heap(vec.begin(), vec.end(), comparator);
    vec.pop_back();
  }

  void remove(const VALUE& v) {
    if (!contains(v)) {
      return;
    }

    int i = 0;
    for (; i < vec.size(); ++i) {
      if (vec[i].first == v) {
        break;
      }
    }
    if (i < vec.size()) {
      std::swap(vec[0], vec[i]);
      pop();
    }
  }

  void insert(const VALUE& v, const PRIORITY& p) {
    vec.push_back(std::make_pair(v, p));
    std::push_heap(vec.begin(), vec.end(), comparator);
    set.insert(v);
  }

  bool contains(const VALUE& v) { return (set.find(v) != set.end()); }

  bool empty() const { return vec.size() == 0; }

 private:
  ska::flat_hash_set<VALUE> set;
  std::vector<std::pair<VALUE, PRIORITY>> vec;

  // This wrapper is necessary to compare
  // std::pair<VALUE,PRIORITY> using the priority comparator.
  struct COMPARATOR {
    bool operator()(const std::pair<VALUE, PRIORITY>& a,
                    const std::pair<VALUE, PRIORITY>& b) {
      return comp(a.second, b.second);
    }
    COMPARE comp;
  } comparator;
};

}  // namespace nanoplan

#endif  // NANOPLAN_PRIORITY_QUEUE_H
