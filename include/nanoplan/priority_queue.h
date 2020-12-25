#ifndef NANOPLAN_PRIORITY_QUEUE_H
#define NANOPLAN_PRIORITY_QUEUE_H

#include <queue>
#include <utility>
#include <vector>

#include "hash.h"

namespace nanoplan {

template <typename STATE>
using PqEntry = std::pair<STATE, double>;

template <typename PQ_ENTRY>
struct PqCompare {
  bool operator()(const PQ_ENTRY& a, const PQ_ENTRY& b) const {
    return a.second > b.second;
  }
};

template <typename STATE>
using PriorityQueue =
    std::priority_queue<PqEntry<STATE>, std::vector<PqEntry<STATE>>,
                        decltype(PqCompare<PqEntry<STATE>>())>;

}  // namespace nanoplan

#endif  // NANOPLAN_PRIORITY_QUEUE_H
