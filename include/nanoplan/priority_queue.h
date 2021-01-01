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

template <class VALUE, class PRIORITY>
class PriorityQueue {
  struct HeapEntry;

 public:
  VALUE top() { return vec.at(0).value; }
  PRIORITY top_priority() { return vec.at(0).priority; }

  void pop() {
    std::pop_heap(vec.begin(), vec.end());
    vec.pop_back();
  }

  void insert(const VALUE& v, const PRIORITY& p) {
    vec.push_back(HeapEntry{v, p});
    std::push_heap(vec.begin(), vec.end());
  }

  bool empty() const { return vec.size() == 0; }

 public:
  std::vector<HeapEntry> vec;

 private:
  struct HeapEntry {
    VALUE value;
    PRIORITY priority;

    bool operator<(const HeapEntry& rhs) const {
      // std::push_heap, std::pop_heap, etc.
      // construct a max. heap. I need a min. heap,
      // so this inequality is backwards from what you might expect.
      return priority > rhs.priority;
    }
  };
};

template <class VALUE, class PRIORITY = double,
          class COMPARE = std::less<PRIORITY>>
class PriorityQueueWithRemove {
  struct HeapEntry;

 public:
  VALUE top() { return vec.at(0).value; }
  PRIORITY top_priority() { return vec.at(0).priority; }

  void pop() { remove(top()); }

  void remove(const VALUE& v) {
    if (!contains(v)) {
      return;
    }
    auto i = idx[v];
    swap(vec[i], vec[vec.size() - 1]);
    vec.pop_back();
    idx.erase(v);
    heap_down(i);
  }

  void insert(const VALUE& v, const PRIORITY& p) {
    if (contains(v)) {
      remove(v);
    }
    vec.push_back(HeapEntry{v, p});
    idx[v] = vec.size() - 1;
    heap_up(vec.size() - 1);
  }

  bool contains(const VALUE& v) { return idx.find(v) != idx.end(); }

  bool empty() const { return vec.size() == 0; }

 private:
  struct HeapEntry {
    VALUE value;
    PRIORITY priority;

    bool operator<(const HeapEntry& rhs) {
      return priority < rhs.priority;
    }
    bool operator==(const HeapEntry& rhs) {
      return priority == rhs.priority;
    }
    bool operator>(const HeapEntry& rhs) {
      return !operator<(rhs) && !operator==(rhs);
    }
  };

  void swap(HeapEntry& a, HeapEntry& b) {
    std::swap(idx[a.value], idx[b.value]);
    std::swap(a, b);
  }

  void heap_up(unsigned long i) {
    while (parent(i) >= 0 && vec[i] < vec[parent(i)]) {
      swap(vec[i], vec[parent(i)]);
      i = parent(i);
    }
  }

  void heap_down(unsigned long i) {
    while (right(i) < vec.size() &&
           (vec[i] > vec[left(i)] || vec[i] > vec[right(i)])) {
      if (vec[left(i)] < vec[right(i)]) {
        swap(vec[i], vec[left(i)]);
        i = left(i);
      } else {
        swap(vec[i], vec[right(i)]);
        i = right(i);
      }
    }
  }

  static unsigned long parent(unsigned long i) {
    return std::floor((i - 1) / 2);
  }
  static unsigned long left(unsigned long i) {
    return 2 * i + 1;
  }
  static unsigned long right(unsigned long i) {
    return 2 * i + 2;
  }

 public:
  ska::flat_hash_map<VALUE, unsigned long> idx;
  std::vector<HeapEntry> vec;
};

}  // namespace nanoplan

#endif  // NANOPLAN_PRIORITY_QUEUE_H
