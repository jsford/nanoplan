#ifndef NANOPLAN_PRIORITY_QUEUE_H
#define NANOPLAN_PRIORITY_QUEUE_H

#include <algorithm>
#include <cassert>
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
  VALUE top() { return vec[0].value; }
  PRIORITY top_priority() { return vec[0].priority; }

  void pop() {
    std::pop_heap(vec.begin(), vec.end());
    vec.pop_back();
  }

  void insert(const VALUE& v, const PRIORITY& p) {
    vec.push_back(HeapEntry{v, p});
    std::push_heap(vec.begin(), vec.end());
  }

  void remove(const VALUE& v) {
    // TRY NOT TO USE THIS. IT IS SLOW AS HELL.
    int i = 0;
    for (; i < vec.size(); ++i) {
      if (vec[i].value == v) {
        break;
      }
    }
    if (i == vec.size()) {
      return;
    }
    vec.erase(vec.begin() + i);
    std::make_heap(vec.begin(), vec.end());
  }

  bool empty() const { return vec.size() == 0; }
  std::size_t size() const { return vec.size(); }

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

template <class VALUE, class PRIORITY, class COMPARE = std::less<PRIORITY>>
class PriorityQueueWithRemove {
  struct HeapEntry;

 public:
  PriorityQueueWithRemove() : vec(1) {}

  VALUE top() { return vec.at(1).value; }
  PRIORITY top_priority() { return vec.at(1).priority; }

  void pop() {
    std::pop_heap(vec.begin(), vec.end());
    vec.pop_back();
  }

  void remove(const VALUE& v) {
    // Quit if you can't find the value to remove.
    auto it = idx.find(v);
    if (it == idx.end()) {
      return;
    }

    // Get the index of the value to remove.
    const std::size_t i = idx[v];

    // We need this later to decide whether to heap up or down.
    auto removed_priority = vec[i].priority;

    // Overwrite it with the last value in the queue.
    auto last_idx = vec.size() - 1;
    auto last_elem = vec[last_idx];
    vec[i] = last_elem;
    idx[last_elem.value] = i;
    idx.erase(v);
    vec.pop_back();

    if (removed_priority < last_elem.priority) {
      heap_down(i);
    } else {
      heap_up(i);
    }
    assert(idx.size() + 1 == vec.size());
  }

  void insert(const VALUE& v, const PRIORITY& p) {
    remove(v);
    vec.push_back(HeapEntry{v, p});
    idx[v] = vec.size() - 1;
    heap_up(vec.size() - 1);
    assert(idx.size() + 1 == vec.size());
  }

  bool contains(const VALUE& v) { return idx.find(v) != idx.end(); }

  bool empty() const { return vec.size() == 1; }
  std::size_t size() const { return vec.size() - 1; }

 private:
  struct HeapEntry {
    VALUE value;
    PRIORITY priority;

    bool operator<(const HeapEntry& rhs) { return priority < rhs.priority; }
    bool operator==(const HeapEntry& rhs) { return priority == rhs.priority; }
    bool operator>(const HeapEntry& rhs) {
      return !operator<(rhs) && !operator==(rhs);
    }
    bool operator>=(const HeapEntry& rhs) {
      return operator>(rhs) || operator==(rhs);
    }
    bool operator<=(const HeapEntry& rhs) {
      return operator<(rhs) || operator==(rhs);
    }
  };

  inline void swap(HeapEntry& a, HeapEntry& b) {
    std::swap(idx[a.value], idx[b.value]);
    std::swap(a, b);
  }

  void heap_up(unsigned long i) {
    while (parent(i) >= 1 && vec[i] < vec[parent(i)]) {
      swap(vec[i], vec[parent(i)]);
      i = parent(i);
    }
  }

  void heap_down(unsigned long i) {
    while (true) {
      const auto l = left(i);
      const auto r = right(i);

      // Quit if this node is a leaf.
      if (l >= vec.size()) {
        break;
      }

      // Find the smallest child node.
      unsigned long child;
      if (l == vec.size() - 1 || vec[l] <= vec[r]) {
        child = l;
      } else {
        child = r;
      }

      if (vec[i] <= vec[child]) {
        break;  // Quit because the smallest child is not smaller.
      } else {
        swap(vec[i], vec[child]);
        i = child;
      }
    }
  }
  inline static unsigned long parent(unsigned long i) { return i / 2; }
  inline static unsigned long left(unsigned long i) { return 2 * i; }
  inline static unsigned long right(unsigned long i) { return 2 * i + 1; }

 public:
  ska::flat_hash_map<VALUE, unsigned long> idx;
  std::vector<HeapEntry> vec;
};

}  // namespace nanoplan

#endif  // NANOPLAN_PRIORITY_QUEUE_H
