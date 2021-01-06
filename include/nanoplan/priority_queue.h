#ifndef NANOPLAN_PRIORITY_QUEUE_H
#define NANOPLAN_PRIORITY_QUEUE_H

#include <unistd.h>  // remove this

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
  PriorityQueueWithRemove() : vec(1) {}

  VALUE top() { return vec[1].value; }
  PRIORITY top_priority() { return vec[1].priority; }

  void pop() { remove(top()); }

  void remove(const VALUE& v) {
    // Quit if you can't find the value to remove.
    auto it = idx.find(v);
    if (it == idx.end()) {
      return;
    }
    // Get the index of the value to remove.
    const auto i = it->second;
    // Overwrite it with the last value in the queue.
    vec[i] = std::move(vec[vec.size() - 1]);
    // Fix up the index for the newly placed value.
    idx[vec[i].value] = i;
    // Remove all traces of the old value.
    vec.pop_back();
    idx.erase(v);
    // Fix the heap.
    heap_down(i);
  }

  void insert(const VALUE& v, const PRIORITY& p) {
    remove(v);
    vec.emplace_back(v, p);
    idx[v] = vec.size() - 1;
    heap_up(vec.size() - 1);
  }

  bool contains(const VALUE& v) { return idx.find(v) != idx.end(); }

  bool empty() const { return vec.size() == 1; }
  std::size_t size() const { return vec.size() - 1; }

  void print() const {
    for (int i = 1; i < vec.size(); ++i) {
      if (!((i != 1) && (i & (i - 1)))) {
        fmt::print("------------------\n");
      }
      auto s = vec[i].value;
      auto p = vec[i].priority;
      fmt::print("{}, {} <{}, {}>\n", s.x, s.y, p.first, p.second);
    }
  }

  bool is_heap(unsigned long i = 1) {
    if (right(i) >= vec.size()) {
      return true;
    }
    return vec[i].priority <= vec[right(i)].priority &&
           vec[i].priority <= vec[left(i)].priority && is_heap(right(i)) &&
           is_heap(left(i));
  }

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
    while (i > 1 && vec[i] < vec[parent(i)]) {
      swap(vec[i], vec[parent(i)]);
      i = parent(i);
    }
  }

  void heap_down(unsigned long i) {
    const auto elmt = vec[i];

    while (true) {
      const auto l = left(i);
      const auto r = right(i);

      // Quit if this node is a leaf.
      if (l > vec.size()) {
        break;
      }

      // Find the smallest child node.
      unsigned long child;
      if (l == vec.size() || vec[l] < vec[r]) {
        child = l;
      } else {
        child = r;
      }

      if (vec[i] < vec[child]) {
        break;  // Quit because the smallest child is not smaller.
      } else {
        // Move the i-th element down and the child up.
        // NOTE(Jordan): This swap can be optimized to a bubble-down,
        // but the idx bookkeeping is a little tricky.
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
