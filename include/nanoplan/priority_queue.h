#ifndef NANOPLAN_PRIORITY_QUEUE_H
#define NANOPLAN_PRIORITY_QUEUE_H

#include <fmt/format.h>  // remove this
#include <unistd.h>      // remove this

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

  void pop() { remove(top()); }

  void insert(const VALUE& v, const PRIORITY& p) {
    // fmt::print("PQ INSERT VALUE {},{} : PRIORITY {}, {}\n", v.x, v.y,
    // p.first.as_double(), p.second.as_double());
    vec.push_back(HeapEntry{v, p});
    std::push_heap(vec.begin(), vec.end());
    // print();
  }

  void remove(const VALUE& v) {
    // fmt::print("PQ REMOVE VALUE {},{}\n", v.x, v.y);
    // TRY NOT TO USE THIS. IT IS SLOW AS HELL.
    int i = 0;
    for (; i < vec.size(); ++i) {
      if (vec[i].value == v) {
        break;
      }
    }
    if (i == vec.size()) {
      // print();
      return;
    }
    vec.erase(vec.begin() + i);
    std::make_heap(vec.begin(), vec.end());
    // print();
  }

  bool empty() const { return vec.size() == 0; }
  std::size_t size() const { return vec.size(); }

  void print() const {
    for (const auto& x : vec) {
      fmt::print("{},{} : {},{}\n", x.value.x, x.value.y,
                 x.priority.first.as_double(), x.priority.second.as_double());
    }
    fmt::print("\n");
  }

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

  void pop() { remove(top()); }

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

  void print() {
    fmt::print("------PQ--------\n");
    for (int i = 1; i < vec.size(); ++i) {
      const auto& x = vec[i];
      fmt::print("{},{} : {},{} idx={}\n", x.value.x, x.value.y,
                 x.priority.first.as_double(), x.priority.second.as_double(),
                 idx[x.value]);
    }
    for (const auto& p : idx) {
      fmt::print("{} -> {},{}\n", p.second, p.first.x, p.first.y);
    }
    fmt::print("\n");
  }

  bool is_good() const {
    for (int i = 2; i < vec.size(); ++i) {
      if (vec[i].priority < vec[1].priority) {
        auto vi = vec[i].priority;
        auto v1 = vec[1].priority;
        fmt::print("GET REKT: {},{} < {},{}\n", vi.first.as_double(),
                   vi.second.as_double(), v1.first.as_double(),
                   v1.second.as_double());

        return false;
      }
    }
    return true;
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
