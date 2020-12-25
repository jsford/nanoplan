#ifndef NANOPLAN_SEARCH_SPACE_H
#define NANOPLAN_SEARCH_SPACE_H

#include <stdexcept>
#include <utility>
#include <vector>

namespace nanoplan {

template <typename STATE>
class SearchSpace {
 public:
  typedef STATE state_type;

  virtual ~SearchSpace() {}

  virtual std::vector<STATE> get_successors(const STATE& state) = 0;

  virtual std::vector<STATE> get_predecessors(const STATE& state) {
    throw std::logic_error(
        "NANOPLAN ERROR [NOT IMPLEMENTED]: "
        "State space does not implement required function "
        "get_predecessors.");
  }

  virtual double get_from_to_cost(const STATE& from, const STATE& to) = 0;

  virtual double get_from_to_heuristic(const STATE& from, const STATE& to) {
    throw std::logic_error(
        "NANOPLAN ERROR [NOT IMPLEMENTED]: "
        "State space does not implement required function "
        "get_from_to_heuristic.");
  }

  virtual std::vector<std::pair<STATE, STATE>> get_changed_edges() {
    throw std::logic_error(
        "NANOPLAN ERROR [NOT IMPLEMENTED]: "
        "State space does not implement required function "
        "get_changed_edges.");
  }
};

}  // namespace nanoplan

#endif  // NANOPLAN_SEARCH_SPACE_H
