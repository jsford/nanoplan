#ifndef SPACE2D_H_
#define SPACE2D_H_

#include <nanoplan/nanoplan.h>

#include <vector>

struct State2D {
  int x;
  int y;

  bool operator==(const State2D& rhs) const { return x == rhs.x && y == rhs.y; }
};
NANOPLAN_MAKE_STATE_HASHABLE(State2D, s.x, s.y);

class SearchSpace2D final : public nanoplan::SearchSpace<State2D> {
 public:
  static const int w = 4096;
  static const int h = 4096;
  bool use_new_costs = false;
  double new_cost_mult = nanoplan::INF_DBL;

  std::vector<State2D> get_successors(const State2D& state) override {
    std::vector<State2D> succs;
    succs.reserve(8);
    if (state.y + 1 < h) {
      succs.push_back({state.x, state.y + 1});
    }
    if (state.x + 1 < w) {
      succs.push_back({state.x + 1, state.y});
    }
    if (state.x - 1 >= 0) {
      succs.push_back({state.x - 1, state.y});
    }
    if (state.y - 1 >= 0) {
      succs.push_back({state.x, state.y - 1});
    }

    if (state.x + 1 < w && state.y + 1 < h) {
      succs.push_back({state.x + 1, state.y + 1});
    }
    if (state.x - 1 >= 0 && state.y + 1 < h) {
      succs.push_back({state.x - 1, state.y + 1});
    }
    if (state.x - 1 >= 0 && state.y - 1 >= 0) {
      succs.push_back({state.x - 1, state.y - 1});
    }
    if (state.x + 1 < w && state.y - 1 >= 0) {
      succs.push_back({state.x + 1, state.y - 1});
    }

    return succs;
  }

  std::vector<State2D> get_predecessors(const State2D& state) override {
    return get_successors(state);
  }

  double get_from_to_cost(const State2D& from, const State2D& to) override {
    if (use_new_costs && to.x == 4000 && to.y == 4000) {
      return new_cost_mult * euclidean(from, to);
    }
    return euclidean(from, to);
  }

  double get_from_to_heuristic(const State2D& from,
                               const State2D& to) override {
    return euclidean(from, to);
  }

  double euclidean(const State2D& from, const State2D& to) const {
    return std::sqrt((to.x - from.x) * (to.x - from.x) +
                     (to.y - from.y) * (to.y - from.y));
  }

  std::vector<State2D> get_changed_states() override {
    std::vector<State2D> states;
    states.push_back(State2D{4000, 4000});
    return states;
  }
};

#endif  // SPACE2D_H_
