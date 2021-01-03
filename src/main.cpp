#include <fmt/format.h>

#include <climits>
#include <cmath>
#include <vector>

#include "nanoplan/nanoplan.h"

struct State2D {
  int x;
  int y;

  bool operator==(const State2D& rhs) const { return x == rhs.x && y == rhs.y; }
};
NANOPLAN_MAKE_STATE_HASHABLE(State2D, s.x, s.y);

class SearchSpace2D final : public nanoplan::SearchSpace<State2D> {
 public:
  static const int w = 1024;
  static const int h = 1024;
  bool use_new_costs = false;
  int new_cost_window = 10;
  double new_cost_mult = 0.5;

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
    if (use_new_costs && to.x < new_cost_window && to.y < new_cost_window) {
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
  double manhattan(const State2D& from, const State2D& to) const {
    return std::abs(to.x - from.x) + std::abs(to.y - from.y);
  }

  std::vector<State2D> get_changed_states() override {
    std::vector<State2D> states;
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        if (y < new_cost_window && x < new_cost_window) {
          states.push_back(State2D{x, y});
        }
      }
    }
    fmt::print("Changed {} states.\n", states.size());
    return states;
  }
};

void test_dijkstra(const State2D& start, const State2D& goal) {
  std::shared_ptr<SearchSpace2D> space2d = std::make_shared<SearchSpace2D>();

  nanoplan::Options options;
  options.timeout_ms = 1000.0;

  nanoplan::Dijkstra<SearchSpace2D> planner(space2d);
  planner.set_options(options);

  const auto plan = planner.plan(start, goal);
  fmt::print(planner.full_report());
  fmt::print("\n");
}

void test_astar(const State2D& start, const State2D& goal) {
  std::shared_ptr<SearchSpace2D> space2d = std::make_shared<SearchSpace2D>();

  nanoplan::Options options;
  options.timeout_ms = 1000.0;

  nanoplan::AStar<SearchSpace2D> planner(space2d);
  planner.set_options(options);

  space2d->use_new_costs = true;
  const auto plan = planner.plan(start, goal);
  fmt::print(planner.full_report());
  fmt::print("\n");
}

void test_lpastar(const State2D& start, const State2D& goal) {
  std::shared_ptr<SearchSpace2D> space2d = std::make_shared<SearchSpace2D>();

  nanoplan::Options options;
  options.timeout_ms = 1000.0;

  nanoplan::LPAStar<SearchSpace2D> planner(space2d);
  planner.set_options(options);

  const auto plan0 = planner.plan(start, goal);
  fmt::print(planner.full_report());
  fmt::print("\n");

  space2d->use_new_costs = true;
  const auto plan1 = planner.replan();
  fmt::print(planner.full_report());
  fmt::print("\n");
}

int main(int argc, char** argv) {
  State2D start{1023, 1023};
  State2D goal{0, 0};

  // test_dijkstra(start, goal);
  test_lpastar(start, goal);
  test_astar(start, goal);

  return 0;
}
