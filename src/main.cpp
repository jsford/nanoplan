#include <fmt/format.h>

#include <climits>
#include <cmath>
#include <vector>

#include "nanoplan/nanoplan.h"

struct State2D {
  long int x;
  long int y;

  bool operator==(const State2D& rhs) const { return x == rhs.x && y == rhs.y; }
};
NANOPLAN_MAKE_STATE_HASHABLE(State2D, s.x, s.y);

class SearchSpace2D final : public nanoplan::SearchSpace<State2D> {
 public:
  static const int w = 90;
  static const int h = 90;
  bool use_new_cost = false;

  std::vector<State2D> get_successors(const State2D& state) override {
    std::vector<State2D> succs;
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
    if (use_new_cost && to.x == to.y) {
      return 10 * std::sqrt((to.x - from.x) * (to.x - from.x) +
                            (to.y - from.y) * (to.y - from.y));
    } else {
      return std::sqrt((to.x - from.x) * (to.x - from.x) +
                       (to.y - from.y) * (to.y - from.y));
    }
  }

  double get_from_to_heuristic(const State2D& from,
                               const State2D& to) override {
    return std::sqrt((to.x - from.x) * (to.x - from.x) +
                     (to.y - from.y) * (to.y - from.y));
  }

  std::vector<std::pair<State2D, State2D>> get_changed_edges() override {
    std::vector<std::pair<State2D, State2D>> edges;
    for (int i = 0; i < w; ++i) {
      edges.push_back(std::make_pair(State2D{0, 0}, State2D{i, i}));
    }
    return edges;
  }
};

int main(int argc, char** argv) {
  std::shared_ptr<SearchSpace2D> space2d = std::make_shared<SearchSpace2D>();

  State2D start{0, 0};
  State2D goal{89, 89};

  nanoplan::Options options;
  options.timeout_ms = 10000.0;

  {
    nanoplan::Dijkstra<SearchSpace2D> planner(space2d);
    planner.set_options(options);

    const auto plan = planner.plan(start, goal);
    fmt::print(planner.full_report());
    fmt::print("\n");
  }

  {
    nanoplan::AStar<SearchSpace2D> planner(space2d);
    planner.set_options(options);

    const auto plan = planner.plan(start, goal);
    fmt::print(planner.full_report());
    fmt::print("\n");
  }

  {
    nanoplan::LPAStar<SearchSpace2D> planner(space2d);
    planner.set_options(options);

    const auto plan0 = planner.plan(start, goal);
    fmt::print(planner.full_report());
    fmt::print("\n");
    for (const auto& s : plan0) {
      fmt::print("({}, {})\n", s.x, s.y);
    }

    space2d->use_new_cost = true;

    const auto plan1 = planner.replan();
    fmt::print(planner.full_report());
    fmt::print("\n");
    for (const auto& s : plan1) {
      fmt::print("({}, {})\n", s.x, s.y);
    }
  }

  return 0;
}
