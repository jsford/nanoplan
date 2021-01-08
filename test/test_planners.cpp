#include <nanoplan/nanoplan.h>

#include <catch2/catch.hpp>

using namespace nanoplan;

struct State2D {
  int x;
  int y;
  bool operator==(const State2D& rhs) const { return x == rhs.x && y == rhs.y; }
};
NANOPLAN_MAKE_STATE_HASHABLE(State2D, s.x, s.y);

struct Space2D final : SearchSpace<State2D> {
  std::vector<State2D> get_successors(const State2D& state) override {
    std::vector<State2D> succs;
    succs.push_back(State2D{state.x + 1, state.y + 0});
    succs.push_back(State2D{state.x + 0, state.y + 1});
    succs.push_back(State2D{state.x - 1, state.y - 0});
    succs.push_back(State2D{state.x - 0, state.y - 1});
    return succs;
  }
  std::vector<State2D> get_predecessors(const State2D& state) override {
    return get_successors(state);
  }
  double get_from_to_cost(const State2D& from, const State2D& to) override {
    return 1.0;
  }
  double get_from_to_heuristic(const State2D& from,
                               const State2D& to) override {
    return std::max(from.x - to.x, from.y - to.y);
  }
};

TEST_CASE("Test dijkstra", "[one_shot_planning]") {
  auto space2d = std::make_shared<Space2D>();
  Dijkstra<Space2D> planner(space2d);
  const auto& plan = planner.plan(State2D{0, 0}, State2D{10, 10});
  REQUIRE(planner.get_summary().total_cost == 20);
}

TEST_CASE("Test astar", "[one_shot_planning]") {
  auto space2d = std::make_shared<Space2D>();
  AStar<Space2D> planner(space2d);
  const auto& plan = planner.plan(State2D{0, 0}, State2D{10, 10});
  REQUIRE(planner.get_summary().total_cost == 20);
}

TEST_CASE("Test lpastar", "[one_shot_planning]") {
  auto space2d = std::make_shared<Space2D>();
  LPAStar<Space2D> planner(space2d);
  const auto& plan = planner.plan(State2D{0, 0}, State2D{10, 10});
  REQUIRE(planner.get_summary().total_cost == 20);
}
