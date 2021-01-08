#include <fmt/format.h>

#include <climits>
#include <cmath>
#include <vector>

#include "nanoplan/nanoplan.h"
#include "space2d.h"

void test_dijkstra(const State2D& start, const State2D& goal) {
  std::shared_ptr<SearchSpace2D> space2d = std::make_shared<SearchSpace2D>();

  nanoplan::Options options;
  options.timeout_ms = 1000.0;

  nanoplan::Dijkstra<SearchSpace2D> planner(space2d);
  planner.set_options(options);

  const auto plan = planner.plan(start, goal);
  fmt::print(planner.full_report());
}

void test_astar(const State2D& start, const State2D& goal) {
  std::shared_ptr<SearchSpace2D> space2d = std::make_shared<SearchSpace2D>();

  nanoplan::Options options;
  options.timeout_ms = 1000.0;

  nanoplan::AStar<SearchSpace2D> planner(space2d);
  planner.set_options(options);

  const auto plan0 = planner.plan(start, goal);
  fmt::print(planner.full_report());
  fmt::print("\n");

  space2d->use_new_costs = true;
  const auto plan1 = planner.plan(start, goal);
  fmt::print(planner.full_report());
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
}

int main(int argc, char** argv) {
  State2D start{0, 0};
  State2D goal{4095, 4095};

  test_dijkstra(start, goal);
  fmt::print("\n");
  test_lpastar(start, goal);
  fmt::print("\n");
  test_astar(start, goal);

  return 0;
}
