#ifndef NANOPLAN_DIJKSTRA_H
#define NANOPLAN_DIJKSTRA_H

#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ext/flat_hash_map.hpp"
#include "planner.h"
#include "priority_queue.h"
#include "search_space.h"

namespace nanoplan {

template <typename SPACE>
class Dijkstra final : public Planner<SPACE> {
 public:
  // Construct an algorithm to search over a given search space.
  Dijkstra(std::shared_ptr<SPACE> space);

  // Search over the search space.
  std::vector<typename SPACE::state_type> plan(
      const typename SPACE::state_type& start,
      const typename SPACE::state_type& goal) override;

  std::string planner_name() const override { return "Dijkstra"; }

 private:
  using Planner<SPACE>::space;
  using Planner<SPACE>::start;
  using Planner<SPACE>::goal;
  using Planner<SPACE>::options;
  using Planner<SPACE>::summary;
  using Planner<SPACE>::start_timer;
  using Planner<SPACE>::check_timer;
};

template <typename SPACE>
Dijkstra<SPACE>::Dijkstra(std::shared_ptr<SPACE> space)
    : Planner<SPACE>(space) {}

template <typename SPACE>
std::vector<typename SPACE::state_type> Dijkstra<SPACE>::plan(
    const typename SPACE::state_type& start,
    const typename SPACE::state_type& goal) {
  start_timer();
  using STATE = typename SPACE::state_type;

  this->start = start;
  this->goal = goal;

  struct NodeData {
    STATE pred;
    double gscore;
  };

  PriorityQueue<STATE, double> open;
  ska::flat_hash_set<STATE> closed;
  ska::flat_hash_map<STATE, NodeData> nodemap;

  open.insert(start, 0.0);
  nodemap[start].pred = start;
  nodemap[start].gscore = 0.0;

  while (!open.empty()) {
    const STATE curr_state = open.top();

    open.pop();
    if (closed.find(curr_state) != closed.end()) {
      continue;
    }
    closed.insert(curr_state);
    summary.expansions++;

    if (curr_state == goal) {
      summary.termination = Termination::SUCCESS;
      break;
    }

    if (options.timeout_ms > 0.0 &&
        check_timer() >= 1000 * options.timeout_ms) {
      summary.termination = Termination::TIMEOUT;
      break;
    }

    const auto& succs = space->get_successors(curr_state);

    const auto curr_gscore = nodemap[curr_state].gscore;
    for (const auto& succ : succs) {
      const double succ_cost = space->get_from_to_cost(curr_state, succ);
      const double tentative_g = curr_gscore + succ_cost;

      if (nodemap.find(succ) == nodemap.end() ||
          tentative_g < nodemap.at(succ).gscore) {
        open.insert(succ, tentative_g);
        nodemap[succ].pred = curr_state;
        nodemap[succ].gscore = tentative_g;
      }
    }
  }

  std::vector<STATE> path;
  if (summary.termination == Termination::SUCCESS) {
    STATE s = goal;
    while (!(s == start)) {
      path.push_back(s);
      s = nodemap[s].pred;
    };
    path.push_back(start);

    std::reverse(path.begin(), path.end());
    summary.total_cost = nodemap[goal].gscore;
    summary.termination = Termination::SUCCESS;
  } else if (summary.termination == Termination::TIMEOUT) {
    summary.total_cost = INF_DBL;
  } else {
    summary.termination = Termination::UNREACHABLE;
    summary.total_cost = INF_DBL;
  }

  summary.elapsed_usec = check_timer();
  return path;
}

}  // namespace nanoplan

#endif  // NANOPLAN_DIJKSTRA_H
