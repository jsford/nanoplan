#ifndef NANOPLAN_ASTAR_H
#define NANOPLAN_ASTAR_H

#include <algorithm>
#include <memory>
#include <vector>

#include "ext/flat_hash_map.hpp"
#include "planner.h"
#include "priority_queue.h"
#include "search_space.h"

namespace nanoplan {

template <typename SPACE>
class AStar final : public Planner<SPACE> {
 public:
  AStar(std::shared_ptr<SPACE> space);

  std::string planner_name() const override;

  std::vector<typename SPACE::state_type> plan(
      const typename SPACE::state_type& start,
      const typename SPACE::state_type& goal) override;

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
AStar<SPACE>::AStar(std::shared_ptr<SPACE> space) : Planner<SPACE>(space) {}

template <typename SPACE>
std::string AStar<SPACE>::planner_name() const {
  return "AStar";
}

template <typename SPACE>
std::vector<typename SPACE::state_type> AStar<SPACE>::plan(
    const typename SPACE::state_type& start,
    const typename SPACE::state_type& goal) {
  using STATE = typename SPACE::state_type;

  summary.termination = Termination::TERMINATION_NOT_SET;
  summary.expansions = 0;
  summary.elapsed_usec = 0;

  // Time the search for debug and optimization purposes.
  start_timer();

  // Saving the start and goal states simplifies the interface for
  // replanning because we will not have to ask the user to provide information
  // they already gave us. This means the user can call replan() without having
  // to provide start or goal again.
  this->start = start;
  this->goal = goal;

  // The open queue is a priority queue containing the frontier.
  // Nodes in open have been reached by the search but have not yet been
  // expanded.
  PriorityQueue<STATE, Cost> open;

  // The closed set contains nodes that have already been visited and expanded.
  // Once a node goes into the closed set, it will never need to be considered
  // again.
  ska::flat_hash_set<STATE> closed;

  // The nodemap contains bookkeeping data for each visited node in the search.
  // nodemap[state].pred is the "backpointer" used to backtrack the final path.
  // nodemap[state].gscore is the cost-to-go, the total cost incurred to reach a
  // given state from the start.
  struct NodeData {
    STATE pred;
    Cost gscore;
  };
  ska::flat_hash_map<STATE, NodeData> nodemap;

  // Initialization
  {
    // Push the start state onto the open queue.
    // Its cost-to-go, or gscore, is zero,
    // so its priority f is equal to the heuristic h.
    // f = 0 + h;
    const Cost h = space->get_from_to_heuristic(start, goal);
    open.insert(start, h);

    // The start node backpointer points to itself.
    // This is optional but may help find bugs in backtracking.
    nodemap[start].pred = start;

    // The start node gscore is zero.
    // Think of gscore as the "cost to get here from the start state".
    nodemap[start].gscore = 0.0;
  }

  while (true) {
    if (open.empty()) {
      summary.termination = Termination::UNREACHABLE;
      break;
    }
    // Grab the min-priority state and remove it from open.
    const STATE curr_state = open.top();
    open.pop();

    // If you have expanded this state before, skip it.
    if (closed.find(curr_state) != closed.end()) {
      continue;
    }

    // Otherwise, add it to closed and expand it.
    closed.insert(curr_state);

    // Count total expansions to help the user debug their search.
    summary.expansions++;

    // If this state is the goal, quit searching!
    if (curr_state == goal) {
      summary.termination = Termination::SUCCESS;
      break;
    }

    // If the user set a timeout and you have exceeded it, give up.
    if (options.timeout_ms > 0.0 &&
        check_timer() >= 1000 * options.timeout_ms) {
      summary.termination = Termination::TIMEOUT;
      break;
    }

    // Ask the search space for the successors of this state.
    // Where can I get to from curr_state?
    const auto& succs = space->get_successors(curr_state);

    // Lookup the cost-to-go to get to curr_state.
    const auto curr_gscore = nodemap[curr_state].gscore;

    // Examine each successor of curr_state.
    for (const auto& succ : succs) {
      // Calculate tentative_g, the cost to get to succ by way of curr_state.
      const Cost succ_cost = space->get_from_to_cost(curr_state, succ);
      const Cost tentative_g = curr_gscore + succ_cost;

      // If this is a new route to succ or it is cheaper than our previous
      // route, replace the old information in the bookkeeping data.
      if (nodemap.find(succ) == nodemap.end() ||
          tentative_g < nodemap.at(succ).gscore) {
        // Calculate the heuristic.
        // This estimates the remaining cost to get from succ to goal.
        Cost h = space->get_from_to_heuristic(succ, goal);

        // Push this successor onto the open queue.
        const Cost f = tentative_g + h;
        open.insert(succ, f);

        // Remember that the best known route to succ is through curr_state.
        nodemap[succ].pred = curr_state;

        // Remember how much the best known route to succ costs
        // so we can compare it to routes we find later.
        nodemap[succ].gscore = tentative_g;
      }
    }
  }

  std::vector<STATE> path;
  if (summary.termination == Termination::SUCCESS &&
      nodemap[goal].gscore < Cost::max()) {
    // If the search succeeded, backtrack to find the least-cost path.
    STATE s = goal;
    while (!(s == start)) {
      path.push_back(s);
      s = nodemap[s].pred;
    };
    path.push_back(start);

    std::reverse(path.begin(), path.end());
    summary.total_cost = nodemap[goal].gscore;
  } else if (summary.termination == Termination::TIMEOUT) {
    // If the search timed out, return an empty path with cost 0.
    summary.total_cost = Cost::max();
  } else if (summary.termination == Termination::TERMINATION_NOT_SET) {
    // Don't do anything, I guess.
  } else {
    // If the goal was not reachable, return an empty path with cost 0.
    summary.termination = Termination::UNREACHABLE;
    summary.total_cost = Cost::max();
  }

  // Record the search duration.
  summary.elapsed_usec = check_timer();

  return path;
}

}  // namespace nanoplan

#endif  // NANOPLAN_ASTAR_H
