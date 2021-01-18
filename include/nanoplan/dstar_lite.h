#ifndef NANOPLAN_DSTAR_LITE_H
#define NANOPLAN_DSTAR_LITE_H

#include <algorithm>
#include <cassert>
#include <iostream>
#include <memory>
#include <vector>

#include "ext/flat_hash_map.hpp"
#include "hash_map.h"
#include "planner.h"
#include "priority_queue.h"
#include "search_space.h"

namespace nanoplan {

template <typename SPACE>
class DStarLite final : public Planner<SPACE> {
 public:
  typedef typename SPACE::state_type STATE;
  DStarLite(std::shared_ptr<SPACE> space);

  std::string planner_name() const override;

  std::vector<STATE> plan(const STATE& from, const STATE& to) override;

  std::vector<STATE> replan() override;

 private:
  struct Key {
    Cost first = 0.0;
    Cost second = 0.0;

    bool operator<(const Key& rhs) const {
      if (first < rhs.first) {
        return true;
      } else if (first == rhs.first) {
        return second < rhs.second;
      } else {
        return false;
      }
    }
    bool operator==(const Key& rhs) const {
      return first == rhs.first && second == rhs.second;
    }
    bool operator>(const Key& rhs) const {
      return !operator<(rhs) && !operator==(rhs);
    }
    bool operator>=(const Key& rhs) const {
      return operator>(rhs) || operator==(rhs);
    }
    bool operator<=(const Key& rhs) const {
      return operator<(rhs) || operator==(rhs);
    }
  };
  PriorityQueueWithRemove<STATE, Key> pq;
  HashMap<STATE, Cost> gscores;
  HashMap<STATE, Cost> rscores;
  ska::flat_hash_set<STATE> closed;

  void initialize();
  std::vector<STATE> compute_shortest_path();
  void update_node(const STATE& state);
  Key calculate_key(const STATE& state);
  std::vector<STATE> backtrack();

  using Planner<SPACE>::space;
  using Planner<SPACE>::goal;
  using Planner<SPACE>::start;
  using Planner<SPACE>::options;
  using Planner<SPACE>::summary;
  using Planner<SPACE>::start_timer;
  using Planner<SPACE>::check_timer;
};

template <typename SPACE>
DStarLite<SPACE>::DStarLite(std::shared_ptr<SPACE> space)
    : Planner<SPACE>(space) {}

template <typename SPACE>
std::string DStarLite<SPACE>::planner_name() const {
  return "DStarLite";
}

template <typename SPACE>
std::vector<typename SPACE::state_type> DStarLite<SPACE>::plan(
    const STATE& from, const STATE& to) {
  this->goal = from;
  this->start = to;

  start_timer();
  summary.elapsed_usec = 0.0;
  summary.total_cost = 0.0;
  summary.expansions = 0;
  summary.termination = Termination::TERMINATION_NOT_SET;

  initialize();
  const auto path = compute_shortest_path();

  summary.elapsed_usec += check_timer();
  summary.total_cost = gscores.at(start);
  return path;
}

template <typename SPACE>
std::vector<typename SPACE::state_type> DStarLite<SPACE>::replan() {
  start_timer();
  summary.expansions = 0;

  const auto& changed_states = space->get_changed_states();
  for (const auto& s : changed_states) {
    update_node(s);
  }

  const auto path = compute_shortest_path();

  summary.elapsed_usec = check_timer();
  summary.total_cost = gscores.at(start);

  return path;
}

template <typename SPACE>
void DStarLite<SPACE>::initialize() {
  pq = PriorityQueueWithRemove<STATE, Key>();
  rscores.put(goal, 0.0);
  pq.insert(goal, calculate_key(goal));
}

template <typename SPACE>
std::vector<typename SPACE::state_type>
DStarLite<SPACE>::compute_shortest_path() {
  while (true) {
    if (rscores.at(start) == gscores.at(start) &&
        gscores.at(start) < Cost::max()) {
      if (pq.empty() || pq.top_priority() >= calculate_key(start)) {
        summary.termination = Termination::SUCCESS;
        break;
      }
    }

    if (pq.empty()) {
      summary.termination = Termination::UNREACHABLE;
      break;
    }

    const auto curr_state = pq.top();
    pq.pop();

    summary.expansions++;

    const auto curr_rscore = rscores.at(curr_state);
    const auto curr_gscore = gscores.at(curr_state);

    if (curr_gscore > curr_rscore) {
      gscores.put(curr_state, curr_rscore);
    } else if (curr_gscore < curr_rscore) {
      gscores.put(curr_state, Cost::max());
      update_node(curr_state);
    } else {
      const auto msg =
          "NANOPLAN ERROR: D*-lite is expanding a consistent state.";
      throw std::runtime_error(msg);
    }

    for (const auto& succ : space->get_successors(curr_state)) {
      update_node(succ);
    }

    if (options.timeout_ms > 0.0 &&
        check_timer() >= 1000 * options.timeout_ms) {
      summary.termination = Termination::TIMEOUT;
      break;
    }
  }

  if (summary.termination == Termination::SUCCESS) {
    summary.termination = Termination::SUCCESS;
    return backtrack();
  } else {
    return {};
  }
}

template <typename SPACE>
void DStarLite<SPACE>::update_node(const STATE& state) {
  if (!(state == goal)) {
    Cost new_rscore = Cost::max();
    for (const auto& pred : space->get_predecessors(state)) {
      const Cost r = gscores.at(pred) + space->get_from_to_cost(pred, state);
      new_rscore = std::min(new_rscore, r);
    }
    rscores.put(state, new_rscore);

    pq.remove(state);  // remove if present.
    if (gscores.at(state) != rscores.at(state)) {
      pq.insert(state, calculate_key(state));
    }
  }
}

template <typename SPACE>
typename DStarLite<SPACE>::Key DStarLite<SPACE>::calculate_key(
    const STATE& state) {
  Key k;
  k.second = std::min(gscores.at(state), rscores.at(state));
  k.first = k.second + space->get_from_to_heuristic(state, start);
  assert(k.second >= 0 && k.first >= 0);
  return k;
}

template <typename SPACE>
std::vector<typename SPACE::state_type> DStarLite<SPACE>::backtrack() {
  std::vector<STATE> path;

  STATE state = start;
  while (!(state == goal)) {
    const auto& preds = space->get_predecessors(state);

    // Find the cheapest predecessor to this state.
    STATE best_pred = preds.at(0);
    {
      Cost min_cost = Cost::max();
      for (const auto& pred : preds) {
        Cost new_cost = gscores.at(pred) + space->get_from_to_cost(pred, state);
        if (new_cost < min_cost) {
          min_cost = new_cost;
          best_pred = pred;
        }
      }
    }
    path.push_back(state);
    state = best_pred;
  }
  path.push_back(goal);
  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace nanoplan

#endif  // NANOPLAN_DSTAR_LITE_H
