#ifndef NANOPLAN_DSTAR_H
#define NANOPLAN_DSTAR_H

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
class DStar final : public Planner<SPACE> {
 public:
  typedef typename SPACE::state_type STATE;
  DStar(std::shared_ptr<SPACE> space);

  std::string planner_name() const override;

  std::vector<STATE> plan(const STATE& start, const STATE& goal) override;
  std::vector<STATE> replan() override;
  std::vector<STATE> replan(const STATE& start) override;

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
  Cost k = Cost(0);
  STATE last_start;

  void initialize();
  std::vector<STATE> compute_shortest_path();
  void update_vertex(const STATE& state);
  Key calculate_key(const STATE& state);
  std::vector<STATE> backtrack();

  using Planner<SPACE>::space;
  using Planner<SPACE>::start;
  using Planner<SPACE>::goal;
  using Planner<SPACE>::options;
  using Planner<SPACE>::summary;
  using Planner<SPACE>::start_timer;
  using Planner<SPACE>::check_timer;
};

template <typename SPACE>
DStar<SPACE>::DStar(std::shared_ptr<SPACE> space) : Planner<SPACE>(space) {}

template <typename SPACE>
std::string DStar<SPACE>::planner_name() const {
  return "DStar";
}

template <typename SPACE>
std::vector<typename SPACE::state_type> DStar<SPACE>::plan(const STATE& start,
                                                           const STATE& goal) {
  this->last_start = start;
  this->start = start;
  this->goal = goal;

  start_timer();
  summary.elapsed_usec = 0.0;
  summary.total_cost = 0.0;
  summary.expansions = 0;
  summary.termination = Termination::TERMINATION_NOT_SET;

  initialize();
  const auto path = compute_shortest_path();

  summary.elapsed_usec += check_timer();
  summary.total_cost = gscores.at(goal);
  return path;
}

template <typename SPACE>
std::vector<typename SPACE::state_type> DStar<SPACE>::replan(
    const STATE& start) {
  if (this->start == start) {
    return replan();
  } else {
    this->last_start = this->start;
    this->start = start;
    k = k + space->get_from_to_heuristic(this->last_start, this->start);
    return replan();
  }
}

template <typename SPACE>
std::vector<typename SPACE::state_type> DStar<SPACE>::replan() {
  start_timer();
  summary.expansions = 0;

  const auto& changed_states = space->get_changed_states();
  // NOTE(Jordan): This part is wrong.
  for (const auto& s : changed_states) {
    update_vertex(s);
  }

  const auto path = compute_shortest_path();

  summary.elapsed_usec = check_timer();
  summary.total_cost = gscores.at(start);

  return path;
}

template <typename SPACE>
void DStar<SPACE>::initialize() {
  pq = PriorityQueueWithRemove<STATE, Key>();
  gscores.clear();
  rscores.clear();

  rscores.put(goal, 0.0);
  pq.insert(goal, calculate_key(goal));
  k = Cost(0);
}

template <typename SPACE>
std::vector<typename SPACE::state_type> DStar<SPACE>::compute_shortest_path() {
  while (true) {
    if (rscores.at(start) <= gscores.at(start) &&
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
    const auto key_old = pq.top_priority();
    const auto key_new = calculate_key(curr_state);

    summary.expansions++;

    if (key_old < key_new) {
      pq.insert(curr_state, key_new);
    } else if (gscores.at(curr_state) > rscores.at(curr_state) ||
               gscores.at(curr_state) == Cost::max()) {
      gscores.put(curr_state, rscores.at(curr_state));
      pq.remove(curr_state);
      for (const auto& pred : space->get_predecessors(curr_state)) {
        if (!(pred == goal)) {
          const auto r = rscores.at(pred);
          const auto c = space->get_from_to_cost(pred, curr_state);
          const auto g = gscores.at(curr_state);
          rscores.put(pred, std::min(r, c + g));
        }
        update_vertex(pred);
      }
    } else {
      auto g_old = gscores.at(curr_state);
      gscores.put(curr_state, Cost::max());
      auto nodes = space->get_predecessors(curr_state);
      nodes.push_back(curr_state);
      for (const auto& s : nodes) {
        if (rscores.at(s) == space->get_from_to_cost(s, curr_state) + g_old) {
          if (!(s == goal)) {
            auto min_c = Cost::max();
            for (const auto& sp : space->get_successors(s)) {
              auto c = space->get_from_to_cost(s, sp) + gscores.at(sp);
              if (c < min_c) {
                min_c = c;
              }
            }
            rscores.put(s, min_c);
          }
        }
        update_vertex(s);
      }
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
void DStar<SPACE>::update_vertex(const STATE& state) {
  pq.remove(state);
  if (gscores.at(state) != rscores.at(state) ||
      gscores.at(state) == Cost::max()) {
    pq.insert(state, calculate_key(state));
  }
}

template <typename SPACE>
typename DStar<SPACE>::Key DStar<SPACE>::calculate_key(const STATE& state) {
  auto g = gscores.at(state);
  auto r = rscores.at(state);
  auto h = space->get_from_to_heuristic(state, start);

  Key key;
  key.first = std::min(g, r + h + k);
  key.second = std::min(g, r);
  return key;
}

template <typename SPACE>
std::vector<typename SPACE::state_type> DStar<SPACE>::backtrack() {
  std::vector<STATE> path;

  STATE state = start;
  while (!(state == goal)) {
    const auto& succs = space->get_successors(state);

    // Find the cheapest predecessor to this state.
    STATE best_succ = succs.at(0);
    {
      Cost min_cost = Cost::max();
      for (const auto& succ : succs) {
        Cost new_cost = gscores.at(succ) + space->get_from_to_cost(state, succ);
        if (new_cost < min_cost) {
          min_cost = new_cost;
          best_succ = succ;
        }
      }
    }
    path.push_back(state);
    state = best_succ;
  }
  path.push_back(goal);
  return path;
}

}  // namespace nanoplan

#endif  // NANOPLAN_DSTAR_H
