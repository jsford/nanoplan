#ifndef NANOPLAN_LPASTAR_H
#define NANOPLAN_LPASTAR_H

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "ext/flat_hash_map.hpp"
#include "planner.h"
#include "priority_queue.h"
#include "search_space.h"

namespace nanoplan {

template <typename SPACE>
class LPAStar final : public Planner<SPACE> {
 public:
  typedef typename SPACE::state_type STATE;
  LPAStar(std::shared_ptr<SPACE> space);

  std::string planner_name() const override;

  std::vector<STATE> plan(const STATE& from, const STATE& to) override;

  std::vector<STATE> replan() override;

 private:
  struct PairCompare {
    bool operator()(const std::pair<double, double>& a,
                    const std::pair<double, double>& b) const {
      if (a.first > b.first) {
        return true;
      } else if (a.first == b.first) {
        return a.second > b.second;
      }
      return false;
    }
  };
  PriorityQueue<STATE, std::pair<double, double>, PairCompare> pq;
  HashMap<STATE, double> gscores;
  HashMap<STATE, double> rscores;
  ska::flat_hash_set<STATE> closed;

  void initialize();
  void compute_shortest_path();
  void update_node(const STATE& state);
  std::pair<double, double> calculate_key(const STATE& state);
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
LPAStar<SPACE>::LPAStar(std::shared_ptr<SPACE> space) : Planner<SPACE>(space) {}

template <typename SPACE>
std::string LPAStar<SPACE>::planner_name() const {
  return "LPAStar";
}

template <typename SPACE>
std::vector<typename SPACE::state_type> LPAStar<SPACE>::plan(const STATE& from,
                                                             const STATE& to) {
  start_timer();
  summary.elapsed_usec = 0.0;
  summary.total_cost = 0.0;
  summary.expansions = 0;
  summary.termination = Termination::TERMINATION_NOT_SET;

  initialize();
  compute_shortest_path();
  const auto path = backtrack();

  summary.elapsed_usec += check_timer();
  summary.termination = Termination::SUCCESS;
  summary.total_cost = gscores.at(goal);
  return path;
}

template <typename SPACE>
std::vector<typename SPACE::state_type> LPAStar<SPACE>::replan() {
  start_timer();

  for (const auto& edge : space->get_changed_edges()) {
    const auto& to = edge.second;
    update_node(to);
  }

  compute_shortest_path();
  const auto path = backtrack();

  summary.elapsed_usec = check_timer();
  summary.termination = Termination::SUCCESS;
  summary.total_cost = gscores.at(goal);

  return path;
}

template <typename SPACE>
void LPAStar<SPACE>::initialize() {
  pq = PriorityQueue<STATE, std::pair<double, double>, PairCompare>();
  gscores.put(goal, INF_DBL);
  rscores.put(goal, INF_DBL);
  gscores.put(start, INF_DBL);
  rscores.put(start, 0.0);
  pq.insert(start, calculate_key(start));
}

template <typename SPACE>
void LPAStar<SPACE>::compute_shortest_path() {
  while (!pq.empty() && (pq.top_priority() < calculate_key(goal) ||
                         rscores.at(goal) != gscores.at(goal))) {
    const auto curr_state = pq.top();
    pq.pop();
    summary.expansions++;

    if (gscores.at(curr_state) > rscores.at(curr_state)) {
      gscores.put(curr_state, rscores.at(curr_state));
    } else {
      gscores.put(curr_state, INF_DBL);
      update_node(curr_state);
    }
    for (const auto& succ : space->get_successors(curr_state)) {
      update_node(succ);
    }
  }
}

template <typename SPACE>
void LPAStar<SPACE>::update_node(const STATE& state) {
  if (state != start) {
    rscores.put(state, INF_DBL);
    for (const auto& pred : space->get_predecessors(state)) {
      const double r =
          std::min(rscores.at(state),
                   gscores.at(pred) + space->get_from_to_cost(pred, state));
      rscores.put(state, r);
    }
  }
  pq.remove(state);

  if (gscores.at(state) != rscores.at(state)) {
    pq.insert(state, calculate_key(state));
  }
}

template <typename SPACE>
std::pair<double, double> LPAStar<SPACE>::calculate_key(const STATE& state) {
  double tmp = std::min(gscores.at(state), rscores.at(state));
  double h = space->get_from_to_heuristic(state, goal);
  return std::make_pair(tmp + h, tmp);
}

template <typename SPACE>
std::vector<typename SPACE::state_type> LPAStar<SPACE>::backtrack() {
  std::vector<STATE> path;

  STATE state = goal;
  while (state != start) {
    const auto& preds = space->get_predecessors(state);

    // Find the cheapest predecessor to this state.
    STATE best_pred = preds.at(0);
    {
      double min_cost = INF_DBL;
      for (const auto& p : preds) {
        if (!gscores.contains(p)) {
          continue;
        }
        double new_cost = gscores.at(p) + space->get_from_to_cost(p, state);
        if (new_cost < min_cost) {
          min_cost = new_cost;
          best_pred = p;
        }
      }
    }
    path.push_back(state);
    state = best_pred;
  }
  path.push_back(start);
  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace nanoplan

#endif  // NANOPLAN_LPASTAR_H
