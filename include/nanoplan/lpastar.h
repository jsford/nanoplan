#ifndef NANOPLAN_LPASTAR_H
#define NANOPLAN_LPASTAR_H

#include <fmt/format.h>

#include <algorithm>
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
  LPAStar(std::shared_ptr<SPACE> space);

  std::string planner_name() const override;

  std::vector<typename SPACE::state_type> plan() override;
  std::vector<typename SPACE::state_type> replan() override;

  void update_cost(const typename SPACE::state_type& from,
                   const typename SPACE::state_type& to, double cost) override;

 private:
  PriorityQueueWithTieBreaker<typename SPACE::state_type> pq;
  ska::flat_hash_map<typename SPACE::state_type, typename SPACE::state_type>
      preds;
  ska::flat_hash_map<typename SPACE::state_type, double> gscores;
  ska::flat_hash_map<typename SPACE::state_type, double> rhs;
  ska::flat_hash_set<typename SPACE::state_type> closed;

  void initialize();
  void compute_shortest_path();
  void update_node(const typename SPACE::state_type& state);
  std::pair<double, double> calculate_key(
      const typename SPACE::state_type& state) const;

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
std::vector<typename SPACE::state_type> LPAStar<SPACE>::plan() {
  initialize();
  compute_shortest_path();
  // TODO(Jordan): Backtrack and return the plan?
  return {};
}

template <typename SPACE>
std::vector<typename SPACE::state_type> LPAStar<SPACE>::replan() {
  for (auto node : space->get_changed_states()) {
    auto& node_state = node.first;
    auto& node_cost = node.second;
    update_node(node_state);
  }
  // TODO(Jordan): Backtrack and return the plan?
  return {};
}

template <typename SPACE>
void LPAStar<SPACE>::update_cost(const typename SPACE::state_type& from,
                                 const typename SPACE::state_type& to,
                                 double cost) {
  // TODO(Jordan): Implement this!
}

template <typename SPACE>
void LPAStar<SPACE>::initialize() {
  pq = PriorityQueueWithTieBreaker<typename SPACE::state_type>();
  preds.clear();
  gscores.clear();
  rhs.clear();
  closed.clear();

  rhs[start] = 0;
  pq.push({start, calculate_key(start)});

  summary = {};  // NOTE(Jordan): Will this break if summary is not POD?
}

template <typename SPACE>
void LPAStar<SPACE>::compute_shortest_path() {
  while ((pq.top() < calculate_key(goal)) || (rhs[goal] != gscores[goal])) {
    auto node = pq.pop();
    if (gscores[node] > rhs[node]) {
      gscores[node] = rhs[node];
    } else {
      gscores[node] = std::numeric_limits<double>::infinity();
      update_node(node);
    }
    for (auto succ : space->get_successors(node)) {
      auto succ_state = succ.first;
      auto succ_cost = succ.second;
      update_node(succ_state);
    }
  }
}

template <typename SPACE>
void LPAStar<SPACE>::update_node(const typename SPACE::state_type& state) {
  if (state != start) {
    rhs[state] = std::numeric_limits<double>::infinity();
    for (const auto& pred : space->get_predecessors(state)) {
      const auto& pred_state = pred.first;
      const auto& pred_cost = pred.second;
      rhs[state] = std::min(rhs[state], gscores[pred_state] + pred_cost);

      if (pq.contains(state)) {
        pq.remove(state);
      }
      if (gscores[state] != rhs[state]) {
        pq.insert(state, calculate_key(state));
      }
    }
  }
}

template <typename SPACE>
std::pair<double, double> LPAStar<SPACE>::calculate_key(
    const typename SPACE::state_type& state) const {
  auto tmp = std::min(gscores[state], rhs[state]);
  auto h = space->get_from_to_heuristic(state, goal);
  return std::make_pair(tmp + h, tmp);
}

}  // namespace nanoplan

#endif  // NANOPLAN_LPASTAR_H
