#ifndef NANOPLAN_LPASTAR_H
#define NANOPLAN_LPASTAR_H

#include "search_space.h"
#include "planner.h"
#include "priority_queue.h"
#include "ext/flat_hash_map.hpp"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <algorithm>

#include <fmt/format.h>

namespace nanoplan {

template <typename SPACE>
class LPAStar final : public Planner<SPACE> {
    public:
        LPAStar(const SPACE& space);

        std::string planner_name() const override;

        std::vector<typename SPACE::state_type> plan() override;
        std::vector<typename SPACE::state_type> replan() override;
        void update_cost(const typename SPACE::state_type& from , const typename SPACE::state_type& to, double cost) override;

    private:
        PriorityQueue<typename SPACE::state_type> pq;
        ska::flat_hash_map<typename SPACE::state_type,typename SPACE::state_type> preds;
        ska::flat_hash_map<typename SPACE::state_type, double> gscores;
        ska::flat_hash_set<typename SPACE::state_type> closed;

        void reset();

        using Planner<SPACE>::space;
        using Planner<SPACE>::start;
        using Planner<SPACE>::goal;
        using Planner<SPACE>::options;
        using Planner<SPACE>::summary;
        using Planner<SPACE>::start_timer;
        using Planner<SPACE>::check_timer;
};

template <typename SPACE>
LPAStar<SPACE>::LPAStar(const SPACE& space) : Planner<SPACE>(space) {}

template <typename SPACE>
std::string LPAStar<SPACE>::planner_name() const {
    return "LPAStar";
}

template <typename SPACE>
void LPAStar<SPACE>::reset() {
    pq = PriorityQueue<typename SPACE::state_type>();
    preds.clear();
    gscores.clear();
    closed.clear();
    summary = {};   // NOTE(Jordan): Will this fail if summary is not POD?
}

template <typename SPACE>
std::vector<typename SPACE::state_type> LPAStar<SPACE>::plan() {
    start_timer();

    reset();    // Empty all data structures and plan from scratch.

    pq.push( {start, 0} );
    preds[start] = start;
    gscores[start] = 0.0;

    while( !pq.empty() ) {
        const auto curr_state = pq.top().first;
        const auto curr_cost  = pq.top().second;
        summary.expansions++;

        if( curr_state == goal ) {
            summary.termination = Termination::SUCCESS;
            break;
        }

        if( options.timeout_ms > 0.0 && check_timer() >= 1000*options.timeout_ms ) {
            summary.termination = Termination::TIMEOUT;
            break;
        }

        closed.insert(curr_state);
        pq.pop();

        const auto& succs = space.get_successors(curr_state);

        const auto curr_gscore = gscores[curr_state];
        for(const auto& succ : succs) {
            const auto& succ_state = succ.first;
            const auto& succ_cost  = succ.second;

            if( closed.find(succ_state) == closed.end() ) {
                double h = space.get_from_to_heuristic(succ_state, goal);
                double g = succ_cost + curr_gscore;
                pq.push( { succ_state, g+h } );
                gscores[succ_state] = g;
                preds[succ_state] = curr_state;
            }
        }
    }

    std::vector<typename SPACE::state_type> path;

    if( summary.termination == Termination::SUCCESS ) {
        auto s = goal;
        while( !(s == start) ) {
            path.push_back(s);
            s = preds[s];
        };
        path.push_back(start);

        std::reverse(path.begin(), path.end());
        summary.total_cost = gscores[goal];
        summary.termination = Termination::SUCCESS;
    } else if( summary.termination == Termination::TIMEOUT ) {
        summary.total_cost = 0.0;
    } else {
        summary.termination = Termination::UNREACHABLE;
    }

    summary.elapsed_usec = check_timer();
    return path;
}

template <typename SPACE>
std::vector<typename SPACE::state_type> LPAStar<SPACE>::replan() {
    // TODO(Jordan): Implement this!
    return plan();
}

template <typename SPACE>
void LPAStar<SPACE>::update_cost(const typename SPACE::state_type& from , const typename SPACE::state_type& to, double cost) {
    // TODO(Jordan): Implement this!
}

} // namespace nanoplan

#endif // NANOPLAN_LPASTAR_H
