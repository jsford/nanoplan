#ifndef NANOPLAN_DIJKSTRA_H
#define NANOPLAN_DIJKSTRA_H

#include "search_space.h"
#include "planner.h"
#include "priority_queue.h"
#include "ext/flat_hash_map.hpp"
#include <vector>
#include <utility>
#include <unordered_map>
#include <algorithm>

namespace nanoplan {

template <typename SPACE>
class Dijkstra final : public Planner<SPACE> {
    public:
        // Construct an algorithm to search over a given search space.
        Dijkstra(const SPACE& space);

        // Search over the search space.
        std::vector<typename SPACE::state_type> plan() override;

        std::string planner_name() const override {
            return "Dijkstra";
        }

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
Dijkstra<SPACE>::Dijkstra(const SPACE& space) :
    Planner<SPACE>(space) {}

template <typename SPACE>
std::vector<typename SPACE::state_type> Dijkstra<SPACE>::plan() {
    using STATE = typename SPACE::state_type;

    start_timer();

    PriorityQueue<STATE> pq;
    ska::flat_hash_map<STATE, STATE> preds;

    pq.push( {start, 0} );
    preds[start] = start;

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

        pq.pop();

        const auto& succs = space.get_successors(curr_state);

        for(const auto& succ : succs) {
            const auto& succ_state = succ.first;
            const auto& succ_cost  = succ.second;

            if( preds.count(succ_state) == 0 ) {
                pq.push( { succ_state, succ_cost+curr_cost } );
                preds[succ_state] = curr_state;
            }
        }
    }

    std::vector<STATE> path;

    if(summary.termination == Termination::SUCCESS) {
        summary.total_cost = pq.top().second;
        typename SPACE::state_type s = goal;
        while( !(s == start) ) {
            path.push_back(s);
            s = preds[s];
        };
        path.push_back(start);

        std::reverse(path.begin(), path.end());
        summary.termination = Termination::SUCCESS;
    } else if(summary.termination == Termination::TIMEOUT) {
        summary.total_cost = 0.0;
    } else {
        summary.termination = Termination::UNREACHABLE;
        summary.total_cost = 0.0;
    }
    summary.elapsed_usec = check_timer();

    return path;
}

} // namespace nanoplan

#endif // NANOPLAN_DIJKSTRA_H
