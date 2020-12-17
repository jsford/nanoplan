#ifndef NANOPLAN_DIJKSTRA_H
#define NANOPLAN_DIJKSTRA_H

#include "search_space.h"
#include "planner.h"
#include "priority_queue.h"
#include <vector>
#include <utility>
#include <unordered_map>
#include <algorithm>

namespace nanoplan {

template <typename SPACE, typename STATE>
class Dijkstra final : public Planner<SPACE, STATE> {
    public:

        // Construct an algorithm to search over a given search space.
        Dijkstra(const SPACE& space);

        // Search over the search space.
        std::vector<STATE> plan() override;

        std::string planner_name() const override {
            return "Dijkstra";
        }

    private:
        using Planner<SPACE,STATE>::space;
        using Planner<SPACE,STATE>::start;
        using Planner<SPACE,STATE>::goal;
        using Planner<SPACE,STATE>::options;
        using Planner<SPACE,STATE>::summary;
        using Planner<SPACE,STATE>::start_timer;
        using Planner<SPACE,STATE>::check_timer;
};

template <typename SPACE, typename STATE>
Dijkstra<SPACE,STATE>::Dijkstra(const SPACE& space) :
    Planner<SPACE, STATE>(space) {}

template <typename SPACE, typename STATE>
std::vector<STATE> Dijkstra<SPACE,STATE>::plan() {
    start_timer();

    PriorityQueue<STATE> pq;
    std::unordered_map<STATE,STATE> preds;

    pq.push( {start, 0} );
    preds[start] = start;

    while( !pq.empty() ) {
        const auto curr_state = pq.top().first;
        const auto curr_cost  = pq.top().second;
        summary.expansions++;

        if( curr_state == goal ) {
            summary.termination = Planner<SPACE,STATE>::Termination::SUCCESS;
            break;
        }

        if( options.timeout_ms > 0.0 && check_timer() >= 1000*options.timeout_ms ) {
            summary.termination = Planner<SPACE,STATE>::Termination::TIMEOUT;
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

    if(summary.termination == Planner<SPACE,STATE>::Termination::SUCCESS) {
        summary.total_cost = pq.top().second;
        STATE s = goal;
        while( !(s == start) ) {
            path.push_back(s);
            s = preds[s];
        };
        path.push_back(start);

        std::reverse(path.begin(), path.end());
        summary.termination = Planner<SPACE,STATE>::Termination::SUCCESS;
    } else if(summary.termination == Planner<SPACE,STATE>::Termination::TIMEOUT) {
        summary.total_cost = 0.0;
    } else {
        summary.termination = Planner<SPACE,STATE>::Termination::UNREACHABLE;
        summary.total_cost = 0.0;
    }
    summary.elapsed_usec = check_timer();

    return path;
}

} // namespace nanoplan

#endif // NANOPLAN_DIJKSTRA_H
