#ifndef NANOPLAN_ASTAR_H
#define NANOPLAN_ASTAR_H

#include "search_space.h"
#include "planner.h"
#include "priority_queue.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <algorithm>

#include <fmt/format.h>

namespace nanoplan {

template <typename SPACE, typename STATE>
class AStar final : public Planner<SPACE, STATE> {
    public:
        AStar(const SPACE& space);

        std::string planner_name() const override;

        std::vector<STATE> plan() override;

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
AStar<SPACE,STATE>::AStar(const SPACE& space) : Planner<SPACE, STATE>(space) {}

template <typename SPACE, typename STATE>
std::string AStar<SPACE,STATE>::planner_name() const {
    return "AStar";
}

template <typename SPACE, typename STATE>
std::vector<STATE> AStar<SPACE,STATE>::plan() {
    start_timer();

    PriorityQueue<STATE> pq;
    std::unordered_map<STATE,STATE> preds;
    std::unordered_map<STATE, double> gscores;
    std::unordered_set<STATE> closed;

    pq.push( {start, 0} );
    preds[start] = start;
    gscores[start] = 0.0;

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

    std::vector<STATE> path;

    if( summary.termination == Planner<SPACE,STATE>::Termination::SUCCESS ) {
        STATE s = goal;
        while( !(s == start) ) {
            path.push_back(s);
            s = preds[s];
        };
        path.push_back(start);

        std::reverse(path.begin(), path.end());
        summary.total_cost = gscores[goal];
        summary.termination = Planner<SPACE,STATE>::Termination::SUCCESS;
    } else if( summary.termination == Planner<SPACE,STATE>::Termination::TIMEOUT ) {
        summary.total_cost = 0.0;
    } else {
        summary.termination = Planner<SPACE,STATE>::Termination::UNREACHABLE;
    }

    summary.elapsed_usec = check_timer();
    return path;
}

} // namespace nanoplan

#endif // NANOPLAN_ASTAR_H
