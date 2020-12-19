#ifndef NANOPLAN_ASTAR_H
#define NANOPLAN_ASTAR_H

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
class AStar final : public Planner<SPACE> {
    public:
        AStar(const SPACE& space);

        std::string planner_name() const override;

        std::vector<typename SPACE::state_type> plan() override;

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
AStar<SPACE>::AStar(const SPACE& space) : Planner<SPACE>(space) {}

template <typename SPACE>
std::string AStar<SPACE>::planner_name() const {
    return "AStar";
}

template <typename SPACE>
std::vector<typename SPACE::state_type> AStar<SPACE>::plan() {
    using STATE = typename SPACE::state_type;

    start_timer();

    struct BookkeepingData {
        STATE pred;
        double gscore;
    };

    PriorityQueue<STATE> pq;
    ska::flat_hash_map<STATE,BookkeepingData> bookkeeping;
    ska::flat_hash_set<STATE> closed;

    pq.push( {start, 0} );
    bookkeeping[start] = { start, 0.0 };

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

        const auto curr_gscore = bookkeeping[curr_state].gscore;
        for(const auto& succ : succs) {
            const auto& succ_state = succ.first;
            const auto& succ_cost  = succ.second;

            if( closed.find(succ_state) == closed.end() ) {
                double h = space.get_from_to_heuristic(succ_state, goal);
                double g = succ_cost + curr_gscore;
                pq.push( { succ_state, g+h } );
                bookkeeping[succ_state] = { curr_state, g };
            }
        }
    }

    std::vector<STATE> path;

    if( summary.termination == Termination::SUCCESS ) {
        STATE s = goal;
        while( !(s == start) ) {
            path.push_back(s);
            s = bookkeeping[s].pred;
        };
        path.push_back(start);

        std::reverse(path.begin(), path.end());
        summary.total_cost = bookkeeping[goal].gscore;
        summary.termination = Termination::SUCCESS;
    } else if( summary.termination == Termination::TIMEOUT ) {
        summary.total_cost = 0.0;
    } else {
        summary.termination = Termination::UNREACHABLE;
    }

    summary.elapsed_usec = check_timer();
    return path;
}

} // namespace nanoplan

#endif // NANOPLAN_ASTAR_H
