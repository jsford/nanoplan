#ifndef NANOPLAN_ASTAR_H
#define NANOPLAN_ASTAR_H

#include "search_space.h"
#include "planner.h"
#include "priority_queue.h"
#include "ext/flat_hash_map.hpp"
#include <memory>
#include <vector>
#include <algorithm>

#include <fmt/format.h>

namespace nanoplan {

template <typename SPACE>
class AStar final : public Planner<SPACE> {
    public:
        AStar(std::shared_ptr<SPACE> space);

        std::string planner_name() const override;

        std::vector<typename SPACE::state_type>
        plan(const typename SPACE::state_type& start,
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
std::vector<typename SPACE::state_type>
AStar<SPACE>::plan(const typename SPACE::state_type& start,
                   const typename SPACE::state_type& goal)
{
    using STATE = typename SPACE::state_type;
    start_timer();

    this->start = start;
    this->goal = goal;

    PriorityQueue<STATE> open;
    ska::flat_hash_map<STATE,STATE> preds;
    ska::flat_hash_map<STATE,double> gscores;
    ska::flat_hash_set<STATE> closed;

    const double h0 = space->get_from_to_heuristic(start, goal);
    open.push( {start, h0} );
    preds[start] = start;
    gscores[start] = 0.0;

    while( !open.empty() ) {
        const STATE curr_state = open.top().first;

        open.pop();
        if( closed.find(curr_state) != closed.end() ) {
            continue;
        }
        closed.insert(curr_state);
        summary.expansions++;

        if( curr_state == goal ) {
            summary.termination = Termination::SUCCESS;
            break;
        }

        if( options.timeout_ms > 0.0 && check_timer() >= 1000*options.timeout_ms ) {
            summary.termination = Termination::TIMEOUT;
            break;
        }

        const auto& succs = space->get_successors(curr_state);

        const auto curr_gscore = gscores[curr_state];
        for(const auto& succ : succs) {
            const double succ_cost = space->get_from_to_cost(curr_state, succ);
            const double tentative_g = curr_gscore + succ_cost;

            if( gscores.find(succ) == gscores.end() || tentative_g < gscores.at(succ) ) {
                double h = space->get_from_to_heuristic(succ, goal);
                open.push( { succ, tentative_g+h } );
                preds[succ] = curr_state;
                gscores[succ] = tentative_g;
            }
        }
    }

    std::vector<STATE> path;
    if( summary.termination == Termination::SUCCESS )
    {
        STATE s = goal;
        while( !(s == start) ) {
            path.push_back(s);
            s = preds[s];
        };
        path.push_back(start);

        std::reverse(path.begin(), path.end());
        summary.total_cost = gscores[goal];
        summary.termination = Termination::SUCCESS;
    }
    else if( summary.termination == Termination::TIMEOUT )
    {
        summary.total_cost = 0.0;
    }
    else
    {
        summary.termination = Termination::UNREACHABLE;
        summary.total_cost = 0.0;
    }

    summary.elapsed_usec = check_timer();
    return path;
}

} // namespace nanoplan

#endif // NANOPLAN_ASTAR_H
