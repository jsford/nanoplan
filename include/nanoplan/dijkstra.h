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
        Dijkstra(std::shared_ptr<SPACE> space);

        // Search over the search space.
        std::vector<typename SPACE::state_type>
        plan(const typename SPACE::state_type& start,
             const typename SPACE::state_type& goal) override;

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
Dijkstra<SPACE>::Dijkstra(std::shared_ptr<SPACE> space) :
    Planner<SPACE>(space) {}

template <typename SPACE>
std::vector<typename SPACE::state_type>
Dijkstra<SPACE>::plan(const typename SPACE::state_type& start,
                      const typename SPACE::state_type& goal)
{
    using STATE = typename SPACE::state_type;
    start_timer();

    this->start = start;
    this->goal = goal;

    PriorityQueue<STATE> pq;
    ska::flat_hash_map<STATE,STATE> preds;
    ska::flat_hash_map<STATE,double> gscores;
    ska::flat_hash_set<STATE> closed;

    pq.push( {start, 0.0} );
    preds[start] = start;
    gscores[start] = 0.0;

    while( !pq.empty() ) {
        const STATE curr_state = pq.top().first;

        pq.pop();
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
                pq.push( { succ, tentative_g } );
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

#endif // NANOPLAN_DIJKSTRA_H
