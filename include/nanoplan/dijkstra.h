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

// -------------------------------------------------------------------------
// INTERFACE
// -------------------------------------------------------------------------

template <typename SPACE, typename STATE>
class Dijkstra : public Planner<SPACE, STATE> {
    public:

        // Construct an algorithm to search over a given search space.
        Dijkstra(const SPACE& space);

        // Search over the search space.
        std::vector<STATE> plan() override;

        std::string planner_name() const override {
            return "Dijkstra";
        }
};

// -------------------------------------------------------------------------
// IMPLEMENTATION
// -------------------------------------------------------------------------

template <typename SPACE, typename STATE>
Dijkstra<SPACE,STATE>::Dijkstra(const SPACE& space) :
    Planner<SPACE, STATE>(space) {}

template <typename SPACE, typename STATE>
std::vector<STATE> Dijkstra<SPACE,STATE>::plan() {
    this->start_timer();

    PriorityQueue<STATE> pq;
    std::unordered_map<STATE,STATE> preds;

    pq.push( {this->space.getStart(), 0} );
    preds[this->space.getStart()] = this->space.getStart();

    while( !pq.empty() ) {
        const auto curr_state = pq.top().first;
        const auto curr_cost  = pq.top().second;
        this->summary.expansions++;

        if( curr_state == this->space.getGoal() ) {
            this->summary.termination = Planner<SPACE,STATE>::Termination::SUCCESS;
            break;
        }

        if( this->options.timeout_ms > 0.0 && this->check_timer() >= 1000*this->options.timeout_ms ) {
            this->summary.termination = Planner<SPACE,STATE>::Termination::TIMEOUT;
            break;
        }

        pq.pop();

        const auto& succs = this->space.getSuccessors(curr_state);

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

    if(this->summary.termination == Planner<SPACE,STATE>::Termination::SUCCESS) {
        this->summary.total_cost = pq.top().second;
        STATE s = this->space.getGoal();
        while( !(s == this->space.getStart()) ) {
            path.push_back(s);
            s = preds[s];
        };
        path.push_back(this->space.getStart());

        std::reverse(path.begin(), path.end());
        this->summary.termination = Planner<SPACE,STATE>::Termination::SUCCESS;
    } else if(this->summary.termination == Planner<SPACE,STATE>::Termination::TIMEOUT) {
        this->summary.total_cost = 0.0;
    } else {
        this->summary.termination = Planner<SPACE,STATE>::Termination::UNREACHABLE;
        this->summary.total_cost = 0.0;
    }
    this->summary.elapsed_usec = this->check_timer();

    return path;
}

} // namespace nanoplan

#endif // NANOPLAN_DIJKSTRA_H
