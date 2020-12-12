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
class AStar : public Planner<SPACE, STATE> {
    public:
        AStar(const SPACE& space) : Planner<SPACE, STATE>(space) {}

        std::string planner_name() const override {
            return "AStar";
        }

        std::vector<STATE> plan() {
            this->start_timer();

            PriorityQueue<STATE> pq;
            std::unordered_map<STATE,STATE> preds;
            std::unordered_map<STATE, double> gscores;
            std::unordered_set<STATE> closed;

            pq.push( {this->space.getStart(), 0} );
            preds[this->space.getStart()] = this->space.getStart();
            gscores[this->space.getStart()] = 0.0;

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

                closed.insert(curr_state);
                pq.pop();

                const auto& succs = this->space.getSuccessors(curr_state);

                const auto curr_gscore = gscores[curr_state];
                for(const auto& succ : succs) {
                    const auto& succ_state = succ.first;
                    const auto& succ_cost  = succ.second;

                    if( closed.find(succ_state) == closed.end() ) {
                        double h = this->space.getHeuristic(succ_state);
                        double g = succ_cost + curr_gscore;
                        pq.push( { succ_state, g+h } );
                        gscores[succ_state] = g;
                        preds[succ_state] = curr_state;
                    }
                }
            }

            std::vector<STATE> path;

            if( this->summary.termination == Planner<SPACE,STATE>::Termination::SUCCESS ) {
                STATE s = this->space.getGoal();
                while( !(s == this->space.getStart()) ) {
                    path.push_back(s);
                    s = preds[s];
                };
                path.push_back(this->space.getStart());

                std::reverse(path.begin(), path.end());
                this->summary.total_cost = gscores[this->space.getGoal()];
                this->summary.termination = Planner<SPACE,STATE>::Termination::SUCCESS;
            } else if( this->summary.termination == Planner<SPACE,STATE>::Termination::TIMEOUT ) {
                this->summary.total_cost = 0.0;
            } else {
                this->summary.termination = Planner<SPACE,STATE>::Termination::UNREACHABLE;
            }

            this->summary.elapsed_usec = this->check_timer();
            return path;
        }
};


} // namespace nanoplan

#endif // NANOPLAN_ASTAR_H
