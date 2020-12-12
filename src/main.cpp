#include "nanoplan/nanoplan.h"
#include <fmt/format.h>
#include <vector>

struct State2D {
    int x;
    int y;

    bool operator==(const State2D& rhs) const { return x==rhs.x && y==rhs.y; }
};
NANOPLAN_MAKE_STATE_HASHABLE(State2D, s.x, s.y);

class SearchSpace2D : public nanoplan::SearchSpace<State2D> {
    public:
        std::vector<std::pair<State2D, double>>
        getSuccessors(const State2D& state) override {
            std::vector<std::pair<State2D, double>> succs;
            succs.push_back( { { state.x, state.y+1 }, 1.0} );
            succs.push_back( { { state.x+1, state.y }, 1.0} );
            succs.push_back( { { state.x-1, state.y }, 1.0} );
            succs.push_back( { { state.x, state.y-1 }, 1.0} );

            if( state.x+state.y % 2) {
                succs.push_back( { { state.x+1, state.y+1 }, 1.0001} );
                succs.push_back( { { state.x-1, state.y+1 }, 1.0001} );
                succs.push_back( { { state.x+1, state.y-1 }, 1.0001} );
                succs.push_back( { { state.x-1, state.y-1 }, 1.0001} );
            }
            return succs;
        }

        std::vector<std::pair<State2D, double>>
            getPredecessors(const State2D& state) override {
            return getSuccessors(state);
        }

        double getHeuristic(const State2D& state) override {
            return (state.x-this->goal.x)*(state.x-this->goal.x) +
                   (state.y-this->goal.y)*(state.y-this->goal.y);
        }
};


int main(int argc, char** argv) {
    using namespace nanoplan;

    SearchSpace2D space2d;
    State2D start { 0, 0 };
    State2D goal { 101, 101 };

    {
        space2d.setStart( start );
        space2d.setGoal( goal );
        AStar<SearchSpace2D, State2D> astar(space2d);
        astar.set_timeout_ms(1000.0);

        const auto plan = astar.plan();
        fmt::print(astar.full_report()+'\n');
    }

    {
        space2d.setStart( start );
        space2d.setGoal( goal );
        Dijkstra<SearchSpace2D, State2D> dijkstra(space2d);
        dijkstra.set_timeout_ms(5000.0);

        const auto plan = dijkstra.plan();
        fmt::print(dijkstra.full_report()+'\n');
    }

    return 0;
}
