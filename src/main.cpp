#include "nanoplan/nanoplan.h"
#include <fmt/format.h>
#include <vector>

struct State2D {
    long int x;
    long int y;

    bool operator==(const State2D& rhs) const { return x==rhs.x && y==rhs.y; }
};
NANOPLAN_MAKE_STATE_HASHABLE(State2D, s.x, s.y);

class SearchSpace2D final : public nanoplan::SearchSpace<State2D> {
    public:
        std::vector<std::pair<State2D, double>>
        get_successors(const State2D& state) override {
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
        get_predecessors(const State2D& state) override {
            return get_successors(state);
        }

        double get_from_to_heuristic(const State2D& from, const State2D& to) override {
            return (from.x-to.x)*(from.x-to.x) +
                   (from.y-to.y)*(from.y-to.y);
        }
};


int main(int argc, char** argv) {
    SearchSpace2D space2d;
    State2D start { 0, 0 };
    State2D goal { 401, 401 };

    nanoplan::Options options;
    options.timeout_ms = 10000.0;

    {
        nanoplan::Dijkstra<SearchSpace2D> planner(space2d);
        planner.set_start(start);
        planner.set_goal(goal);
        planner.set_options(options);

        const auto plan = planner.plan();
        fmt::print(planner.full_report());
        fmt::print("\n");
    }

    {
        nanoplan::AStar<SearchSpace2D> planner(space2d);
        planner.set_start(start);
        planner.set_goal(goal);
        planner.set_options(options);

        const auto plan = planner.plan();
        fmt::print(planner.full_report());
        fmt::print("\n");
    }

    {
        nanoplan::LPAStar<SearchSpace2D> planner(space2d);
        planner.set_start(start);
        planner.set_goal(goal);
        planner.set_options(options);

        const auto plan = planner.plan();
        fmt::print(planner.full_report());
        fmt::print("\n");
    }

    return 0;
}
