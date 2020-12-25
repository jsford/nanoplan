#include "nanoplan/nanoplan.h"
#include <fmt/format.h>
#include <cmath>
#include <vector>

struct State2D {
    long int x;
    long int y;

    bool operator==(const State2D& rhs) const { return x==rhs.x && y==rhs.y; }
};
NANOPLAN_MAKE_STATE_HASHABLE(State2D, s.x, s.y);

class SearchSpace2D final : public nanoplan::SearchSpace<State2D> {
    public:
        const int w = 1000;
        const int h = 1000;

        std::vector<State2D>
        get_successors(const State2D& state) override {
            std::vector<State2D> succs;
            if( state.y+1 < h )
            succs.push_back( { state.x, state.y+1 } );
            if( state.x+1 < w )
            succs.push_back( { state.x+1, state.y } );
            if( state.x-1 >= 0 )
            succs.push_back( { state.x-1, state.y } );
            if( state.y-1 >= 0 )
            succs.push_back( { state.x, state.y-1 } );

            if( state.x+1 < w && state.y+1 < h )
            succs.push_back( { state.x+1, state.y+1 } );
            if( state.x-1 >= 0 && state.y+1 < h )
            succs.push_back( { state.x-1, state.y+1 } );
            if( state.x-1 >= 0 && state.y-1 >= 0 )
            succs.push_back( { state.x-1, state.y-1 } );
            if( state.x+1 < w && state.y-1 >= 0 )
            succs.push_back( { state.x+1, state.y-1 } );


            return succs;
        }

        std::vector<State2D>
        get_predecessors(const State2D& state) override {
            return get_successors(state);
        }

        double get_from_to_cost(const State2D& from, const State2D& to) override {
            return std::sqrt((to.x-from.x)*(to.x-from.x)+(to.y-from.y)*(to.y-from.y));
        }

        double get_from_to_heuristic(const State2D& from, const State2D& to) override {
            return get_from_to_cost(from, to);
        }
};


int main(int argc, char** argv) {
    std::shared_ptr<SearchSpace2D> space2d = std::make_shared<SearchSpace2D>();

    State2D start { 0, 0 };
    State2D goal { 999, 999 };

    nanoplan::Options options;
    options.timeout_ms = 10000.0;

    {
        nanoplan::Dijkstra<SearchSpace2D> planner(space2d);
        planner.set_options(options);

        const auto plan = planner.plan(start, goal);
        fmt::print(planner.full_report());
        fmt::print("\n");
    }

    {
        nanoplan::AStar<SearchSpace2D> planner(space2d);
        planner.set_options(options);

        const auto plan = planner.plan(start, goal);
        fmt::print(planner.full_report());
        fmt::print("\n");
    }

    /*
    {
        nanoplan::LPAStar<SearchSpace2D> planner(space2d);
        planner.set_options(options);

        const auto plan0 = planner.plan(start, goal);
        fmt::print(planner.full_report());
        fmt::print("\n");

        const auto plan1 = planner.replan( State2D {10,10} );

        fmt::print(planner.full_report());
        fmt::print("\n");
    }
    */

    return 0;
}
