#include "terminal.h"
#include <nanoplan/nanoplan.h>
#include <fmt/format.h>
#include <fmt/color.h>

#include <unistd.h>
#include <chrono>
#include <memory>
#include <algorithm>
#include <random>

using namespace nanoplan;

struct State2D {
  int x;
  int y;

  bool operator==(const State2D& rhs) const { return x == rhs.x && y == rhs.y; }
};
NANOPLAN_MAKE_STATE_HASHABLE(State2D, s.x, s.y);

class SearchSpace2D final : public SearchSpace<State2D> {
 public:
   int w; int h;
   double threshold = 1.0;
   std::vector<double> costmap;

   SearchSpace2D(const std::vector<double>& costs, int w, int h) : costmap(costs), w(w), h(h) {}

  std::vector<State2D> get_successors(const State2D& state) override {
    std::vector<State2D> succs; succs.reserve(8);
    succs.push_back({state.x, state.y + 1});
    succs.push_back({state.x + 1, state.y});
    succs.push_back({state.x - 1, state.y});
    succs.push_back({state.x, state.y - 1});

    succs.push_back({state.x + 1, state.y + 1});
    succs.push_back({state.x + 1, state.y - 1});
    succs.push_back({state.x - 1, state.y - 1});
    succs.push_back({state.x - 1, state.y + 1});

    auto out_of_bounds_or_obstacle = [this](const State2D& succ) {
      return 0 > succ.x || succ.x >= w || 0 > succ.y || succ.y >= h ||
        costmap[succ.y*w+succ.x] > threshold;
    };
    succs.erase(std::remove_if(
          succs.begin(), succs.end(),
          out_of_bounds_or_obstacle), succs.end());
    return succs;
  }

  std::vector<State2D> get_predecessors(const State2D& state) override {
    return get_successors(state);
  }

  double get_from_to_cost(const State2D& from, const State2D& to) override {
    return std::sqrt((to.x-from.x)*(to.x-from.x) + (to.y-from.y)*(to.y-from.y));
  }

  double get_from_to_heuristic(const State2D& from,
                               const State2D& to) override {
    return get_from_to_cost(from, to);
  }

  std::vector<State2D> get_changed_states() override {
    std::vector<State2D> states;
    states.push_back(State2D{4000, 4000});
    return states;
  }

  void render(const std::vector<State2D>& path) {
    cls();

    fmt::print("\n +{:->{}}+\n", "", w);
    for(int i=0; i<h; ++i) {
      fmt::print(" |");
      for(int j=0; j<w; ++j) {
        auto state = State2D {j, i};

        if(costmap[i*w+j] > threshold) {
          fmt::print("{}", "\u2588");
        } else if(std::find(path.begin(), path.end(), state) != path.end()) {
          if( state == path[path.size()-1] ) {
            fmt::print(fmt::fg(fmt::color::red), "\u2588");
          }
          else if( state == path[0] ) {
            fmt::print(fmt::fg(fmt::color::green), "\u2588");
          } else {
            fmt::print(fmt::fg(fmt::color::blue), "\u2588");
          }
        } else {
          fmt::print(" ");
        }
      }
      fmt::print("|\n");
    }
    fmt::print(" +{:->{}}+\n", "", w);

  }
};

void fill_random(std::vector<double>& v, double lo, double hi) {
  for(auto& x : v) {
    x = rand()/(double)RAND_MAX * (hi-lo) + lo;
  }
}


int main(int argc, char** argv) {
  int w; int h;
  get_terminal_size(w, h);
  w -= 4;
  h -= 4;

  std::vector<double> costs(w*h);
  fill_random(costs, 0.0, 1.0);
  costs[0] = 0.0;
  costs[costs.size()-1] = 0.0;

  auto space2d = std::make_shared<SearchSpace2D>(costs, w, h);

  State2D start {0, 0};
  State2D  goal {w-1, h-1};

  AStar<SearchSpace2D> planner(space2d);
  auto path = planner.plan(start, goal);

  hidecursor();
  do {
    space2d->render(path);
    space2d->threshold *= 0.999;
    path = planner.replan();
    usleep(1e5);
  } while( planner.get_summary().termination == Termination::SUCCESS);
  showcursor();

  return 0;
}

