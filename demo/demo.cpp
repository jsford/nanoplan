#include <fmt/color.h>
#include <fmt/format.h>
#include <nanoplan/nanoplan.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <random>
#include <thread>

#include "argh.h"
#include "terminal.h"

using namespace nanoplan;

struct State2D {
  int x;
  int y;

  bool operator==(const State2D& rhs) const { return x == rhs.x && y == rhs.y; }
};
NANOPLAN_MAKE_STATE_HASHABLE(State2D, s.x, s.y);

class SearchSpace2D final : public SearchSpace<State2D> {
 public:
  int w;
  int h;
  std::vector<double> costmap;

  SearchSpace2D(const std::vector<double>& costs, int w, int h)
      : costmap(costs), w(w), h(h) {}

  std::vector<State2D> get_successors(const State2D& state) override {
    std::vector<State2D> succs;
    succs.reserve(4);
    succs.push_back({state.x, state.y + 1});
    succs.push_back({state.x + 1, state.y});
    succs.push_back({state.x - 1, state.y});
    succs.push_back({state.x, state.y - 1});

    // Uncomment for 8-connected grid.
    // succs.push_back({state.x + 1, state.y + 1});
    // succs.push_back({state.x + 1, state.y - 1});
    // succs.push_back({state.x - 1, state.y - 1});
    // succs.push_back({state.x - 1, state.y + 1});

    auto out_of_bounds = [this](const State2D& succ) {
      return 0 > succ.x || succ.x >= w || 0 > succ.y || succ.y >= h;
    };
    succs.erase(std::remove_if(succs.begin(), succs.end(), out_of_bounds),
                succs.end());
    return succs;
  }

  std::vector<State2D> get_predecessors(const State2D& state) override {
    return get_successors(state);
  }

  Cost get_from_to_cost(const State2D& from, const State2D& to) override {
    if (costmap[to.y * w + to.x] > threshold) {
      return Cost::max();
    }
    return std::sqrt((to.x - from.x) * (to.x - from.x) +
                     (to.y - from.y) * (to.y - from.y));
  }

  Cost get_from_to_heuristic(const State2D& from, const State2D& to) override {
    return get_from_to_cost(from, to);
  }

  std::vector<State2D> get_changed_states() override {
    std::vector<State2D> states;
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        if (costmap[y * w + x] > threshold &&
            costmap[y * w + x] <= prev_threshold) {
          states.push_back(State2D{x, y});
        }
      }
    }
    return states;
  }

  void render(const std::vector<State2D>& path, const State2D& start,
              const State2D& goal) {
    cls();

    fmt::print("\n +{:->{}}+\n", "", w);
    for (int i = 0; i < h; ++i) {
      fmt::print(" |");
      for (int j = 0; j < w; ++j) {
        auto state = State2D{j, i};

        if (costmap[i * w + j] > prev_threshold) {
          // Old obstacles.
          fmt::print("{}", "\u2588");
        } else if (costmap[i * w + j] > threshold) {
          // New obstacles.
          fmt::print(fmt::fg(fmt::color::yellow), "{}", "\u2588");
        } else if (state == start) {
          // Start.
          fmt::print(fmt::fg(fmt::color::green), "\u2588");
        } else if (state == goal) {
          // Goal.
          fmt::print(fmt::fg(fmt::color::red), "\u2588");
        } else if (std::find(path.begin(), path.end(), state) != path.end()) {
          // Path.
          fmt::print(fmt::fg(fmt::color::blue), "\u2588");
        } else {
          // Free Space.
          fmt::print(" ");
        }
      }
      fmt::print("|\n");
    }
    fmt::print(" +{:->{}}+\n", "", w);
  }

  void add_obstacles() {
    prev_threshold = threshold;
    threshold *= 0.998;
  }

 private:
  double threshold = 1.0;
  double prev_threshold = 10.0;
};

void fill_random(std::vector<double>& v, double lo, double hi) {
  for (auto& x : v) {
    x = rand() / (double)RAND_MAX * (hi - lo) + lo;
  }
}

int main(int argc, char** argv) {
  argh::parser cmdl(argc, argv);
  std::string planner_name = cmdl(1, "astar").str();
  const bool move_robot = cmdl[{"-m", "--move"}];

  int w;
  int h;
  get_terminal_size(w, h);
  w -= 4;
  h -= 5;

  State2D start{0, 0};
  State2D goal{w - 1, h - 1};

  std::vector<double> costs(w * h);
  fill_random(costs, 0.0, 1.0);
  costs[start.y * w + start.x] = 0.0;
  costs[goal.y * w + goal.x] = 0.0;

  auto space2d = std::make_shared<SearchSpace2D>(costs, w, h);

  std::unique_ptr<Planner<SearchSpace2D>> planner;

  if (planner_name == "lpastar") {
    planner.reset(new LPAStar<SearchSpace2D>(space2d));
  } else if (planner_name == "dstar") {
    planner.reset(new DStar<SearchSpace2D>(space2d));
  } else if (planner_name == "astar") {
    planner.reset(new AStar<SearchSpace2D>(space2d));
  } else if (planner_name == "dijkstra") {
    planner.reset(new Dijkstra<SearchSpace2D>(space2d));
  } else {
    fmt::print("{} is not a valid planner option.\n", argv[1]);
    fmt::print("usage: ./demo_nanoplan <planner>\n");
    fmt::print("available planners:\n");
    fmt::print("  \"dijkstra\"\n");
    fmt::print("  \"astar\"\n");
    fmt::print("  \"lpastar\"\n");
    fmt::print("  \"dstar\"\n");
    return 0;
  }

  auto path = planner->plan(start, goal);
  space2d->render(path, start, goal);
  auto summary = planner->get_summary();

  int i = 0;
  hidecursor();
  do {
    space2d->add_obstacles();
    path = planner->replan(start);
    space2d->render(path, start, goal);
    summary = planner->get_summary();
    fmt::print(
        " PLANNER: {} ITER: {} COST: {:0.3f} TIME: {} [ms] EXPANSIONS: {} "
        "RATE: {:0.3f} [kHz]\n",
        planner->planner_name(), i++, summary.total_cost.as_double(),
        summary.elapsed_usec / 1000.0, summary.expansions,
        summary.expansions / summary.elapsed_usec * (1e3));

    if (start == goal) {
      break;
    }
    if (move_robot && path.size() >= 2) {
      start = path[1];
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } while (summary.termination == Termination::SUCCESS);
  showcursor();

  return 0;
}
