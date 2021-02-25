#include <nanoplan/nanoplan.h>
#include <fmt/format.h>


struct State3D {
  int x;
  int y;
  int z;

  bool operator==(const State3D& rhs) const {
    return x == rhs.x && y == rhs.y && z == rhs.z;
  }
};
NANOPLAN_MAKE_STATE_HASHABLE(State3D, s.x, s.y, s.z);


class SearchSpace2D final : public nanoplan::SearchSpace<State3D> {
  public:
    SearchSpace2D() = default;

    std::vector<State3D> get_successors(const State3D& state) override {
      std::vector<State3D> succs;
      for(int i=-1; i<2; ++i) {
        for(int j=-1; j<2; ++j) {
          for(int k=-1; k<2; ++k) {
            State3D succ {state.x+i, state.y+j, state.z+k};
            if(0 <= succ.x && succ.x < 10 && 0 <= succ.y && succ.y < 10 && 0 <= succ.z && succ.z < 10) {
              succs.push_back(succ);
            }
          }
        }
      }
      return succs;
    }

    std::vector<State3D> get_predecessors(const State3D& state) override {
      return get_successors(state);
    }

    nanoplan::Cost get_from_to_cost(const State3D& from, const State3D& to) override {
      double manhattan = std::abs(from.x-to.x) + std::abs(from.y-to.y) + std::abs(from.z-to.z);
      return nanoplan::Cost(manhattan);
    }

    nanoplan::Cost get_from_to_heuristic(const State3D& from, const State3D& to) override {
      double euclidean = std::sqrt( (from.x-to.x)*(from.x-to.x)
                                  + (from.y-to.y)*(from.y-to.y)
                                  + (from.z-to.z)*(from.z-to.z));
      return nanoplan::Cost(euclidean);
    }
};


int main(int argc, char** argv) {

  auto space2d = std::make_shared<SearchSpace2D>();
  nanoplan::AStar<SearchSpace2D> planner(space2d);

  State3D start {0, 0, 0};
  State3D  goal {9, 9, 9};

  std::vector<State3D> path = planner.plan(start, goal);

  for(int i=0; i<path.size(); ++i) {
    fmt::print("({}, {}, {})", path[i].x, path[i].y, path[i].z);
    if( i != path.size()-1 ) { fmt::print("->"); }
  }
  fmt::print("\n");

  return 0;
}
