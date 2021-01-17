#ifndef NANOPLAN_PLANNER_H
#define NANOPLAN_PLANNER_H

#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "cost.h"
#include "search_space.h"

namespace nanoplan {

enum class Termination {
  SUCCESS,             // The planner found a path from the start to the goal.
  UNREACHABLE,         // The goal is not reachable from the start.
  TIMEOUT,             // The planner exceeded its allotted planning time.
  TERMINATION_NOT_SET  // The planner forgot to set the termination condition.
};

struct Options {
  double timeout_ms = -1.0;
};

struct Summary {
  double elapsed_usec = 0.0;
  Cost total_cost = Cost(0);
  unsigned long long expansions = 0;
  Termination termination = Termination::TERMINATION_NOT_SET;
};

template <typename SPACE>
class Planner {
 public:
  typedef typename SPACE::state_type STATE;

  void set_options(const Options& opts);
  Options get_options() const;

  Planner(std::shared_ptr<SPACE> search_space);

  virtual std::vector<STATE> plan(const STATE& start, const STATE& goal) = 0;
  virtual std::vector<STATE> replan();

  virtual std::string planner_name() const = 0;
  std::string full_report();

  Summary get_summary() const;

 protected:
  const std::shared_ptr<SPACE> space;
  STATE start;
  STATE goal;

  Options options;
  Summary summary;

  void start_timer();
  double check_timer() const;

 private:
  std::chrono::high_resolution_clock::time_point start_time;
};

template <typename SPACE>
void Planner<SPACE>::set_options(const Options& opts) {
  options = opts;
}

template <typename SPACE>
Options Planner<SPACE>::get_options() const {
  return options;
}

template <typename SPACE>
Planner<SPACE>::Planner(std::shared_ptr<SPACE> search_space)
    : space(search_space) {}

template <typename SPACE>
std::vector<typename SPACE::state_type> Planner<SPACE>::replan() {
  return plan(start, goal);
}

template <typename SPACE>
Summary Planner<SPACE>::get_summary() const {
  return summary;
}

template <typename SPACE>
std::string Planner<SPACE>::full_report() {
  std::ostringstream report;
  report << "NANOPLAN " << NANOPLAN_VERSION << " Report\n"
         << "-----------------------------------\n"
         << std::setw(20) << std::left << "Planner:" << planner_name() << '\n'
         << std::setw(20) << std::left << "Cost:" << summary.total_cost << '\n'
         << std::setw(20) << std::left << "Time:" << summary.elapsed_usec / 1e3
         << " ms\n"
         << std::setw(20) << std::left << "Expansions:" << summary.expansions
         << '\n'
         << std::setw(20) << std::left
         << "Expansion Rate:" << std::setprecision(6)
         << summary.expansions / summary.elapsed_usec * 1e3 << " kHz\n"
         << std::setw(20) << std::left << "Termination: ";

  if (summary.termination == Termination::TIMEOUT) {
    report << "TIMEOUT\n";
  } else if (summary.termination == Termination::SUCCESS) {
    report << "SUCCESS\n";
  } else if (summary.termination == Termination::UNREACHABLE) {
    report << "UNREACHABLE\n";
  } else {
    report << "TERMINATION NOT SET\n";
  }
  report << "-----------------------------------\n";
  return report.str();
}

template <typename SPACE>
void Planner<SPACE>::start_timer() {
  start_time = std::chrono::high_resolution_clock::now();
}

template <typename SPACE>
double Planner<SPACE>::check_timer() const {
  using namespace std::chrono;
  const auto end_time = high_resolution_clock::now();
  return duration_cast<microseconds>(end_time - start_time).count();
}

}  // namespace nanoplan

#endif  // NANOPLAN_PLANNER_H
