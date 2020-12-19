#ifndef NANOPLAN_PLANNER_H
#define NANOPLAN_PLANNER_H

#include "search_space.h"
#include <chrono>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

namespace nanoplan {

enum class Termination {
    SUCCESS,                // The planner found a path from the start to the goal.
    UNREACHABLE,            // The goal is not reachable from the start.
    TIMEOUT,                // The planner exceeded its allotted planning time.
    TERMINATION_NOT_SET     // The planning algorithm forgot to set the termination condition.
};

struct Options {
    double timeout_ms = 0.0;
};

struct Summary {
    double elapsed_usec = -1.0;
    double total_cost = -1.0;
    unsigned long long expansions = 0;
    Termination termination = Termination::TERMINATION_NOT_SET;
};


template <typename SPACE>
class Planner {
    public:
        typedef typename SPACE::state_type STATE;

        void set_options(const Options& opts);
        Options get_options() const;

        Planner(const SPACE& search_space);

        virtual std::vector<STATE> plan() = 0;
        virtual std::vector<STATE> replan();
        virtual void update_cost(const STATE& from, const STATE& to, double cost);


        void set_start(const STATE& start);
        void  set_goal(const STATE& goal);

        STATE& get_start();
        STATE& get_goal();
        const STATE& get_start() const;
        const STATE& get_goal() const;

        virtual std::string planner_name() const = 0;
        std::string full_report();

    protected:
        SPACE space;
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
Planner<SPACE>::Planner(const SPACE& search_space) : space(search_space) {}

template <typename SPACE>
std::vector<typename SPACE::state_type> Planner<SPACE>::replan() { return plan(); }

template <typename SPACE>
void Planner<SPACE>::set_start(const STATE& start) { this->start = start; }

template <typename SPACE>
void Planner<SPACE>::set_goal(const STATE& goal) { this->goal = goal; }

template <typename SPACE>
typename SPACE::state_type& Planner<SPACE>::get_start() { return start; }

template <typename SPACE>
typename SPACE::state_type& Planner<SPACE>::get_goal() { return goal; }

template <typename SPACE>
const typename SPACE::state_type& Planner<SPACE>::get_start() const { return start; }

template <typename SPACE>
const typename SPACE::state_type& Planner<SPACE>::get_goal() const { return goal; }

template <typename SPACE>
void Planner<SPACE>::update_cost(const STATE& from, const STATE& to, double cost) {}

template <typename SPACE>
std::string Planner<SPACE>::full_report() {
    std::ostringstream report;
    report << "NANOPLAN " << NANOPLAN_VERSION << " Report\n"
           << "-----------------------------------\n"
           << std::setw(20) << std::left << "Planner:"    << planner_name() << '\n'
           << std::setw(20) << std::left << "Cost:"       << summary.total_cost << '\n' 
           << std::setw(20) << std::left << "Time:"       << summary.elapsed_usec / 1e3 << " ms\n"
           << std::setw(20) << std::left << "Expansions:" << summary.expansions << '\n' 
           << std::setw(20) << std::left << "Expansion Rate:" << std::setprecision(6) << summary.expansions / summary.elapsed_usec * 1e3 << " kHz\n" 
           << std::setw(20) << std::left << "Termination: ";

    if ( summary.termination == Termination::TIMEOUT) {
       report << "TIMEOUT\n";
    } else if ( summary.termination == Termination::SUCCESS) {
       report << "SUCCESS\n";
    } else if ( summary.termination == Termination::UNREACHABLE) {
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
    return duration_cast<microseconds>(end_time-start_time).count();
}

} // namespace nanoplan

#endif // NANOPLAN_PLANNER_H
