#ifndef NANOPLAN_PLANNER_H
#define NANOPLAN_PLANNER_H

#include "search_space.h"
#include <chrono>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

namespace nanoplan {

template <typename SPACE, typename STATE>
class Planner {
    public:
        Planner(const SPACE& search_space) : space(search_space) {}

        virtual std::vector<STATE> plan() = 0;

        virtual std::vector<STATE> replan() {
            return plan();
        }

        virtual std::string planner_name() const = 0;

        void set_timeout_ms(const double ms) { options.timeout_ms = ms; }

        std::string full_report() {
            std::ostringstream report;
            report << "NANOPLAN " << NANOPLAN_VERSION << " Report\n"
                   << "-----------------------------------\n"
                   << std::setw(20) << std::left << "Planner:"    << planner_name() << '\n'
                   << std::setw(20) << std::left << "Cost:"       << summary.total_cost << '\n' 
                   << std::setw(20) << std::left << "Time:"       << summary.elapsed_usec / 1e3 << " ms\n"
                   << std::setw(20) << std::left << "Expansions:" << summary.expansions << '\n' 
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

    protected:
        SPACE space;

        struct Options {
            double timeout_ms = 0.0;
        };
        enum class Termination {
            SUCCESS,
            UNREACHABLE,
            TIMEOUT,
            TERMINATION_NOT_SET
        };

        struct Summary {
            double elapsed_usec = -1.0;
            double total_cost = -1.0;
            unsigned long long expansions = 0;
            Termination termination = Termination::TERMINATION_NOT_SET;
        };

        Options options;
        Summary summary;

        void start_timer() {
            start_time = std::chrono::high_resolution_clock::now();
        }

        double check_timer() {
            using namespace std::chrono;
            const auto end_time = high_resolution_clock::now();
            return duration_cast<microseconds>(end_time-start_time).count();
        }

    private:
        std::chrono::high_resolution_clock::time_point start_time;

};

} // namespace nanoplan

#endif // NANOPLAN_PLANNER_H
