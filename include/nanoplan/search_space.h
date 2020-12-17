#ifndef NANOPLAN_SEARCH_SPACE_H
#define NANOPLAN_SEARCH_SPACE_H

#include <vector>
#include <stdexcept>
#include <utility>

namespace nanoplan {

template <typename STATE>
class SearchSpace {
    public: 
        virtual ~SearchSpace() {}

        virtual std::vector<std::pair<STATE, double>>
        get_successors(const STATE& state) = 0;

        virtual std::vector<std::pair<STATE, double>>
        get_predecessors(const STATE& state) {
            throw std::logic_error(
                "NANOPLAN ERROR [NOT IMPLEMENTED]: "
                "State space does not implement required function getPredecessors.");
        }

        virtual double get_from_to_heuristic(const STATE& from, const STATE& to) {
            throw std::logic_error(
                "NANOPLAN ERROR [NOT IMPLEMENTED]: "
                "State space does not implement required function getHeuristic.");
        }
};

} // namespace nanoplan

#endif // NANOPLAN_SEARCH_SPACE_H
