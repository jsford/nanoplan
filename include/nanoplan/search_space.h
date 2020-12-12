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

        void setStart(const STATE& start) { this->start = start; }
        void  setGoal(const STATE& goal) { this->goal = goal; }

        STATE& getStart() { return start; }
        STATE& getGoal() { return goal; }
        const STATE& getStart() const { return start; }
        const STATE& getGoal() const { return goal; }

        virtual std::vector<std::pair<STATE, double>>
        getSuccessors(const STATE& state) = 0;

        virtual std::vector<std::pair<STATE, double>>
        getPredecessors(const STATE& state) {
            throw std::logic_error(
                "NANOPLAN ERROR [NOT IMPLEMENTED]: "
                "State space does not implement required function getPredecessors.");
        }

        virtual double getHeuristic(const STATE& state) {
            throw std::logic_error(
                "NANOPLAN ERROR [NOT IMPLEMENTED]: "
                "State space does not implement required function getHeuristic.");
        }

    protected:
        STATE start, goal;
};

} // namespace nanoplan

#endif // NANOPLAN_SEARCH_SPACE_H
