#ifndef UTILS_H_
#define UTILS_H_

#include <limits>

namespace nanoplan {

constexpr double NANOPLAN_EPSILON = 1e-7;
constexpr double INF_DBL = std::numeric_limits<double>::infinity();
constexpr float INF_FLT = std::numeric_limits<float>::infinity();
constexpr int INF_INT = std::numeric_limits<int>::infinity();

}  // namespace nanoplan

#endif  // UTILS_H_
