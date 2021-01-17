#ifndef NANOPLAN_COST_H
#define NANOPLAN_COST_H

namespace nanoplan {

class Cost {
 public:
  static const int FP_MULT = 256;

  static Cost max() {
    Cost c;
    c.value = std::numeric_limits<int>::max() / FP_MULT;
    return c;
  }

  Cost() : value(0) {}
  Cost(const double c) : value(FP_MULT * c) {}
  Cost(const float c) : value(FP_MULT * c) {}
  Cost(const int c) : value(FP_MULT * c) {}

  double as_double() const { return value / static_cast<double>(FP_MULT); }
  float as_float() const { return value / static_cast<float>(FP_MULT); }
  int as_int() const { return value / FP_MULT; }

  bool operator<(const Cost& rhs) const { return value < rhs.value; }
  bool operator<=(const Cost& rhs) const { return value <= rhs.value; }
  bool operator==(const Cost& rhs) const { return value == rhs.value; }
  bool operator!=(const Cost& rhs) const { return value != rhs.value; }
  bool operator>(const Cost& rhs) const { return value > rhs.value; }
  bool operator>=(const Cost& rhs) const { return value >= rhs.value; }

  friend Cost operator+(const Cost& lhs, const Cost& rhs);
  friend Cost operator-(const Cost& lhs, const Cost& rhs);

 private:
  int value = 0;
};

inline Cost operator+(const Cost& lhs, const Cost& rhs) {
  Cost c;
  c.value = lhs.value + rhs.value;
  return c;
}

inline Cost operator-(const Cost& lhs, const Cost& rhs) {
  Cost c;
  c.value = lhs.value - rhs.value;
  return c;
}

}  // namespace nanoplan
#endif  // NANOPLAN_COST_H
