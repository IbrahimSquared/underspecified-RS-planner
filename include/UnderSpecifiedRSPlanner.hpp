#ifndef UNDERSPECIFIED_RS_PLANNER_HPP
#define UNDERSPECIFIED_RS_PLANNER_HPP

#include <complex>
#include <numbers>

using cdouble = std::complex<double>;

class UnderSpecifiedRSPlanner {
public:
  UnderSpecifiedRSPlanner() = default;
  void getOmega(const double x0, const double y0, const double xn,
                const double yn, const double r, double &omega) const;
  inline void case1(const double x0, const double y0, const double xn,
                    const double yn, const double r, double &omega) const;
  inline void case2(const double x0, const double y0, const double xn,
                    const double yn, const double r, double &omega) const;
  inline void case3(const double x0, const double y0, const double xn,
                    const double yn, const double r, double &gamma) const;

private:
  inline const double get2norm(const double x1, const double y1,
                               const double x2, const double y2) const;
  // Constant expressions
  static constexpr double theta0 = std::numbers::pi / 2;
  static constexpr cdouble i = cdouble(0, 1.0);

  // static constexpr cdouble expThetaI = cdouble(0, 1.0);
  // static constexpr cdouble expTheta2I = cdouble(-1, 0);
  // static constexpr cdouble expHalfThetaI = cdouble(1 / sqrt(2), 1 / sqrt(2));
  // static constexpr cdouble exp3HalfThetaI = cdouble(-1 / sqrt(2), 1 /
  // sqrt(2));
};

#endif
