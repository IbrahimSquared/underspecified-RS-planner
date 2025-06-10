#include "UnderSpecifiedRSPlanner.hpp"
#include <cmath>

using namespace std::numbers;

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void UnderSpecifiedRSPlanner::getOmega(const double x0, const double y0,
                                       const double xf, const double yf,
                                       const double r, const double theta,
                                       double &omega, const bool offset) const {
  // rotate to the local frame
  double xn, yn;
  if (offset) {
    xn = (xf - x0) * cos(theta - pi / 2) + (yf - y0) * sin(theta - pi / 2) + x0;
    yn =
        -(xf - x0) * sin(theta - pi / 2) + (yf - y0) * cos(theta - pi / 2) + y0;
  } else {
    xn = (xf - x0) * cos(theta) + (yf - y0) * sin(theta) + x0;
    yn = -(xf - x0) * sin(theta) + (yf - y0) * cos(theta) + y0;
  }

  // centers of left and right side circles
  // cxr0 = x0 + r * cos(theta0) with theta0 fixed pi/2 implicit, etc.
  const double cxr0 = x0 + r;
  const double cyr0 = y0;
  const double cxl0 = x0 - r;
  const double cyl0 = y0;

  const double d1 = get2norm(xn, yn, cxr0, cyr0);
  const double d2 = get2norm(xn, yn, cxl0, cyl0);

  if (xn >= x0 && yn >= y0) {
    // Q1
    if ((d1 >= r) && ((yn >= r + y0) | (xn - cxr0 < 0))) {
      case1(x0, y0, xn, yn, r, omega);
      omega = theta0 - omega;
    } else if ((d2 >= sqrt(5) * r) && ((yn < r + y0) && (xn - cxr0 > 0))) {
      case2(x0, y0, xn, yn, r, omega);
      omega = theta0 - (pi - omega);
    } else if ((d2 < sqrt(5) * r) && (d1 <= r)) {
      double gamma;
      case3(x0, y0, xn, yn, r, gamma);

      const double px = cxl0 + 2 * r * cos(gamma);
      const double py = cyl0 - 2 * r * sin(gamma);

      omega = theta0 - (pi - atan2(yn - py, xn - px));
    }
    // Q2
  } else if (xn <= x0 && yn >= y0) {
    if ((d2 >= r) && ((yn >= r + y0) | (xn - cxl0 > 0))) {
      case1(xn, y0, x0, yn, r, omega);
      omega = theta0 + omega;
    } else if ((d1 >= sqrt(5) * r) && ((yn <= r + y0) && (xn - cxl0 < 0))) {
      case2(xn, y0, x0, yn, r, omega);
      omega = theta0 + (pi - omega);
    } else if ((d1 <= sqrt(5) * r) && (d2 <= r)) {
      double gamma;
      case3(xn, y0, x0, yn, r, gamma);

      const double px = cxr0 + 2 * r * cos(pi - gamma);
      const double py = cyr0 - 2 * r * sin(pi - gamma);

      omega = theta0 + atan2(yn - py, xn - px);
    }
    // Q3
  } else if (xn <= x0 && yn <= y0) {
    if ((d2 >= r) && ((yn <= y0 - r) | (xn - cxl0 > 0))) {
      case1(xn, yn, x0, y0, r, omega);
      omega = theta0 - omega;
    } else if ((d1 >= sqrt(5) * r) && (yn >= y0 - r && xn - cxl0 < 0)) {
      case2(xn, yn, x0, y0, r, omega);
      omega = theta0 - (pi - omega);
    } else if ((d1 <= sqrt(5) * r) && (d2 <= r)) {

      double gamma;
      case3(xn, yn, x0, y0, r, gamma);

      const double px = cxr0 + 2 * r * cos(pi - gamma);
      const double py = cyr0 + 2 * r * sin(pi - gamma);

      omega = theta0 + atan2(yn - py, xn - px);
    }
    // Q4
  } else if (xn >= x0 && yn <= y0) {
    if ((d1 >= r) && ((yn <= y0 - r) | (xn - cxr0 < 0))) {
      case1(x0, yn, xn, y0, r, omega);
      omega = theta0 + omega;
    } else if ((d2 >= sqrt(5) * r) && (yn >= y0 - r && xn - cxr0 > 0)) {
      case2(x0, yn, xn, y0, r, omega);
      omega = theta0 + (pi - omega);
    } else if ((d2 <= sqrt(5) * r) && (d1 <= r)) {
      double gamma;
      case3(x0, yn, xn, y0, r, gamma);

      const double px = cxl0 + 2 * r * cos(gamma);
      const double py = cyl0 + 2 * r * sin(gamma);
      omega = theta0 + (pi + atan2(yn - py, xn - px));
    }
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline void UnderSpecifiedRSPlanner::case1(const double x0, const double y0,
                                           const double xn, const double yn,
                                           const double r,
                                           double &omega) const {
  const double dx = xn - x0;
  const double dy = yn - y0;
  const cdouble a = r * exp(0.5 * i * theta0);
  const cdouble b = -i * r * (dx + i * dy);
  const cdouble c = exp(i * theta0) * (dx * dx + dy * dy);
  const cdouble d = i * r * exp(2.0 * i * theta0) * (dx - i * dy);
  const cdouble e = i * (r * exp(theta0 * 3.0 * i / 2.0)) +
                    exp(theta0 * i / 2.0) * (dx + i * dy);
  omega = (-log((-a + sqrt(b + c + d) * i) / e) * i).real();
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline void UnderSpecifiedRSPlanner::case2(const double x0, const double y0,
                                           const double xn, const double yn,
                                           const double r,
                                           double &omega) const {
  const double dx = xn - x0;
  const double dy = yn - y0;
  const cdouble a = exp(0.5 * i * theta0);
  const cdouble b = i * r * (dx - dy);
  const cdouble c = exp(theta0 * i) * (dx * dx + dy * dy);
  const cdouble d = -i * r * exp(2.0 * i * theta0) * (dx + dy);
  const cdouble e = -r * exp(theta0 * i) * i;
  const cdouble f = (r - exp(theta0 * i) * (i * dx + dy));
  const cdouble g = -log((a * sqrt(b + c + d) + e) / f) * i;
  omega = g.real();
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline void UnderSpecifiedRSPlanner::case3(const double x0, const double y0,
                                           const double xn, const double yn,
                                           const double r,
                                           double &gamma) const {
  const double dx = xn - x0;
  const double dy = yn - y0;

  const cdouble a = -0.25 * r * (dx + i * dy);
  const cdouble b1 = (2 * r * r - 2 * r * exp(theta0 * i) * (i * dx + dy));
  const cdouble b = cdouble(std::round(b1.real()), std::round(b1.imag()));
  const cdouble c1 = 2 * r * r * exp(theta0 * 2 * i);
  const cdouble c2 = 2 * r * exp(theta0 * i) * (i * dx - dy);
  const cdouble c = cdouble(std::round(c1.real() + c2.real()),
                            std::round(c1.imag() + c2.imag()));
  const cdouble d = -r * (dx + i * dy);
  const cdouble e = r * r * exp(theta0 * i) * 4.0 * i;
  const cdouble f = i * exp(theta0 * i) * (dx * dx + dy * dy);
  const cdouble g = r * exp(theta0 * 2 * i) * (dx - i * dy);
  const cdouble h = r * r * exp(theta0 * i) * i;
  const cdouble j = 0.25 * i * exp(theta0 * i) * (dx * dx + dy * dy);
  const cdouble k = 0.25 * r * exp(theta0 * 2 * i) * (dx - i * dy);
  const cdouble l1 = r * r * exp(theta0 * 2 * i);
  const cdouble l2 = r * exp(theta0 * i) * (i * dx - dy);
  const cdouble l = l1 + l2;

  const cdouble m1 = 4.0 * b * c;
  const cdouble m2 = (d + e + f + g) * (d + e + f + g);
  const cdouble m3 = h + j + k;

  gamma = (-log((a + 0.25 * sqrt(m1 + m2) + m3) / l) * i).real();
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline double UnderSpecifiedRSPlanner::get2norm(const double x1,
                                                const double y1,
                                                const double x2,
                                                const double y2) const {
  return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
