#pragma once

#include "defines.hpp"
#include "simulator/simulator.hpp"
#include "utils/type_and_name.hpp"

class Cart2D : public Simulatable {
 public:
  Cart2D(double x0, double y0, double u0) {
    x = x0;
    y = y0;
    u = u0;
    w = 0.0;
    alpha = 10.0 * CONST_TO_RAD;
  }
  virtual void step(double dt) final {
    t = t + dt;
    if (x > 10) {
      u = u - (CONST_G * std::sin(alpha) * std::cos(alpha) * dt);
      w = w + (CONST_G * std::sin(alpha) * std::sin(alpha) * dt);
    }
    x = x + u * dt;
    y = y + w * dt;
  }

  std::vector<Real> getValues() const { return {t, x, y, u, w}; }
  std::vector<Real> getValuesWithNoise() const {
    return {t, x.getWithNoise(), y.getWithNoise(), u.getWithNoise(),
            w.getWithNoise()};
  }

  double alpha;

 private:
  REAL_TYPE(t, 0.0);
  REAL_TYPE(x, 0.1);
  REAL_TYPE(y, 0.1);
  REAL_TYPE(u, 0.01);
  REAL_TYPE(w, 0.01);
};
