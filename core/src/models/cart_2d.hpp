#pragma once

#include "defines.hpp"
#include "simulator/simulator.hpp"
#include "utils/type_and_name.hpp"

class Cart2D : public Simulatable {
 public:
  void setIC(double x0, double y0, double u0) {
    t = 0.0;
    x = x0;
    y = y0;
    u = u0;
    w = 0.0;
    alpha = 10.0 * CONST_TO_RAD;
  }
  virtual void step(double dt) final {
    t = t + dt;
    double ax = 0.0;
    double ay = 0.0;
    if (x.value > 10) {
      ax = -CONST_G * std::sin(alpha) * std::cos(alpha);
      ay = +CONST_G * std::sin(alpha) * std::sin(alpha);
      u = u + ax * dt;
      w = w + ay * dt;
    }
    x = x + u * dt + ax * dt * dt / 2.0;
    y = y + w * dt + ay * dt * dt / 2.0;
  }

  double alpha;

 private:
  std::vector<Real *> getValuesPtr() override { return {&t, &x, &y, &u, &w}; }

  REAL_TYPE(t, 0.0);
  REAL_TYPE(x, 0.1);
  REAL_TYPE(y, 0.1);
  REAL_TYPE(u, 0.01);
  REAL_TYPE(w, 0.01);
};
