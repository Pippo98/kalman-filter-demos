#pragma once

#include "defines.hpp"
#include "simulator/simulator.hpp"
#include "utils/type_and_name.hpp"

class Cart2D : public Simulatable {
 public:
  Cart2D() {
    vx = 1.0;
    y = 5.0;
    alpha = 10.0 * CONST_TO_RAD;
    planeInclinationX = 10.0;
  }
  virtual void step(double dt) final {
    t = t + dt;
    double ax = 0.0;
    double ay = 0.0;
    if (x.value > planeInclinationX) {
      double Vg = std::sqrt(vx * vx + vy * vy);
      vx = Vg * std::cos(alpha);
      vy = -Vg * std::sin(alpha);

      ax = -CONST_G * std::sin(alpha) * std::cos(alpha);
      ay = +CONST_G * std::sin(alpha) * std::sin(alpha);
      vx = vx + ax * dt;
      vy = vy + ay * dt;
    }
    x = x + vx * dt + ax * dt * dt / 2.0;
    y = y + vy * dt + ay * dt * dt / 2.0;
  }

  double alpha;
  double planeInclinationX;

 private:
  std::vector<Real *> getValuesPtr() override { return {&t, &x, &y, &vx, &vy}; }

  REAL_TYPE(t, 0.0);
  REAL_TYPE(x, 0.1);
  REAL_TYPE(y, 0.1);
  REAL_TYPE(vx, 0.01);
  REAL_TYPE(vy, 0.01);
};
