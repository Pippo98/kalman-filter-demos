#pragma once

#include "defines.hpp"
#include "simulator/simulator.hpp"
#include "utils/type_and_name.hpp"

class Ball : public Simulatable {
 public:
  Ball() {
    x = 0.0;
    y = 0.0;
    vx = 30.0;
    vy = 30.0;
  }
  virtual void step(double dt) final {
    t = t + dt;
    x = x + vx * dt;
    y = y + vy * dt;
    vx = vx.value - (c * vx + Cd * std::abs(vx) * vx) * dt;
    vy = vy + CONST_G * dt - (c * vy + Cd * std::abs(vy) * vy) * dt;
    if (y <= 0) {
      y = 0.0;
      vy = 0.0;
      vx = 0.0;
    }
  }

  double c = 0.0;
  double Cd = 0.0;

 private:
  std::vector<Real *> getValuesPtr() override { return {&t, &x, &y, &vx, &vy}; }

  REAL_TYPE(t, 0.0);
  REAL_TYPE(x, 0.1);
  REAL_TYPE(y, 0.1);
  REAL_TYPE(vx, 0.01);
  REAL_TYPE(vy, 0.01);
};
