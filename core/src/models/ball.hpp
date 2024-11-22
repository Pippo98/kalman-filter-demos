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
    double Vg = std::sqrt(vx * vx + vy * vy);
    double viscous = c * Vg;
    double aero = Cd * Vg * Vg;
    vx = vx.value + (viscous * dt + aero * dt) * (vx > 0 ? -1 : 1);
    vy = vy + CONST_G * dt + (viscous * dt + aero * dt) * (vy > 0 ? -1 : 1);
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
