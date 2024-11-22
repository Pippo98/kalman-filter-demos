#pragma once

#include "simulator/simulator.hpp"
#include "utils/type_and_name.hpp"

class Cart1D : public Simulatable {
 public:
  Cart1D() {
    t = 0.0;
    x = 0.0;
    vx = 1.0;
  }
  virtual void step(double dt) final {
    t = t + dt;
    x = x + vx * dt;
  }

 private:
  std::vector<Real *> getValuesPtr() override { return {&t, &x, &vx}; }

  REAL_TYPE(t, 0.0);
  REAL_TYPE(x, 0.1);
  REAL_TYPE(vx, 0.01);
};
