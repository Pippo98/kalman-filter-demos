#pragma once

#include "simulator/simulator.hpp"
#include "utils/time_base.hpp"
#include "utils/type_and_name.hpp"

class Cart1D : public Simulatable {
 public:
  void setIC(double x0, double u0) {
    t = 0.0;
    x = x0;
    u = u0;
  }
  virtual void step(double dt) final {
    t = t + dt;
    x = x + u * dt;
    // u = u;
  }

 private:
  std::vector<Real *> getValuesPtr() override { return {&t, &x, &u}; }

  REAL_TYPE(t, 0.0);
  REAL_TYPE(x, 0.1);
  REAL_TYPE(u, 0.01);
};
