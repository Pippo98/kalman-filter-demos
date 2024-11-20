#pragma once

#include "simulator/simulator.hpp"
#include "utils/time_base.hpp"
#include "utils/type_and_name.hpp"

class Cart1D : public Simulatable {
 public:
  Cart1D(double x0, double u0) {
    x = x0;
    u = u0;
  }
  virtual void step(double dt) final {
    t = t + dt;
    x = x + u * dt;
    // u = u;
  }

  std::vector<Real> getValues() const { return {t, x, u}; }
  std::vector<Real> getValuesWithNoise() const {
    return {t, x.getWithNoise(), u.getWithNoise()};
  }

 private:
  REAL_TYPE(t, 0.0);
  REAL_TYPE(x, 0.1);
  REAL_TYPE(u, 0.01);
};
