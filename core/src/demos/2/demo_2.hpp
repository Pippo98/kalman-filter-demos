#pragma once

#include "external/kflib/src/ukf.hpp"
#include "models/cart_2d.hpp"
#include "simulator/simulation_manager.hpp"
#include "utils/utils.hpp"

class Demo2 : public Simulatable {
 public:
  Demo2();

  virtual void step(double dt [[maybe_unused]]) final {};
  void draw(SimulationData &cart);

 private:
  std::vector<Real *> getValuesPtr() override { return std::vector<Real *>(); }
};
