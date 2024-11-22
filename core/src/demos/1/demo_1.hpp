#pragma once

#include "kflib/src/ukf.hpp"
#include "simulator/simulation_manager.hpp"
#include "utils/utils.hpp"

class Demo1 : public Simulatable {
 public:
  Demo1();

  virtual void step(double dt [[maybe_unused]]) final {};

  void draw(SimulationData &cart);

 private:
  std::vector<Real *> getValuesPtr() override { return std::vector<Real *>(); }

  void runKF(SimulationData &sim);

  bool ukf1Simulated;
  UnscentedKalmanFilter ukf1;
  std::vector<std::vector<double>> ukf1States;
};
