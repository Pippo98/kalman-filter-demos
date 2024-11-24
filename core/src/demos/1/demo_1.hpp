#pragma once

#include "Eigen/Core"
#include "simulator/simulation_manager.hpp"
#include "utils/kf_utils.hpp"
#include "utils/utils.hpp"

class Demo1 : public Simulatable {
 public:
  Demo1();

  virtual void step(double dt [[maybe_unused]]) final {};

  void draw(SimulationData &cart);

 private:
  std::vector<Real *> getValuesPtr() override { return std::vector<Real *>(); }

  void runKF(SimulationData &sim);
  void setupKF();

  bool ukfSimulated;
  size_t lastSimCount = 0;
  KFData kfPosOnly;
  KFData kfPosAndSpeed;
};
