#pragma once

#include "external/kflib/src/ukf.hpp"
#include "models/cart_2d.hpp"
#include "simulator/simulation_manager.hpp"
#include "utils/kf_utils.hpp"
#include "utils/utils.hpp"

class Demo2 : public Simulatable {
 public:
  Demo2();

  virtual void step(double dt [[maybe_unused]]) final {};
  void draw(SimulationData &cart);

 private:
  std::vector<Real *> getValuesPtr() override { return std::vector<Real *>(); }

  void runKF(SimulationData &sim);
  void setupKF();

  bool ukfSimulated;
  size_t lastSimCount = 0;
  int updateEvery = 0;
  KFData kfPosOnly;
  KFData kfPosAndSpeed;
  KFData kfPosSpeedAccel;
};
