#pragma once

#include "external/kflib/src/ukf.hpp"
#include "models/cart_2d.hpp"
#include "simulator/simulation_manager.hpp"
#include "utils/kf_utils.hpp"
#include "utils/utils.hpp"

class Demo4 : public Simulatable {
 public:
  Demo4();

  virtual void step(double dt [[maybe_unused]]) final {};
  void draw(SimulationData &cart);

 private:
  std::vector<Real *> getValuesPtr() override { return std::vector<Real *>(); }

  bool runKF(SimulationData &sim, int iterations = -1);
  void setupKF();

  bool ukfSimulated;
  size_t lastSimCount = 0;
  int updateEvery = 0;
  KFData kfPosOnly;
  KFData kfPosAndSpeed;
  KFData kfPosSpeedAccel;
  KFData smoother;
};
