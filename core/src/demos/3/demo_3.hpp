#pragma once

#include "external/kflib/src/ukf.hpp"
#include "models/cart_2d.hpp"
#include "simulator/simulation_manager.hpp"
#include "utils/kf_utils.hpp"
#include "utils/utils.hpp"

class Demo3 : public Simulatable {
 public:
  Demo3();

  virtual void step(double dt [[maybe_unused]]) final {};
  void draw(SimulationData &cart);

 private:
  std::vector<Real *> getValuesPtr() override { return std::vector<Real *>(); }

  void setupKF();
  void runKF(SimulationData &sim);

  bool ukfSimulated;
  size_t lastSimCount = 0;
  int updateEvery = 0;
  KFData kf;
  KFData kf2;

  double obstructionCoords[2];
  bool obstructedView = false;
};
