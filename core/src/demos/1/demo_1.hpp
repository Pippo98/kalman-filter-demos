#pragma once

#include "Eigen/Core"
#include "kflib/src/ukf.hpp"
#include "simulator/simulation_manager.hpp"
#include "utils/utils.hpp"

struct KFData {
  UnscentedKalmanFilter ukf;
  std::vector<std::vector<Real>> states;
  std::vector<std::vector<Real>> cov;
  Eigen::VectorXd X0;  // initial state
  Eigen::MatrixXd P0;  // initial state covariance
  Eigen::MatrixXd Q;   // process covariance
  Eigen::MatrixXd R;   // measureent covariance

  void clearData() {
    states.clear();
    cov.clear();
    states.resize(ukf.getState().size() + 1);
    cov.resize(ukf.getState().size() + 1);
  }

  bool hasStates() { return !states.empty() && !states.front().empty(); }
  bool hasCov() { return !cov.empty() && !cov.front().empty(); }

  void initializeMatrices(size_t states, size_t measurements) {
    X0 = Eigen::VectorXd(states);
    P0 = Eigen::MatrixXd(states, states);
    Q = Eigen::MatrixXd(states, states);
    R = Eigen::MatrixXd(measurements, measurements);
    X0.setZero();
    P0.setZero();
    Q.setZero();
    R.setZero();
  }
  void setMatrices() {
    ukf.setState(X0);
    ukf.setStateCovariance(P0);
    ukf.setProcessCovariance(Q);
    ukf.setMeasurementCovariance(R);
  }
};

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
