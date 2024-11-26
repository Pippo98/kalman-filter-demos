#pragma once

#include "kflib/src/ukf.hpp"

class DemoSigmaPoints {
 public:
  DemoSigmaPoints();
  void draw();

 private:
  UnscentedKalmanFilter ukf;
  double alpha;
  double beta;
  double kappa;

  Eigen::VectorXd state;
  Eigen::MatrixXd covariance;
};
