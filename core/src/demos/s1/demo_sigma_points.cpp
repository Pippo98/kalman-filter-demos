#include "demo_sigma_points.hpp"
#include "imgui.h"
#include "implot.h"
#include "kflib/src/ukf.hpp"
#include "utils/kf_utils.hpp"

DemoSigmaPoints::DemoSigmaPoints() {
  kappa = 0.0;
  alpha = 1.0;
  beta = 2.0;
  state = Eigen::VectorXd(2);
  covariance = Eigen::MatrixXd(2, 2);
  state(0) = 1;
  state(1) = 1;
  covariance.setIdentity();
}

void DemoSigmaPoints::draw() {
  ImGui::SetNextItemWidth(150);
  if (ImGui::InputDouble("Alpha", &alpha)) {
    ukf.setMerweScaledSigmaPointsParams(alpha, beta, kappa);
  }
  ImGui::SameLine();
  ImGui::SetNextItemWidth(150);
  if (ImGui::InputDouble("Beta", &beta)) {
    ukf.setMerweScaledSigmaPointsParams(alpha, beta, kappa);
  }
  ImGui::SameLine();
  ImGui::SetNextItemWidth(150);
  if (ImGui::InputDouble("Kappa", &kappa)) {
    ukf.setMerweScaledSigmaPointsParams(alpha, beta, kappa);
  }
  auto reg = ImGui::GetContentRegionAvail();
  drawMatrix("State", state, {reg.x / 2.0f, 0.0f});
  ImGui::SameLine();
  drawMatrix("Covariance", covariance, {reg.x / 2.0f, 0.0f});

  std::vector<ImVec2> sigma;
  MerweScaledSigmaPoints sigmaPoints;
  ukf.computeMerweScaledSigmaPoints(state, covariance, sigmaPoints);
  for (Eigen::Index col = 0; col < sigmaPoints.sigmas.cols(); ++col) {
    sigma.push_back(
        {(float)sigmaPoints.sigmas(0, col), (float)sigmaPoints.sigmas(1, col)});
  }

  if (ImPlot::BeginPlot("Sigma points sampling", ImVec2(-1, 0),
                        ImPlotFlags_Equal)) {
    ImPlot::SetupAxes("x", "y");
    ImPlot::PlotScatter("points", &sigma[0].x, &sigma[0].y, sigma.size(), 0, 0,
                        sizeof(ImVec2));
    ImPlot::EndPlot();
  }
}
