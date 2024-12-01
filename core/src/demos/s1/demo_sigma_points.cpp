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
  drawMatrix("State", state, {"x", "y"}, {reg.x / 2.0f, 0.0f});
  ImGui::SameLine();
  drawMatrix("Covariance", covariance, {"x", "y"}, {reg.x / 2.0f, 0.0f});

  std::vector<ImVec2> sigma;
  MerweScaledSigmaPoints sigmaPoints;
  ukf.computeMerweScaledSigmaPoints(state, covariance, sigmaPoints);
  for (Eigen::Index col = 0; col < sigmaPoints.sigmas.cols(); ++col) {
    sigma.push_back(
        {(float)sigmaPoints.sigmas(0, col), (float)sigmaPoints.sigmas(1, col)});
  }

  static std::vector<Real> temp;
  static std::vector<Real> tempWithNoise;
  static std::vector<Real> tempWithNoise2;
  static std::vector<Real> tempWithNoise3;
  if (temp.size() == 0) {
    Real t("temp", 0.2);
    for (int i = 0; i < 10; i++) {
      t.value = -1.0 + i * 0.1;
      temp.push_back(t);
    }
    for (int i = 0; i < 80; i++) {
      t.value = 0.0;
      temp.push_back(t);
    }
    for (int i = 0; i < 20; i++) {
      t.value = 0.0 + i * 0.2;
      temp.push_back(t);
    }
    for (const auto &t : temp) {
      tempWithNoise.push_back(
          Real(t.name.c_str(), t.getWithNoise(), t.noise.stddev()));
      Real t2(t.name.c_str(), t.value, 1.0);
      tempWithNoise2.push_back(t2.getWithNoise());
      Real t3(t.name.c_str(), t.value, 5.0);
      tempWithNoise3.push_back(t3.getWithNoise());
    }
  }

  if (ImPlot::BeginPlot("Cart position", ImVec2(900, 600))) {
    ImPlot::SetupAxes("time", "position [m]");
    ImPlot::SetupAxisLimits(ImAxis_X1, 10.0, 90.0);

    ImPlot::SetNextLineStyle({1.0, 1.0, 1.0, 1.0}, 3.0);
    ImPlot::PlotLine("True", &temp[0].value, temp.size(), 1.0, 0.0, 0, 0,
                     sizeof(Real));

    ImPlot::SetNextLineStyle(
        ImVec4(52 / 255.0f, 94 / 255.0f, 155 / 255.0f, 1.0));
    ImPlot::SetNextMarkerStyle(
        IMPLOT_AUTO, IMPLOT_AUTO,
        ImVec4(52 / 255.0f, 94 / 255.0f, 155 / 255.0f, 1.0));
    ImPlot::PlotLine("0.2", &tempWithNoise[0].value, temp.size(), 1.0, 0.0, 0,
                     0, sizeof(Real));
    ImPlot::PlotScatter("0.2", &tempWithNoise[0].value, temp.size(), 1.0, 0.0,
                        0, 0, sizeof(Real));

    ImPlot::SetNextLineStyle(ImVec4(204 / 255.0f, .0f, .0f, 1.0));
    ImPlot::SetNextMarkerStyle(IMPLOT_AUTO, IMPLOT_AUTO,
                               ImVec4(204 / 255.0f, .0f, .0f, 1.0));
    ImPlot::PlotLine("1.0", &tempWithNoise2[0].value, temp.size(), 1.0, 0.0, 0,
                     0, sizeof(Real));
    ImPlot::PlotScatter("1.0", &tempWithNoise2[0].value, temp.size(), 1.0, 0.0,
                        0, 0, sizeof(Real));

    ImPlot::SetNextMarkerStyle(IMPLOT_AUTO, IMPLOT_AUTO,
                               ImVec4(200 / 255.0f, 138 / 255.0f, .0f, 1.0));
    ImPlot::SetNextLineStyle(ImVec4(200 / 255.0f, 138 / 255.0f, .0f, 1.0));
    ImPlot::PlotLine("5.0", &tempWithNoise3[0].value, temp.size(), 1.0, 0.0, 0,
                     0, sizeof(Real));
    ImPlot::PlotScatter("5.0", &tempWithNoise3[0].value, temp.size(), 1.0, 0.0,
                        0, 0, sizeof(Real));
    ImPlot::EndPlot();
  }

  // if (ImPlot::BeginPlot("Sigma points sampling", ImVec2(-1, 0),
  //                       ImPlotFlags_Equal)) {
  //   ImPlot::SetupAxes("x", "y");
  //   ImPlot::PlotScatter("points", &sigma[0].x, &sigma[0].y, sigma.size(), 0,
  //   0,
  //                       sizeof(ImVec2));
  //   ImPlot::EndPlot();
  // }
}
