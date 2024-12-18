#include "demo_1.hpp"
#include <memory>
#include "imgui.h"
#include "imgui_internal.h"
#include "implot.h"
#include "utils/csv.hpp"

Demo1::Demo1() {
  {
    kfPosOnly.initializeMatrices({"x"}, {"x"});
    kfPosOnly.X0(0) = -10.0;
    kfPosOnly.P0(0, 0) = 1e1;
    kfPosOnly.Q(0, 0) = 0.1;
    kfPosOnly.R(0, 0) = 0.5;
  }
  {
    kfPosAndSpeed.initializeMatrices({"x", "vx"}, {"x"});
    kfPosAndSpeed.X0(0) = -10.0;
    kfPosAndSpeed.X0(1) = 0.0;
    kfPosAndSpeed.P0(0, 0) = 1e1;
    kfPosAndSpeed.P0(1, 1) = 1e1;
    kfPosAndSpeed.Q(0, 0) = 0.01;
    kfPosAndSpeed.Q(1, 1) = 0.001;
    kfPosAndSpeed.R(0, 0) = 0.5;
  }
}
void Demo1::setupKF() {
  {
    auto &ukfPositionOnly = kfPosOnly.ukf;
    kfPosOnly.setMatrices();
    ukfPositionOnly.setStateUpdateFunction(
        [](const Eigen::VectorXd &state, const Eigen::VectorXd &input,
           void *userData) -> Eigen::VectorXd {
          (void)input;
          (void)userData;
          // double dt = *(double *)userData;
          auto newState = state;
          newState(0) = state(0);
          return newState;
        });
    ukfPositionOnly.setMeasurementFunction(
        [](const Eigen::VectorXd &state, const Eigen::VectorXd &input,
           void *userData) -> Eigen::VectorXd {
          (void)input;
          (void)userData;
          Eigen::VectorXd measures(1);
          measures(0) = state(0);
          return measures;
        });
  }
  {
    auto &ukfPositionAndSpeed = kfPosAndSpeed.ukf;
    kfPosAndSpeed.setMatrices();
    ukfPositionAndSpeed.setStateUpdateFunction(
        [](const Eigen::VectorXd &state, const Eigen::VectorXd &input,
           void *userData) -> Eigen::VectorXd {
          (void)input;
          double dt = *(double *)userData;
          auto newState = state;
          newState(0) = state(0) + state(1) * dt;
          newState(1) = state(1);
          return newState;
        });
    ukfPositionAndSpeed.setMeasurementFunction(
        [](const Eigen::VectorXd &state, const Eigen::VectorXd &input,
           void *userData) -> Eigen::VectorXd {
          (void)input;
          (void)userData;
          Eigen::VectorXd measures(1);
          measures(0) = state(0);
          return measures;
        });
  }
}

void Demo1::runKF(SimulationData &sim) {
  auto &data = sim.simulation.dataWithNoise;
  size_t rows = data.front().size();

  kfPosOnly.clearData();
  kfPosAndSpeed.clearData();
  for (size_t row = 0; row < rows; row++) {
    {
      kfPosOnly.ukf.predict();
      Eigen::VectorXd measure(1);
      measure(0) = data[1][row].value;
      kfPosOnly.ukf.update(measure);

      kfPosOnly.addStateAndCovariance(data[0][row]);
    }
    {
      double dt = 0.0;
      if (row > 0) {
        dt = data[0][row] - data[0][row - 1];
      }

      kfPosAndSpeed.ukf.setUserData(&dt);
      kfPosAndSpeed.ukf.predict();
      Eigen::VectorXd measure(1);
      measure(0) = data[1][row].value;
      kfPosAndSpeed.ukf.update(measure);

      kfPosAndSpeed.addStateAndCovariance(data[0][row]);
    }
  }
}

void Demo1::draw(SimulationData &sim) {
  if (!ukfSimulated || lastSimCount != sim.simCount) {
    lastSimCount = sim.simCount;
    ukfSimulated = true;
    setupKF();
    runKF(sim);
  }
  bool kfModified = false;

  ImVec2 reg = ImGui::GetContentRegionAvail();

  if (ImGui::BeginTabBar("KF settings")) {
    if (ImGui::BeginTabItem("KF position")) {
      kfModified |= drawKFData(kfPosOnly);
      ImGui::EndTabItem();
    }

    if (ImGui::BeginTabItem("KF position and speed")) {
      kfModified |= drawKFData(kfPosAndSpeed);
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }

  if (kfModified) {
    ukfSimulated = false;
  }

  float rowRatios[2] = {0.75f, 0.25f};
  if (ImPlot::BeginSubplots("Cart 1D", 2, 1, reg, ImPlotSubplotFlags_LinkAllX,
                            rowRatios)) {
    if (ImPlot::BeginPlot("position", ImVec2(-1, 0), ImPlotFlags_NoTitle)) {
      ImPlot::SetupAxes("time", "position");

      ImPlot::SetNextLineStyle({1.0f, 1.0f, 1.0f, 1.0f});
      ImPlot::PlotLine("Real", &sim.simulation.data[0][0].value,
                       &sim.simulation.data[1][0].value,
                       sim.simulation.data.front().size(), 0, 0, sizeof(Real));

      ImPlot::PlotScatter("Measured", &sim.simulation.dataWithNoise[0][0].value,
                          &sim.simulation.dataWithNoise[1][0].value,
                          sim.simulation.dataWithNoise[0].size(), 0, 0,
                          sizeof(Real));

      if (kfPosOnly.hasStates()) {
        ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 2.0);
        ImPlot::PlotLine("x estimated 1", &kfPosOnly.states[0][0].value,
                         &kfPosOnly.states[1][0].value,
                         kfPosOnly.states[0].size(), 0, 0, sizeof(Real));
      }
      if (kfPosAndSpeed.hasStates()) {
        ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 2.0);
        ImPlot::PlotLine("x estimated 2", &kfPosAndSpeed.states[0][0].value,
                         &kfPosAndSpeed.states[1][0].value,
                         kfPosAndSpeed.states[0].size(), 0, 0, sizeof(Real));
      }

      ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("#1D Cart position", ImVec2(-1, 0),
                          ImPlotFlags_NoTitle)) {
      ImPlot::SetupAxes("time", "covariance");

      if (kfPosOnly.hasCov()) {
        ImPlot::PlotLine("x covariance 1", &kfPosOnly.cov[0][0].value,
                         &kfPosOnly.cov[1][0].value, kfPosOnly.cov[0].size(), 0,
                         0, sizeof(Real));
      }
      if (kfPosAndSpeed.hasCov()) {
        ImPlot::PlotLine("x covariance 2", &kfPosAndSpeed.cov[0][0].value,
                         &kfPosAndSpeed.cov[1][0].value,
                         kfPosAndSpeed.cov[0].size(), 0, 0, sizeof(Real));
      }
      ImPlot::EndPlot();
    }
    ImPlot::EndSubplots();
  }
}
