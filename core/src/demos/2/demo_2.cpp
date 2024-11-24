#include "demo_2.hpp"

#include <memory>
#include "defines.hpp"
#include "imgui.h"
#include "implot.h"
#include "models/cart_2d.hpp"
#include "simulator/simulation_manager.hpp"
#include "utils/csv.hpp"

Demo2::Demo2() {
  {
    kfPosOnly.initializeMatrices(2, 2);
    kfPosOnly.X0(0) = -10.0;
    kfPosOnly.X0(1) = -10.0;
    kfPosOnly.P0(0, 0) = 1e1;
    kfPosOnly.P0(1, 1) = 1e1;
    kfPosOnly.Q(0, 0) = 0.1;
    kfPosOnly.R(0, 0) = 0.5;
    kfPosOnly.R(1, 1) = 0.5;
  }
  {
    kfPosAndSpeed.initializeMatrices(4, 2);
    kfPosAndSpeed.X0(0) = -10.0;
    kfPosAndSpeed.X0(1) = -10.0;
    kfPosAndSpeed.X0(2) = 0.0;
    kfPosAndSpeed.X0(3) = 0.0;
    kfPosAndSpeed.P0(0, 0) = 1e1;
    kfPosAndSpeed.P0(1, 1) = 1e1;
    kfPosAndSpeed.P0(2, 2) = 1e1;
    kfPosAndSpeed.P0(3, 3) = 1e1;
    kfPosAndSpeed.Q(0, 0) = 0.01;
    kfPosAndSpeed.Q(1, 1) = 0.01;
    kfPosAndSpeed.Q(2, 2) = 0.001;
    kfPosAndSpeed.Q(3, 3) = 0.001;
    kfPosAndSpeed.R(0, 0) = 0.5;
    kfPosAndSpeed.R(1, 1) = 0.5;
  }
  {
    kfPosSpeedAccel.initializeMatrices(4, 2);
    kfPosSpeedAccel.X0(0) = -10.0;
    kfPosSpeedAccel.X0(1) = -10.0;
    kfPosSpeedAccel.X0(2) = 0.0;
    kfPosSpeedAccel.X0(3) = 0.0;
    kfPosSpeedAccel.P0(0, 0) = 1e1;
    kfPosSpeedAccel.P0(1, 1) = 1e1;
    kfPosSpeedAccel.P0(2, 2) = 1e1;
    kfPosSpeedAccel.P0(3, 3) = 0.1;
    kfPosSpeedAccel.Q(0, 0) = 0.01;
    kfPosSpeedAccel.Q(1, 1) = 0.01;
    kfPosSpeedAccel.Q(2, 2) = 0.001;
    kfPosSpeedAccel.Q(3, 3) = 0.001;
    kfPosSpeedAccel.R(0, 0) = 0.5;
    kfPosSpeedAccel.R(1, 1) = 0.5;
  }
}
void Demo2::setupKF() {
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
          newState(1) = state(1);
          return newState;
        });
    ukfPositionOnly.setMeasurementFunction(
        [](const Eigen::VectorXd &state, const Eigen::VectorXd &input,
           void *userData) -> Eigen::VectorXd {
          (void)input;
          (void)userData;
          Eigen::VectorXd measures(2);
          measures(0) = state(0);
          measures(1) = state(1);
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
          newState(0) = state(0) + state(2) * dt;
          newState(1) = state(1) + state(3) * dt;
          newState(2) = state(2);
          newState(3) = state(3);
          return newState;
        });
    ukfPositionAndSpeed.setMeasurementFunction(
        [](const Eigen::VectorXd &state, const Eigen::VectorXd &input,
           void *userData) -> Eigen::VectorXd {
          (void)input;
          (void)userData;
          Eigen::VectorXd measures(2);
          measures(0) = state(0);
          measures(1) = state(1);
          return measures;
        });
  }
  {
    auto &ukfPositionSpeedAccel = kfPosSpeedAccel.ukf;
    kfPosSpeedAccel.setMatrices();
    ukfPositionSpeedAccel.setStateUpdateFunction(
        [](const Eigen::VectorXd &state, const Eigen::VectorXd &input,
           void *userData) -> Eigen::VectorXd {
          (void)input;
          double dt = *(double *)userData;
          auto newState = state;
          newState(0) = state(0) + std::cos(state(3)) * state(2) * dt;
          newState(1) = state(1) + std::sin(state(3)) * state(2) * dt;
          newState(2) = state(2) + CONST_G * std::sin(state(3)) * dt;
          newState(3) = state(3);
          return newState;
        });
    ukfPositionSpeedAccel.setMeasurementFunction(
        [](const Eigen::VectorXd &state, const Eigen::VectorXd &input,
           void *userData) -> Eigen::VectorXd {
          (void)input;
          (void)userData;
          Eigen::VectorXd measures(2);
          measures(0) = state(0);
          measures(1) = state(1);
          return measures;
        });
  }
}
void Demo2::runKF(SimulationData &sim) {
  auto &data = sim.simulation.dataWithNoise;
  size_t rows = data.front().size();

  kfPosOnly.clearData();
  kfPosAndSpeed.clearData();
  kfPosSpeedAccel.clearData();
  for (size_t row = 0; row < rows; row++) {
    {
      kfPosOnly.ukf.predict();
      Eigen::VectorXd measure(2);
      measure(0) = data[1][row].value;
      measure(1) = data[2][row].value;
      kfPosOnly.ukf.update(measure);
      auto state = kfPosOnly.ukf.getState();
      kfPosOnly.states[0].push_back(Real("t", data[0][row].value, 0.0));
      kfPosOnly.states[1].push_back(Real("x", state(0), 0.0));
      kfPosOnly.states[2].push_back(Real("y", state(1), 0.0));

      const auto &cov = kfPosOnly.ukf.getCovariance();
      kfPosOnly.cov[0].push_back(Real("t", data[0][row].value, 0.0));
      kfPosOnly.cov[1].push_back(Real("x-x", cov(0, 0), 0.0));
      kfPosOnly.cov[2].push_back(Real("y-y", cov(1, 1), 0.0));
    }
    {
      double dt = 0.0;
      if (row > 0) {
        dt = data[0][row] - data[0][row - 1];
      }

      kfPosAndSpeed.ukf.setUserData(&dt);
      kfPosAndSpeed.ukf.predict();
      Eigen::VectorXd measure(2);
      measure(0) = data[1][row].value;
      measure(1) = data[2][row].value;
      kfPosAndSpeed.ukf.update(measure);
      const auto &state = kfPosAndSpeed.ukf.getState();
      kfPosAndSpeed.states[0].push_back(Real("t", data[0][row].value, 0.0));
      kfPosAndSpeed.states[1].push_back(Real("x", state(0), 0.0));
      kfPosAndSpeed.states[2].push_back(Real("y", state(1), 0.0));
      kfPosAndSpeed.states[3].push_back(Real("vx", state(2), 0.0));
      kfPosAndSpeed.states[4].push_back(Real("vy", state(3), 0.0));

      const auto &cov = kfPosAndSpeed.ukf.getCovariance();
      kfPosAndSpeed.cov[0].push_back(Real("t", data[0][row], 0.0));
      kfPosAndSpeed.cov[1].push_back(Real("x-x", cov(0, 0), 0.0));
      kfPosAndSpeed.cov[2].push_back(Real("y-y", cov(1, 1), 0.0));
      kfPosAndSpeed.cov[3].push_back(Real("vx-vx", cov(2, 2), 0.0));
      kfPosAndSpeed.cov[4].push_back(Real("vy-vy", cov(3, 3), 0.0));
    }
    {
      double dt = 0.0;
      if (row > 0) {
        dt = data[0][row] - data[0][row - 1];
      }

      kfPosSpeedAccel.ukf.setUserData(&dt);
      kfPosSpeedAccel.ukf.predict();
      Eigen::VectorXd measure(2);
      measure(0) = data[1][row].value;
      measure(1) = data[2][row].value;
      kfPosSpeedAccel.ukf.update(measure);
      const auto &state = kfPosSpeedAccel.ukf.getState();
      kfPosSpeedAccel.states[0].push_back(Real("t", data[0][row].value, 0.0));
      kfPosSpeedAccel.states[1].push_back(Real("x", state(0), 0.0));
      kfPosSpeedAccel.states[2].push_back(Real("y", state(1), 0.0));
      kfPosSpeedAccel.states[3].push_back(Real("vg", state(2), 0.0));
      kfPosSpeedAccel.states[4].push_back(Real("alpha", state(3), 0.0));

      const auto &cov = kfPosSpeedAccel.ukf.getCovariance();
      kfPosSpeedAccel.cov[0].push_back(Real("t", data[0][row], 0.0));
      kfPosSpeedAccel.cov[1].push_back(Real("x-x", cov(0, 0), 0.0));
      kfPosSpeedAccel.cov[2].push_back(Real("y-y", cov(1, 1), 0.0));
      kfPosSpeedAccel.cov[3].push_back(Real("vg-vg", cov(2, 2), 0.0));
      kfPosSpeedAccel.cov[4].push_back(Real("alpha-alpha", cov(3, 3), 0.0));
    }
  }
}
void Demo2::draw(SimulationData &sim) {
  Cart2D &cart = *dynamic_cast<Cart2D *>(sim.simulatable);
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
    if (ImGui::BeginTabItem("KF position speed and acceleration")) {
      kfModified |= drawKFData(kfPosSpeedAccel);
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }

  if (kfModified) {
    ukfSimulated = false;
  }

  reg = ImGui::GetContentRegionAvail();

  float rowRatios[2] = {0.75f, 0.25f};
  if (ImPlot::BeginSubplots("Cart 1D", 2, 1, reg, ImPlotSubplotFlags_LinkAllX,
                            rowRatios)) {
    if (ImPlot::BeginPlot("2D Cart position", reg, ImPlotFlags_Equal)) {
      ImPlot::SetupAxes("x", "y", ImPlotAxisFlags_AutoFit,
                        ImPlotAxisFlags_AutoFit);

      ImPlot::PlotLine("x", &sim.simulation.data[1].front().value,
                       &sim.simulation.data[2].front().value,
                       sim.simulation.data.front().size(), 0, 0, sizeof(Real));

      ImPlot::PlotScatter(
          "x measured", &sim.simulation.dataWithNoise[1].front().value,
          &sim.simulation.dataWithNoise[2].front().value,
          sim.simulation.dataWithNoise.front().size(), 0, 0, sizeof(Real));

      ImVec2 groundPoints[3] = {
          {0.0, 5.0},
          {float(cart.planeInclinationX), 5.0},
          {float(cart.planeInclinationX + 30.0 * std::cos(cart.alpha)),
           float(5.0 - 30 * std::sin(cart.alpha))}};
      ImPlot::PlotLine("ground", &groundPoints[0].x, &groundPoints[0].y, 3, 0,
                       0, sizeof(ImVec2));

      if (kfPosOnly.hasStates()) {
        ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 2.0);
        ImPlot::PlotLine("x estimated 1", &kfPosOnly.states[1][0].value,
                         &kfPosOnly.states[2][0].value,
                         kfPosOnly.states[1].size(), 0, 0, sizeof(Real));
      }
      if (kfPosAndSpeed.hasStates()) {
        ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 2.0);
        ImPlot::PlotLine("x estimated 2", &kfPosAndSpeed.states[1][0].value,
                         &kfPosAndSpeed.states[2][0].value,
                         kfPosAndSpeed.states[0].size(), 0, 0, sizeof(Real));
      }
      if (kfPosSpeedAccel.hasStates()) {
        ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 2.0);
        ImPlot::PlotLine("x estimated 3", &kfPosSpeedAccel.states[1][0].value,
                         &kfPosSpeedAccel.states[2][0].value,
                         kfPosSpeedAccel.states[0].size(), 0, 0, sizeof(Real));
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
      if (kfPosSpeedAccel.hasCov()) {
        ImPlot::PlotLine("x covariance 3", &kfPosSpeedAccel.cov[0][0].value,
                         &kfPosSpeedAccel.cov[1][0].value,
                         kfPosSpeedAccel.cov[0].size(), 0, 0, sizeof(Real));
      }
      ImPlot::EndPlot();
    }
    ImPlot::EndSubplots();
  }
}
