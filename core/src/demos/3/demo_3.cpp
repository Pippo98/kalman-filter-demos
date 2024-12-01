#include "demo_3.hpp"

#include <iostream>
#include <memory>
#include "defines.hpp"
#include "imgui.h"
#include "implot.h"
#include "models/cart_2d.hpp"
#include "simulator/simulation_manager.hpp"
#include "utils/csv.hpp"

Demo3::Demo3() {
  obstructionCoords[0] = 10;
  obstructionCoords[1] = 20;
  {
    kf.initializeMatrices({"x", "y", "vx", "vy", "c", "Cd"}, {"x", "y"});
    kf.X0(0) = 0.0;
    kf.X0(1) = 0.0;
    kf.X0(2) = 0.0;
    kf.X0(3) = 0.0;
    kf.X0(4) = 0.0;
    kf.X0(5) = 0.0;
    kf.P0(0, 0) = 1e1;
    kf.P0(1, 1) = 1e1;
    kf.P0(2, 2) = 1e2;
    kf.P0(3, 3) = 1e2;
    kf.P0(4, 4) = 0.01;
    kf.P0(5, 5) = 0.01;
    kf.Q(0, 0) = 0.01;
    kf.Q(1, 1) = 0.01;
    kf.Q(2, 2) = 0.01;
    kf.Q(3, 3) = 0.01;
    kf.Q(4, 4) = 0.000001;
    kf.Q(5, 5) = 0.000001;
    kf.R(0, 0) = 0.1;
    kf.R(1, 1) = 0.1;
    kf.ukf.setMerweScaledSigmaPointsParams(0.05, 2.0, -1.0);
  }
}
void Demo3::setupKF() {
  {
    auto &ukf = kf.ukf;
    kf.setMatrices();

    ukf.setStateConstraintsFunction([](const Eigen::VectorXd &state,
                                       const Eigen::VectorXd &input,
                                       void *userData) -> Eigen::VectorXd {
      (void)input;
      (void)userData;
      auto constrainedState = state;
      auto constr = [](double value) {
        if (value < 0.0) {
          return value * 0.7;
        }
        return value * 1.0;
      };
      constrainedState(4) = constr(state(4));
      constrainedState(5) = constr(state(5));
      // constrainedState(4) = std::max(state(4), 0.0);
      // constrainedState(5) = std::max(state(5), 0.0);
      // constrainedState(4) = state(4);
      // constrainedState(5) = state(5);
      return constrainedState;
    });
    ukf.setStateUpdateFunction([](const Eigen::VectorXd &state,
                                  const Eigen::VectorXd &input,
                                  void *userData) -> Eigen::VectorXd {
      (void)input;
      (void)userData;
      double dt = *(double *)userData;
      auto newState = state;

      newState(0) = state(0) + dt * state(2);
      newState(1) = state(1) + dt * state(3);
      newState(2) = state(2) + dt * (-state(4) * state(2) -
                                     state(5) * std::abs(state(2)) * state(2));
      newState(3) = state(3) +
                    dt * (-state(4) * state(3) -
                          state(5) * std::abs(state(3)) * state(3)) +
                    dt * CONST_G;
      newState(4) = state(4);
      newState(5) = state(5);

      return newState;
    });
    ukf.setMeasurementFunction([](const Eigen::VectorXd &state,
                                  const Eigen::VectorXd &input,
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
void Demo3::runKF(SimulationData &sim) {
  auto &data = sim.simulation.dataWithNoise;
  size_t rows = data.front().size();

  kf.clearData();
  for (size_t row = 0; row < rows; row++) {
    {
      double dt = 0.0;
      if (row > 0) {
        dt = data[0][row] - data[0][row - 1];
      }
      kf.ukf.setUserData(&dt);

      kf.ukf.predict();

      bool obstructed =
          obstructedView && (data[1][row] > obstructionCoords[0] &&
                             data[1][row] < obstructionCoords[1]);

      Eigen::VectorXd measure(2);
      measure(0) = data[1][row].value;
      measure(1) = data[2][row].value;
      if ((updateEvery == 0 || row % updateEvery == 0) && !obstructed) {
        kf.ukf.update(measure);
      }

      kf.addStateAndCovariance(data[0][row]);
    }
    kf.calculateResiduals("x", data[1]);
    kf.calculateResiduals("y", data[2]);
  }
}
void Demo3::draw(SimulationData &sim) {
  if (!ukfSimulated || lastSimCount != sim.simCount) {
    lastSimCount = sim.simCount;
    ukfSimulated = true;
    setupKF();
    runKF(sim);
  }
  bool kfModified = false;

  ImVec2 reg = ImGui::GetContentRegionAvail();

  ImGui::SetNextItemWidth(120);
  if (ImGui::InputInt("Update every n samples", &updateEvery, 1, 1)) {
    kfModified = true;
  }
  if (ImGui::BeginTabBar("KF settings")) {
    if (ImGui::BeginTabItem("KF position")) {
      kfModified |= drawKFData(kf);
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }
  if (ImGui::CollapsingHeader("State comparison")) {
    static bool showCov = true;
    static bool showRes = false;
    ImGui::Checkbox("show Cov", &showCov);
    ImGui::SameLine();
    ImGui::Checkbox("show Res", &showRes);
    if (ImPlot::BeginSubplots("States and Covariances", 2, 2, reg,
                              ImPlotSubplotFlags_LinkAllX)) {
      if (ImPlot::BeginPlot("State X")) {
        ImPlot::SetupAxes("time", "position");
        ImPlot::SetNextLineStyle({1.0f, 1.0f, 1.0f, 1.0f});
        ImPlot::PlotLine("real x", &sim.simulation.data[0][0].value,
                         &sim.simulation.data[1][0].value,
                         sim.simulation.data[0].size(), 0, 0, sizeof(Real));

        plotKFState("kf1", "x", kf);
        ImPlot::EndPlot();
      }
      if (ImPlot::BeginPlot("State Y")) {
        ImPlot::SetupAxes("time", "position");
        ImPlot::SetNextLineStyle({1.0f, 1.0f, 1.0f, 1.0f});
        ImPlot::PlotLine("real y", &sim.simulation.data[0][0].value,
                         &sim.simulation.data[2][0].value,
                         sim.simulation.data[0].size(), 0, 0, sizeof(Real));

        plotKFState("kf1", "y", kf);
        ImPlot::EndPlot();
      }
      if (ImPlot::BeginPlot("Covariances X")) {
        ImPlot::SetupAxis(ImAxis_X1, "time");
        if (showCov) {
          ImPlot::SetupAxis(ImAxis_Y1, "cov");
        }
        if (showRes) {
          ImPlot::SetupAxis(ImAxis_Y2, "residuals");
        }

        if (showCov) {
          plotKFCovariance("kf1", "x", kf);
        }

        if (showRes) {
          ImPlot::SetAxis(ImAxis_Y2);
          ImPlot::PlotLine("res kf1.x", &kf.states[0][0].value,
                           &kf.residuals["x"][0].value, kf.states[0].size(), 0,
                           0, sizeof(Real));
        }

        ImPlot::EndPlot();
      }
      if (ImPlot::BeginPlot("Covariances Y")) {
        ImPlot::SetupAxis(ImAxis_X1, "time");
        if (showCov) {
          ImPlot::SetupAxis(ImAxis_Y1, "cov");
        }
        if (showRes) {
          ImPlot::SetupAxis(ImAxis_Y2, "residuals");
        }

        if (showCov) {
          plotKFCovariance("kf1", "y", kf);
        }
        if (showRes) {
          ImPlot::SetAxis(ImAxis_Y2);
          ImPlot::PlotLine("res kf1.y", &kf.states[0][0].value,
                           &kf.residuals["y"][0].value, kf.states[0].size(), 0,
                           0, sizeof(Real));
        }
        ImPlot::EndPlot();
      }

      ImPlot::EndSubplots();
    }
  }

  if (ImGui::Checkbox("Obstructred view", &obstructedView)) {
    kfModified = true;
  }

  if (!sim.simulation.data.empty() &&
      ImPlot::BeginPlot("Ball position", reg, ImPlotFlags_Equal)) {
    ImPlot::SetupAxes("x", "y");

    ImPlot::SetNextLineStyle({1.0f, 1.0f, 1.0f, 1.0f});
    ImPlot::PlotLine("Real", &sim.simulation.data[1].front().value,
                     &sim.simulation.data[2].front().value,
                     sim.simulation.data.front().size(), 0, 0, sizeof(Real));

    ImPlot::PlotScatter(
        "Measured", &sim.simulation.dataWithNoise[1].front().value,
        &sim.simulation.dataWithNoise[2].front().value,
        sim.simulation.dataWithNoise.front().size(), 0, 0, sizeof(Real));

    if (obstructedView) {
      kfModified |=
          ImPlot::DragLineX(1, &obstructionCoords[0], {1.0, 1.0, 1.0, 1.0});
      kfModified |=
          ImPlot::DragLineX(2, &obstructionCoords[1], {1.0, 1.0, 1.0, 1.0});
    }

    if (kf.hasStates()) {
      ImPlot::PlotLine("estimated", &kf.states[1][0].value,
                       &kf.states[2][0].value, kf.states[0].size(), 0, 0,
                       sizeof(Real));
    }

    ImPlot::EndPlot();
  }

  if (kfModified) {
    ukfSimulated = false;
  }
}
