#include "demo_1.hpp"
#include <memory>
#include "imgui.h"
#include "implot.h"
#include "utils/csv.hpp"

Demo1::Demo1() {}
void Demo1::setupKF() {
  {
    Eigen::VectorXd state(1);
    Eigen::MatrixXd stateCovariance(1, 1);
    Eigen::MatrixXd processCovariance(1, 1);
    Eigen::MatrixXd measurementCovariance(1, 1);
    state(0) = -10.0;
    stateCovariance(0, 0) = 1e1;
    processCovariance(0, 0) = 0.1;
    measurementCovariance(0, 0) = 0.5;
    ukfPositionOnly.setState(state);
    ukfPositionOnly.setStateCovariance(stateCovariance);
    ukfPositionOnly.setProcessCovariance(processCovariance);
    ukfPositionOnly.setMeasurementCovariance(measurementCovariance);
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
    Eigen::VectorXd state(2);
    Eigen::MatrixXd stateCovariance(2, 2);
    Eigen::MatrixXd processCovariance(2, 2);
    Eigen::MatrixXd measurementCovariance(1, 1);
    state(0) = -10.0;
    state(1) = 0.0;
    stateCovariance.setZero();
    stateCovariance(0, 0) = 1e1;
    stateCovariance(1, 1) = 1e1;
    processCovariance.setZero();
    processCovariance(0, 0) = 0.01;
    processCovariance(1, 1) = 0.001;
    measurementCovariance(0, 0) = 0.5;
    ukfPositionAndSpeed.setState(state);
    ukfPositionAndSpeed.setStateCovariance(stateCovariance);
    ukfPositionAndSpeed.setProcessCovariance(processCovariance);
    ukfPositionAndSpeed.setMeasurementCovariance(measurementCovariance);
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

  ukfPositionOnlyStates.clear();
  ukfPositionOnlyStates.resize(2);
  ukfPositionAndSpeedStates.clear();
  ukfPositionAndSpeedStates.resize(3);
  for (size_t row = 0; row < rows; row++) {
    {
      ukfPositionOnly.predict();
      Eigen::VectorXd measure(1);
      measure(0) = data[1][row].value;
      ukfPositionOnly.update(measure);
      auto state = ukfPositionOnly.getState();
      ukfPositionOnlyStates[0].push_back(data[0][row].value);
      ukfPositionOnlyStates[1].push_back(state(0));
    }
    {
      double dt = 0.0;
      if (row > 0) {
        dt = data[0][row] - data[0][row - 1];
      }
      ukfPositionAndSpeed.setUserData(&dt);
      ukfPositionAndSpeed.predict();
      Eigen::VectorXd measure(1);
      measure(0) = data[1][row].value;
      ukfPositionAndSpeed.update(measure);
      auto state = ukfPositionAndSpeed.getState();
      ukfPositionAndSpeedStates[0].push_back(data[0][row].value);
      ukfPositionAndSpeedStates[1].push_back(state(0));
      ukfPositionAndSpeedStates[2].push_back(state(1));
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
  ImVec2 reg = ImGui::GetContentRegionAvail();
  if (!sim.simulation.data.empty() &&
      ImPlot::BeginPlot("1D Cart position", reg)) {
    ImPlot::SetupAxes("time", "position", ImPlotAxisFlags_AutoFit,
                      ImPlotAxisFlags_AutoFit);

    ImPlot::PlotLine("x", &sim.simulation.data[0].front().value,
                     &sim.simulation.data[1].front().value,
                     sim.simulation.data.front().size(), 0, 0, sizeof(Real));

    ImPlot::PlotScatter(
        "x measured", &sim.simulation.dataWithNoise[0].front().value,
        &sim.simulation.dataWithNoise[1].front().value,
        sim.simulation.dataWithNoise.front().size(), 0, 0, sizeof(Real));

    if (!ukfPositionOnlyStates.empty() &&
        !ukfPositionOnlyStates.front().empty()) {
      ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 2.0);
      ImPlot::PlotLine("x estimated 1", &ukfPositionOnlyStates[0].front(),
                       &ukfPositionOnlyStates[1].front(),
                       ukfPositionOnlyStates.front().size(), 0, 0,
                       sizeof(double));
    }
    if (!ukfPositionAndSpeedStates.empty() &&
        !ukfPositionAndSpeedStates.front().empty()) {
      ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 2.0);
      ImPlot::PlotLine("x estimated 2", &ukfPositionAndSpeedStates[0].front(),
                       &ukfPositionAndSpeedStates[1].front(),
                       ukfPositionAndSpeedStates.front().size(), 0, 0,
                       sizeof(double));
    }

    ImPlot::EndPlot();
  }
}
