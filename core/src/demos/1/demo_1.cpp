#include "demo_1.hpp"
#include <memory>
#include "imgui.h"
#include "implot.h"
#include "utils/csv.hpp"

Demo1::Demo1() {
  {
    Eigen::VectorXd state(1);
    Eigen::MatrixXd stateCovariance(1, 1);
    Eigen::MatrixXd processCovariance(1, 1);
    Eigen::MatrixXd measurementCovariance(1, 1);
    state(0) = 0.0;
    stateCovariance(0, 0) = 1e6;
    processCovariance(0, 0) = 0.1;
    measurementCovariance(0, 0) = 0.5;
    ukf1.setState(state);
    ukf1.setStateCovariance(stateCovariance);
    ukf1.setProcessCovariance(processCovariance);
    ukf1.setMeasurementCovariance(measurementCovariance);
    ukf1.setStateUpdateFunction([](const Eigen::VectorXd &state,
                                   const Eigen::VectorXd &input,
                                   void *userData) -> Eigen::VectorXd {
      (void)input;
      (void)userData;
      // double dt = *(double *)userData;
      auto newState = state;
      newState(0) = state(0);
      return newState;
    });
    ukf1.setMeasurementFunction([](const Eigen::VectorXd &state,
                                   const Eigen::VectorXd &input,
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
  ukf1States.clear();
  ukf1States.resize(2);
  for (size_t row = 0; row < rows; row++) {
    {
      ukf1.predict();
      Eigen::VectorXd measure(1);
      measure(0) = data[1][row].value;
      ukf1.update(measure);
      auto state = ukf1.getState();
      ukf1States[0].push_back(data[0][row].value);
      ukf1States[1].push_back(state(0));
    }
  }
}
void Demo1::draw(SimulationData &sim) {
  if (!ukf1Simulated) {
    ukf1Simulated = true;
    runKF(sim);
  }
  if (!sim.simulation.data.empty() &&
      ImPlot::BeginPlot("1D Cart", ImVec2(-1, 0))) {
    ImPlot::SetupAxes("x", "y", ImPlotAxisFlags_AutoFit,
                      ImPlotAxisFlags_AutoFit);
    ImVec2 realPos{(float)sim.simulation.data[1].back().value, 1.0f};
    ImVec2 measuredPos{(float)sim.simulation.dataWithNoise[1].back().value,
                       1.0f};
    ImPlot::SetupAxisLimits(ImAxis_X1, -1, realPos.x + 2, ImPlotCond_Always);

    ImPlot::PlotScatter("Real Position", &realPos.x, &realPos.y, 1);
    ImPlot::PlotScatter("Measured Position", &measuredPos.x, &measuredPos.y, 1);

    ImPlot::EndPlot();
  }
  if (!sim.simulation.data.empty() &&
      ImPlot::BeginPlot("1D Cart position", ImVec2(-1, 0))) {
    ImPlot::SetupAxes("position", "time", ImPlotAxisFlags_AutoFit,
                      ImPlotAxisFlags_AutoFit);

    ImPlot::SetupAxisLimits(ImAxis_X1, -1,
                            sim.simulation.data[1].back().value + 2,
                            ImPlotCond_Always);

    ImPlot::PlotLine("x", &sim.simulation.data[1].front().value,
                     &sim.simulation.data[0].front().value,
                     sim.simulation.data.front().size(), 0, 0, sizeof(Real));

    ImPlot::PlotScatter(
        "x measured", &sim.simulation.dataWithNoise[1].front().value,
        &sim.simulation.dataWithNoise[0].front().value,
        sim.simulation.dataWithNoise.front().size(), 0, 0, sizeof(Real));

    if (!ukf1States.empty() && !ukf1States.front().empty()) {
      ImPlot::PlotLine("x estimated", &ukf1States[1].front(),
                       &ukf1States[0].front(), ukf1States.front().size(), 0, 0,
                       sizeof(double));
    }

    ImPlot::EndPlot();
  }
}
