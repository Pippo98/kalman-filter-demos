#include "demo_4.hpp"

#include <memory>
#include "defines.hpp"
#include "imgui.h"
#include "implot.h"
#include "models/cart_2d.hpp"
#include "simulator/simulation_manager.hpp"
#include "utils/csv.hpp"
#include "utils/kf_utils.hpp"

Demo4::Demo4() {
  {
    kfPosOnly.initializeMatrices({"x", "y"}, {"x", "y"});
    kfPosOnly.X0(0) = -10.0;
    kfPosOnly.X0(1) = -10.0;
    kfPosOnly.P0(0, 0) = 1e1;
    kfPosOnly.P0(1, 1) = 1e1;
    kfPosOnly.Q(0, 0) = 0.1;
    kfPosOnly.Q(1, 1) = 0.1;
    kfPosOnly.R(0, 0) = 0.5;
    kfPosOnly.R(1, 1) = 0.5;
  }
  {
    kfPosAndSpeed.initializeMatrices({"x", "y", "vx", "vy"}, {"x", "y"});
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
    kfPosSpeedAccel.initializeMatrices(
        {"x", "y", "u", "v", "psi"},
        {"w_fl", "w_fr", "w_rl", "w_rr", "x", "y"});
    kfPosSpeedAccel.X0(0) = 0.0;
    kfPosSpeedAccel.X0(1) = 0.0;
    kfPosSpeedAccel.X0(2) = 0.0;
    kfPosSpeedAccel.X0(3) = 0.0;
    kfPosSpeedAccel.X0(4) = 1.5;

    kfPosSpeedAccel.P0(0, 0) = 5;
    kfPosSpeedAccel.P0(1, 1) = 5;
    kfPosSpeedAccel.P0(2, 2) = 5;
    kfPosSpeedAccel.P0(3, 3) = 5;
    kfPosSpeedAccel.P0(4, 4) = 1;

    kfPosSpeedAccel.Q(0, 0) = 0.005;
    kfPosSpeedAccel.Q(1, 1) = 0.005;
    kfPosSpeedAccel.Q(2, 2) = 0.005;
    kfPosSpeedAccel.Q(3, 3) = 0.005;
    kfPosSpeedAccel.Q(4, 4) = 0.01;

    kfPosSpeedAccel.R(0, 0) = 0.2;
    kfPosSpeedAccel.R(1, 1) = 0.2;
    kfPosSpeedAccel.R(2, 2) = 0.2;
    kfPosSpeedAccel.R(3, 3) = 0.2;
    kfPosSpeedAccel.R(4, 4) = 0.1;
    kfPosSpeedAccel.R(5, 5) = 0.1;
  }
}
void Demo4::setupKF() {
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

          newState(0) = state(2) * cos(state(4)) - state(3) * sin(state(4));
          newState(1) = state(2) * sin(state(4)) + state(3) * cos(state(4));
          newState(2) = input(0) + input(2) * state(3);
          newState(3) = input(1) - input(2) * state(2);
          newState(4) = input(2);

          newState = state + newState * dt;
          return newState;
        });
    ukfPositionSpeedAccel.setMeasurementFunction(
        [](const Eigen::VectorXd &state, const Eigen::VectorXd &input,
           void *userData) -> Eigen::VectorXd {
          (void)input;
          (void)userData;
          Eigen::VectorXd measures(6);
          double Rf = 0.1957;
          double Rr = 0.1918;
          double Wf = 1.27;
          double Wr = 1.21;
          measures(0) = 1.0 / Rf * (state(2) - input(2) * Wf / 2.0);
          measures(1) = 1.0 / Rf * (state(2) + input(2) * Wf / 2.0);
          measures(2) = 1.0 / Rr * (state(2) - input(2) * Wr / 2.0);
          measures(3) = 1.0 / Rr * (state(2) + input(2) * Wr / 2.0);
          measures(4) = state(0);
          measures(5) = state(1);
          return measures;
        });
  }
}
bool Demo4::runKF(SimulationData &sim, int iterations) {
  auto &data = sim.simulation.dataWithNoise;
  size_t rows = data.front().size();

  static size_t startRow = 0;
  static size_t endRow;
  if (iterations > 0) {
    endRow = std::min(startRow + iterations + 1, rows);
  } else {
    startRow = 0;
    endRow = rows;
  }
  if (startRow == 0) {
    kfPosOnly.clearData();
    kfPosAndSpeed.clearData();
    kfPosSpeedAccel.clearData();
  }
  for (size_t row = startRow; row < endRow; row++) {
    {
      kfPosOnly.ukf.predict();
      Eigen::VectorXd measure(2);
      measure(0) = data[8][row].value;
      measure(1) = data[9][row].value;
      if (updateEvery == 0 || row % updateEvery == 0) {
        kfPosOnly.ukf.update(measure);
      }

      kfPosOnly.addStateAndCovariance(data[0][row]);
    }
    {
      double dt = 0.0;
      if (row > 0) {
        dt = data[0][row] - data[0][row - 1];
      }
      kfPosAndSpeed.ukf.setUserData(&dt);

      kfPosAndSpeed.ukf.predict();
      Eigen::VectorXd measure(2);
      measure(0) = data[8][row].value;
      measure(1) = data[9][row].value;
      if (updateEvery == 0 || row % updateEvery == 0) {
        kfPosAndSpeed.ukf.update(measure);
      }

      kfPosAndSpeed.addStateAndCovariance(data[0][row]);
    }
    {
      double dt = 0.0;
      if (row > 0) {
        dt = data[0][row] - data[0][row - 1];
      }
      kfPosSpeedAccel.ukf.setUserData(&dt);

      Eigen::VectorXd inputs(3);
      inputs(0) = data[1][row].value * 9.81;
      inputs(1) = data[2][row].value * 9.81;
      inputs(2) = data[3][row].value * M_PI / 180.0;

      kfPosSpeedAccel.ukf.predict(inputs);
      Eigen::VectorXd measure(6);
      measure(0) = data[11][row].value;
      measure(1) = data[12][row].value;
      measure(2) = data[14][row].value;
      measure(3) = data[13][row].value;
      measure(4) = data[8][row].value;
      measure(5) = data[9][row].value;
      // measure(3) = data[10][row].value;
      if (updateEvery == 0 || row % updateEvery == 0) {
        kfPosSpeedAccel.ukf.update(measure);
      }

      kfPosSpeedAccel.addStateAndCovariance(data[0][row]);
    }
  }
  startRow = endRow;
  if (endRow >= rows) {
    startRow = 0;

    kfPosOnly.calculateResiduals("x", data[8]);
    kfPosOnly.calculateResiduals("y", data[9]);
    kfPosAndSpeed.calculateResiduals("x", data[8]);
    kfPosAndSpeed.calculateResiduals("y", data[9]);
    kfPosSpeedAccel.calculateResiduals("x", data[8]);
    kfPosSpeedAccel.calculateResiduals("y", data[9]);

    return true;
  } else {
    return false;
  }
}
void Demo4::draw(SimulationData &sim) {
  // time,
  // accel.x,
  // accel.y,
  // gyro.z,
  // brake.front,
  // throttle,
  // steer.angle,
  // s,
  // position.x, [8]
  // position.y,
  // position.heading,
  // w_f.fl, [11]
  // w_f.fr,
  // w_rr.n_act_filt,
  // w_rl.n_act_filt,
  // gps.lon,
  // gps.lat

  if (!ukfSimulated || lastSimCount != sim.simCount) {
    static bool first = true;
    lastSimCount = sim.simCount;
    if (first) {
      setupKF();
    }
    ukfSimulated = runKF(sim, 50);
    first = ukfSimulated;
  }
  bool kfModified = false;

  ImVec2 reg = ImGui::GetContentRegionAvail();

  ImGui::SetNextItemWidth(120);
  if (ImGui::InputInt("Update every n samples", &updateEvery, 1, 1)) {
    kfModified = true;
  }
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
                         &sim.simulation.data[8][0].value,
                         sim.simulation.data[0].size(), 0, 0, sizeof(Real));

        plotKFState("kf1", "x", kfPosOnly);
        plotKFState("kf2", "x", kfPosAndSpeed);
        plotKFState("kf3", "x", kfPosSpeedAccel);
        ImPlot::EndPlot();
      }
      if (ImPlot::BeginPlot("State Y")) {
        ImPlot::SetupAxes("time", "position");
        ImPlot::SetNextLineStyle({1.0f, 1.0f, 1.0f, 1.0f});
        ImPlot::PlotLine("real y", &sim.simulation.data[0][0].value,
                         &sim.simulation.data[9][0].value,
                         sim.simulation.data[0].size(), 0, 0, sizeof(Real));

        plotKFState("kf1", "y", kfPosOnly);
        plotKFState("kf2", "y", kfPosAndSpeed);
        plotKFState("kf3", "y", kfPosSpeedAccel);
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
          plotKFCovariance("kf1", "x", kfPosOnly);
          plotKFCovariance("kf2", "x", kfPosAndSpeed);
          plotKFCovariance("kf3", "x", kfPosSpeedAccel);
        }

        if (showRes) {
          ImPlot::SetAxis(ImAxis_Y2);
          ImPlot::PlotLine("res kf1.x", &kfPosOnly.states[0][0].value,
                           &kfPosOnly.residuals["x"][0].value,
                           kfPosOnly.states[0].size(), 0, 0, sizeof(Real));
          ImPlot::PlotLine("res kf2.x", &kfPosAndSpeed.states[0][0].value,
                           &kfPosAndSpeed.residuals["x"][0].value,
                           kfPosAndSpeed.states[0].size(), 0, 0, sizeof(Real));
          ImPlot::PlotLine("res kf3.x", &kfPosSpeedAccel.states[0][0].value,
                           &kfPosSpeedAccel.residuals["x"][0].value,
                           kfPosSpeedAccel.states[0].size(), 0, 0,
                           sizeof(Real));
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
          plotKFCovariance("kf1", "y", kfPosOnly);
          plotKFCovariance("kf2", "y", kfPosAndSpeed);
          plotKFCovariance("kf3", "y", kfPosSpeedAccel);
        }
        if (showRes) {
          ImPlot::SetAxis(ImAxis_Y2);
          ImPlot::PlotLine("res kf1.y", &kfPosOnly.states[0][0].value,
                           &kfPosOnly.residuals["y"][0].value,
                           kfPosOnly.states[0].size(), 0, 0, sizeof(Real));
          ImPlot::PlotLine("res kf2.y", &kfPosAndSpeed.states[0][0].value,
                           &kfPosAndSpeed.residuals["y"][0].value,
                           kfPosAndSpeed.states[0].size(), 0, 0, sizeof(Real));
          ImPlot::PlotLine("res kf3.y", &kfPosSpeedAccel.states[0][0].value,
                           &kfPosSpeedAccel.residuals["y"][0].value,
                           kfPosSpeedAccel.states[0].size(), 0, 0,
                           sizeof(Real));
        }
        ImPlot::EndPlot();
      }

      ImPlot::EndSubplots();
    }
  }

  if (kfModified) {
    ukfSimulated = false;
  }

  if (ImPlot::BeginPlot("Vehicle position", reg, ImPlotFlags_Equal)) {
    ImPlot::SetupAxes("x", "y");

    ImPlot::PlotLine("x", &sim.simulation.data[8].front().value,
                     &sim.simulation.data[9].front().value,
                     sim.simulation.data[0].size(), 0, 0, sizeof(Real));

    if (kfPosOnly.hasStates()) {
      ImPlot::PlotLine("kf1", &kfPosOnly.states[1][0].value,
                       &kfPosOnly.states[2][0].value,
                       kfPosOnly.states[0].size(), 0, 0, sizeof(Real));
    }
    if (kfPosAndSpeed.hasStates()) {
      ImPlot::PlotLine("kf2", &kfPosAndSpeed.states[1][0].value,
                       &kfPosAndSpeed.states[2][0].value,
                       kfPosAndSpeed.states[0].size(), 0, 0, sizeof(Real));
    }
    if (kfPosSpeedAccel.hasStates()) {
      ImPlot::PlotLine("kf3", &kfPosSpeedAccel.states[1][0].value,
                       &kfPosSpeedAccel.states[2][0].value,
                       kfPosSpeedAccel.states[0].size(), 0, 0, sizeof(Real));
    }

    ImPlot::EndPlot();
  }
}
