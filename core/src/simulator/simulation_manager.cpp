#include "simulation_manager.hpp"
#include "imgui.h"
#include "utils/csv.hpp"
#include "utils/utils.hpp"
#include <cassert>
#include <string>

void loadSimulation(Simulation &simulation) {
  CSV csv;
  if (!csv.open(simulation.getCSVPath(), "r")) {
    assert(false && "Cannot open file");
  }
  simulation.data.clear();
  simulation.dataWithNoise.clear();
  auto line = csv.readLine();
  while (true) {
    line = csv.readLine();
    if (line.empty()) {
      break;
    }
    rowToMatrix(simulation.data, line);
    for (size_t col = 0; col < line.size(); col++) {
      if (simulation.dataWithNoise.size() != line.size()) {
        simulation.dataWithNoise.resize(line.size());
      }
      simulation.dataWithNoise[col].push_back(line[col].getWithNoise());
    }
  }
}
void storeSimulation(Simulation &simulation) {
  CSV csv;
  if (!csv.open(simulation.getCSVPath(), "w")) {
    assert(false && "Cannot open file");
  }
  assert(!simulation.data.empty() && "Data empty");
  size_t cols = simulation.data.size();
  size_t rows = simulation.data[0].size();
  std::vector<Real> rowValues;
  rowValues.reserve(cols);
  for (size_t row = 0; row < rows; row++) {
    for (size_t col = 0; col < cols; col++) {
      rowValues.push_back(simulation.data[col][row]);
    }
    csv.writeLine(rowValues);
    rowValues.clear();
  }
}

SimulationManager::SimulationManager() {
  // clang-format off
    simulations = {
        {"Cart 1D", cart1d},
        {"Cart 2D", cart2d},
        {"Ball", ball},
    };
    for(size_t i = 0; i < simulations.size(); i++) {
        nameToIndex[simulations[i].simulation.name] = i;
    }
  // clang-format on

  setAllParams();
}
void SimulationManager::setParams(size_t idx) {
  simulations[idx].simulatable->setValueByName("t", 0.0);
  for (const auto &[name, value, noise] : simulations[idx].params) {
    simulations[idx].simulatable->setValueByName(name, value, noise.stddev());
  }
}
void SimulationManager::setAllParams() {
  for (auto &simulation : simulations) {
    simulation.simulatable->setValueByName("t", 0.0);
    for (const auto &[name, value, noise] : simulation.params) {
      simulation.simulatable->setValueByName(name, value, noise.stddev());
    }
  }
}
void SimulationManager::reset() {}
void SimulationManager::simulateAll() {
  for (auto &sim : simulations) {
    sim.simulation.data.clear();
    sim.simulation.dataWithNoise.clear();
    sim.simCount++;
  }
  for (double t = 0.0; t < T; t += dt) {
    for (auto &sim : simulations) {
      sim.simulatable->step(dt);
      rowToMatrix(sim.simulation.data, sim.simulatable->getValues());
      rowToMatrix(sim.simulation.dataWithNoise,
                  sim.simulatable->getValuesWithNoise());
    }
  }
}
void SimulationManager::simulateOne(size_t idx) {
  SimulationData &sim = simulations[idx];
  sim.simulation.data.clear();
  sim.simulation.dataWithNoise.clear();
  sim.simCount++;
  for (double t = 0.0; t < T; t += dt) {
    sim.simulatable->step(dt);
    rowToMatrix(sim.simulation.data, sim.simulatable->getValues());
    rowToMatrix(sim.simulation.dataWithNoise,
                sim.simulatable->getValuesWithNoise());
  }
}
SimulationData &SimulationManager::getByName(const std::string &name) {
  return simulations[nameToIndex[name]];
}
bool SimulationManager::drawExtraParams(const std::string &modelName) {
  bool changed = false;
  if (modelName == "Cart 1D") {
  } else if (modelName == "Cart 2D") {
    float alpha = cart2d.alpha;
    if (ImGui::SliderAngle("alpha", &alpha, 0.0, 30.0)) {
      cart2d.alpha = alpha;
      changed = true;
    }
    float inclX = cart2d.planeInclinationX;
    if (ImGui::SliderFloat("inclination start X", &inclX, 0.0, 10.0)) {
      cart2d.planeInclinationX = inclX;
      changed = true;
    }
  } else if (modelName == "Ball") {
    float c = ball.c;
    if (ImGui::SliderFloat("c", &c, 0.0, 0.05)) {
      changed = true;
      ball.c = c;
    }
    float Cd = ball.Cd;
    if (ImGui::SliderFloat("Cd", &Cd, 0.0, 0.05)) {
      changed = true;
      ball.Cd = Cd;
    }
  }
  return changed;
}

void SimulationManager::draw() {
  bool resimulateAll = false;
  int freqHz = 1.0 / dt;
  float time = T;
  if (ImGui::SliderInt("Frequency", &freqHz, 1, 100)) {
    resimulateAll = true;
    dt = 1.0 / freqHz;
  }
  if (ImGui::SliderFloat("Duration", &time, 1.0, 20.0)) {
    resimulateAll = true;
    T = time;
  }

  ssize_t simulationChangedIdx = -1;
  if (ImGui::BeginTabBar("Simulations")) {
    for (size_t simulationIdx = 0; simulationIdx < simulations.size();
         simulationIdx++) {
      SimulationData &simulation = simulations[simulationIdx];

      if (ImGui::BeginTabItem((std::to_string(simulationIdx + 1) + ") " +
                               simulation.simulation.name)
                                  .c_str())) {
        ImGui::SeparatorText("States Initial Conditions");
        for (auto &[name, value, noise] : simulation.params) {
          if (name == "t") {
            continue;
          }
          float valueFloat = value;

          ImGui::SetNextItemWidth(120);
          if (ImGui::InputFloat(name.c_str(), &valueFloat, 0.01, 0.1)) {
            simulationChangedIdx = simulationIdx;
            value = valueFloat;
          }
          ImGui::SameLine();
          ImGui::SetNextItemWidth(120);
          float std = noise.stddev();
          if (ImGui::InputFloat(("std of " + name).c_str(), &std, 0.01, 0.1)) {
            simulationChangedIdx = simulationIdx;
            noise = decltype(noise)(0.0, std);
          }
        }
        ImGui::SeparatorText("Model Parameters");
        if (drawExtraParams(simulation.simulation.name)) {
          simulationChangedIdx = simulationIdx;
        }

        ImGui::SeparatorText("Data");
        if (ImGui::Button("Save simulation")) {
          storeSimulation(simulation.simulation);
          loadSimulation(simulation.simulation);
        }

        ImVec2 reg = ImGui::GetContentRegionAvail();
        if (ImGui::CollapsingHeader("Simulated Data")) {
          drawCSVTable("real", simulation.simulation.data,
                       ImVec2(reg.x, reg.y / 5.0));
        }
        if (ImGui::CollapsingHeader("Measured Data")) {
          drawCSVTable("measured", simulation.simulation.dataWithNoise,
                       ImVec2(reg.x, reg.y / 5.0));
        }

        ImGui::EndTabItem();
      }
    }

    ImGui::EndTabBar();
  }
  if (simulationChangedIdx != -1) {
    setParams(simulationChangedIdx);
    simulateOne(simulationChangedIdx);
  }
  if (resimulateAll) {
    setAllParams();
    simulateAll();
  }
}
