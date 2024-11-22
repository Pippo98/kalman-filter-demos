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
  auto line = csv.readLine();
  while (true) {
    line = csv.readLine();
    if (line.empty()) {
      break;
    }
    simulation.data.push_back(line);
  }
}
void storeSimulation(Simulation &simulation) {
  CSV csv;
  if (!csv.open(simulation.getCSVPath(), "w")) {
    assert(false && "Cannot open file");
  }
  for (const auto &row : simulation.data) {
    csv.writeLine(row);
  }
}

SimulationManager::SimulationManager() {
  // clang-format off
    simulations = {
        {"Cart 1D", cart1d},
        {"Cart 2D", cart2d},
    };
    for(size_t i = 0; i < simulations.size(); i++) {
        nameToIndex[simulations[i].simulation.name] = i;
    }
  // clang-format on

  setAllParams();
}
void SimulationManager::setParams(size_t idx) {
  simulations[idx].simulatable->setValueByName("t", 0.0);
  for (const auto &[name, value, _] : simulations[idx].params) {
    simulations[idx].simulatable->setValueByName(name, value);
  }
}
void SimulationManager::setAllParams() {
  for (auto &simulation : simulations) {
    simulation.simulatable->setValueByName("t", 0.0);
    for (const auto &[name, value, _] : simulation.params) {
      simulation.simulatable->setValueByName(name, value);
    }
  }
}
void SimulationManager::reset() {}
void SimulationManager::simulateAll() {
  for (auto &sim : simulations) {
    sim.simulation.data.clear();
    sim.simulation.dataWithNoise.clear();
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
      if (ImGui::BeginTabItem((std::to_string(simulationIdx) + ") " +
                               simulation.simulation.name)
                                  .c_str())) {
        ImGui::SeparatorText("States Initial Conditions");
        for (auto &[name, value, _] : simulation.params) {
          if (name == "t") {
            continue;
          }
          float valueFloat = value;

          if (ImGui::SliderFloat(name.c_str(), &valueFloat, -10.0, 10.0)) {
            simulationChangedIdx = simulationIdx;
            value = valueFloat;
          }
        }
        ImGui::SeparatorText("Model Parameters");
        if (drawExtraParams(simulation.simulation.name)) {
          simulationChangedIdx = simulationIdx;
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
