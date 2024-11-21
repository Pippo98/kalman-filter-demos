#include "simulation_manager.hpp"
#include "utils/csv.hpp"
#include "utils/utils.hpp"
#include <cassert>

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
    {
        .simulatable = &cart1d,
        .simulation = Simulation("Cart 1D"),
        .params = {
            {"x", 0.0},
            {"u", 1.0}
        }
    },
    {
        .simulatable = &cart2d,
        .simulation = Simulation("Cart 2D"),
        .params = {
            {"x", 0.0},
            {"y", 5.0},
            {"u", 1.0}
        }
    }
    };
    for(size_t i = 0; i < simulations.size(); i++) {
        nameToIndex[simulations[i].simulation.name] = i;
    }
  // clang-format on

  setParams();
}
void SimulationManager::setParams() {
  for (auto &simulation : simulations) {
    simulation.simulatable->setValueByName("t", 0.0);
    for (const auto &[name, value] : simulation.params) {
      simulation.simulatable->setValueByName(name, value);
    }
  }
}
void SimulationManager::reset() {}
void SimulationManager::simulateAll(double dt, double T) {
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
SimulationData &SimulationManager::getByName(const std::string &name) {
  return simulations[nameToIndex[name]];
}
