#pragma once

#include <map>
#include <vector>
#include "defines.hpp"
#include "models/cart_1d.hpp"
#include "models/cart_2d.hpp"
#include "simulator/simulator.hpp"
#include "utils/type_and_name.hpp"

class Simulation {
 public:
  Simulation() {}
  Simulation(std::string _name) : name(_name) {}
  std::string name;
  std::vector<std::vector<Real>> data;
  std::vector<std::vector<Real>> dataWithNoise;
  std::string getCSVPath() { return CSV_OUT_PATH "/" + name + ".csv"; }
};
struct SimulationData {
  Simulatable *simulatable;
  Simulation simulation;
  std::map<std::string, double> params;
};

void loadSimulation(Simulation &simulation);
void storeSimulation(Simulation &simulation);

class SimulationManager {
 private:
  std::vector<SimulationData> simulations;
  std::map<std::string, size_t> nameToIndex;

  void setParams();

 public:
  SimulationManager();

  Cart1D cart1d;
  Cart2D cart2d;

  void reset();
  void simulateAll(double dt, double T);

  SimulationData &getByName(const std::string &name);
};
