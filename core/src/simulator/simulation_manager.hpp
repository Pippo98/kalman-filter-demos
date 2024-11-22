#pragma once

#include <map>
#include <variant>
#include <vector>
#include "defines.hpp"
#include "models/ball.hpp"
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
  std::vector<Real> params;
  SimulationData() {};
  template <class Model>
  SimulationData(const std::string &name, Model &model) {
    simulatable = &model;
    simulation.name = name;
    params = model.getValues();
  }
};

void loadSimulation(Simulation &simulation);
void storeSimulation(Simulation &simulation);

class SimulationManager {
 private:
  std::map<std::string, size_t> nameToIndex;
  std::vector<SimulationData> simulations;

  void setParams(size_t idx);
  void setAllParams();
  bool drawExtraParams(const std::string &modelName);

  double dt = 0.05;
  double T = 20.0;

 public:
  SimulationManager();

  Cart1D cart1d;
  Cart2D cart2d;
  Ball ball;

  void reset();
  void simulateAll();
  void simulateOne(size_t index);

  SimulationData &getByName(const std::string &name);

  void draw();
};
