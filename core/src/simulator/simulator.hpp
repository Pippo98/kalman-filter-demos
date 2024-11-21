#pragma once

#include <initializer_list>
#include <memory>
#include <vector>
#include "utils/type_and_name.hpp"

class Simulatable {
 public:
  virtual void step(double dt) = 0;
  std::vector<Real> getValues() {
    auto ptrs = getValuesPtr();
    std::vector<Real> ret;
    for (const auto &ptr : ptrs) {
      ret.push_back(*ptr);
    }
    return ret;
  };
  std::vector<Real> getValuesWithNoise() {
    auto vals = getValues();
    for (auto &val : vals) {
      val.addNoise();
    }
    return vals;
  }

 private:
  virtual std::vector<Real *> getValuesPtr() = 0;
};

class Simulator {
 public:
  Simulator();
  void setSystems(std::initializer_list<std::shared_ptr<Simulatable>> systems);

  void step(double dt);
  void step();

  void resetTime();

 private:
  double time;
  std::vector<std::shared_ptr<Simulatable>> systems;
};
