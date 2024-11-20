#pragma once

#include <initializer_list>
#include <memory>
#include <vector>

class Simulatable {
 public:
  virtual void step(double dt) = 0;
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
