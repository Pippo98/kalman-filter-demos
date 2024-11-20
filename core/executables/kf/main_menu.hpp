#pragma once

#include "simulator/simulator.hpp"

class MainMenu {
 public:
  void draw(Simulator &simulator);

  int getSimulationFrequency() const;

 private:
  bool paused = false;
  int frequency = 20;

  void onPausedChanged(Simulator &simulator);
};
