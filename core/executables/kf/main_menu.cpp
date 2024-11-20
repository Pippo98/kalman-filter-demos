#include "main_menu.hpp"
#include "imgui.h"
#include "utils/time_base.hpp"

void MainMenu::onPausedChanged(Simulator &simulator) {
  paused = !paused;
  if (paused) {
    TimeBase::setTimestamp(TimeBase::getTimestampMicroseconds());
    TimeBase::enableTesting();
  } else {
    TimeBase::disableTesting();
    simulator.resetTime();
  }
}

void MainMenu::draw(Simulator &simulator) {
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::Button(paused ? "Resume" : "Pause")) {
      onPausedChanged(simulator);
    }
    ImGui::Text("Simulation frequency");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::SliderInt("##simulationFrequency", &frequency, 1, 100);
    ImGui::EndMainMenuBar();
  }
}
int MainMenu::getSimulationFrequency() const { return frequency; }
