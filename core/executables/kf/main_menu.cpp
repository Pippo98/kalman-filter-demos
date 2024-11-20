#include "main_menu.hpp"
#include "imgui.h"
#include "imgui_internal.h"
#include "utils/time_base.hpp"
#include "resources/fonts/font_awesome.h"

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
    ImGui::Text("Kalman Filter Demos");
    ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical, 3);
    if (ImGui::Button(paused ? ICON_FA_PLAY : ICON_FA_PAUSE)) {
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
