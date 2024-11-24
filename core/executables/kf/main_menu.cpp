#include "main_menu.hpp"
#include "imgui.h"
#include "imgui_internal.h"
#include "implot.h"
#include "utils/time_base.hpp"
#include "resources/fonts/font_awesome.h"

void MainMenu::onPausedChanged() { paused = !paused; }

void MainMenu::draw() {
  if (ImGui::BeginMainMenuBar()) {
    ImGui::Text("Kalman Filter Demos");
    ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical, 3);
    if (ImGui::Button(paused ? ICON_FA_PLAY : ICON_FA_PAUSE)) {
      onPausedChanged();
    }
    ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical, 3);
    ImGui::Text("def line weight");
    ImGui::InputFloat("##default line weight", &ImPlot::GetStyle().LineWeight);

    ImGui::EndMainMenuBar();
  }
}
