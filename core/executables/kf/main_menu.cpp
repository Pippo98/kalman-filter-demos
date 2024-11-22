#include "main_menu.hpp"
#include "imgui.h"
#include "imgui_internal.h"
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
    ImGui::EndMainMenuBar();
  }
}
