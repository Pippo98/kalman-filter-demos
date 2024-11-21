#include <stdio.h>
#include <memory>

#include "core/executables/kf/main_menu.hpp"
#include "demos/1/demo_1.hpp"
#include "app/app.hpp"
#include "demos/2/demo_2.hpp"
#include "imgui.h"
#include "simulator/simulation_manager.hpp"
#include "utils/time_base.hpp"
#include "utils/styles.hpp"
#include "resources/fonts/font_awesome.h"

int main(void) {
  App app("KF Demos");

  app.Open();
  Dracula();

  ImGui::GetIO().Fonts->AddFontFromFileTTF(
      FONTS_PATH "/JetBrainsMonoNerdFont-Regular.ttf", 18);

  static const ImWchar icons_ranges[] = {ICON_MIN_FA, ICON_MAX_FA, 0};
  ImFontConfig icons_config;
  icons_config.MergeMode = true;
  icons_config.PixelSnapH = true;
  ImGui::GetIO().Fonts->AddFontFromFileTTF(FONTS_PATH "/fa-solid-900.ttf", 18,
                                           &icons_config, icons_ranges);

  std::shared_ptr<Demo1> demo1 = std::make_shared<Demo1>();
  std::shared_ptr<Demo2> demo2 = std::make_shared<Demo2>();

  Simulator simulator;
  simulator.setSystems({demo1, demo2});

  SimulationManager simulationManager;
  simulationManager.simulateAll(1 / 50.0, 20.0);

  MainMenu mainMenu;

  double lastTime = TimeBase::getTimestampSeconds();
  while (app.IsOpen()) {
    app.BeginFrame();

    mainMenu.draw(simulator);

    if (TimeBase::getTimestampSeconds() - lastTime >=
        1.0 / mainMenu.getSimulationFrequency()) {
      simulator.step();
      lastTime = TimeBase::getTimestampSeconds();
    }

    if (ImGui::Begin("Demos", nullptr)) {
      if (ImGui::BeginTabBar("Demos tabs")) {
        if (ImGui::BeginTabItem("Demo 1")) {
          demo1->draw(simulationManager.getByName("Cart 1D"));
          ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Demo 2")) {
          demo2->draw(simulationManager.getByName("Cart 2D"));
          ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
      }
    }
    ImGui::End();

    app.EndFrame();
  }
  return 0;
}
