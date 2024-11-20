#include <stdio.h>
#include <memory>

#include "core/executables/kf/main_menu.hpp"
#include "demos/1/demo_1.hpp"
#include "app/app.hpp"
#include "demos/2/demo_2.hpp"
#include "imgui.h"
#include "utils/time_base.hpp"

int main(void) {
  App app("KF Demos");

  app.Open();

  std::shared_ptr<Demo1> demo1 = std::make_shared<Demo1>();
  std::shared_ptr<Demo2> demo2 = std::make_shared<Demo2>();

  Simulator simulator;
  simulator.setSystems({demo1, demo2});

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
          demo1->draw();
          ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Demo 2")) {
          demo2->draw();
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
