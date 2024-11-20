#include <stdio.h>
#include <memory>

#include "core/executables/kf/main_menu.hpp"
#include "demos/1/demo_1.hpp"
#include "app/app.hpp"
#include "imgui.h"
#include "utils/time_base.hpp"

int main(void) {
  App app("KF Demos");

  app.Open();

  std::shared_ptr<Demo1> demo1 = std::make_shared<Demo1>();

  Simulator simulator;
  simulator.setSystems({demo1});

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
      demo1->draw();
    }
    ImGui::End();

    app.EndFrame();
  }
  return 0;
}
