#include <stdio.h>
#include <memory>

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

  double lastTime = TimeBase::getTimestampSeconds();

  while (app.IsOpen()) {
    app.BeginFrame();

    if (TimeBase::getTimestampSeconds() - lastTime > 0.05) {
      simulator.step();
      lastTime = TimeBase::getTimestampSeconds();
    }
    demo1->draw();

    app.EndFrame();
  }
  return 0;
}
