#include <stdio.h>

#include "demos/1/demo_1.hpp"
#include "app/app.hpp"
#include "imgui.h"

int main(void) {
  App app("KF Demos");

  app.Open();

  while (app.IsOpen()) {
    app.BeginFrame();

    app.EndFrame();
  }
  return 0;
}
