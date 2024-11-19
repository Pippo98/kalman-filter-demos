#include <stdio.h>

#include "demos/1/demo_1.hpp"
#include "app/app.hpp"
#include "imgui.h"

int main(void) {
  App app("KF Demos");

  app.Open();

  while (app.IsOpen()) {
    app.BeginFrame();

    ImGui::Begin("KF");
    ImGui::Text("--- heellooo");
    ImGui::End();

    app.EndFrame();
  }
  return 0;
}
