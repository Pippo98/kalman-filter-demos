#include <stdio.h>
#include <iostream>
#include <memory>

#include "core/executables/kf/main_menu.hpp"
#include "defines.hpp"
#include "demos/1/demo_1.hpp"
#include "app/app.hpp"
#include "demos/2/demo_2.hpp"
#include "demos/3/demo_3.hpp"
#include "demos/4/demo_4.hpp"
#include "demos/s1/demo_sigma_points.hpp"
#include "imgui.h"
#include "imgui_internal.h"
#include "kflib/src/ukf.hpp"
#include "simulator/simulation_manager.hpp"
#include "utils/time_base.hpp"
#include "utils/styles.hpp"
#include "resources/fonts/font_awesome.h"

ImFont *font_H1;
ImFont *font_H2;
ImFont *font_H3;
ImFont *font_text;

int main(void) {
  App app("KF Demos");

  app.Open();
  Dracula();

  font_text = ImGui::GetIO().Fonts->AddFontFromFileTTF(
      FONTS_PATH "/JetBrainsMonoNerdFont-Regular.ttf", 18);
  font_H1 = ImGui::GetIO().Fonts->AddFontFromFileTTF(
      FONTS_PATH "/JetBrainsMonoNerdFont-Regular.ttf", 28);
  font_H2 = ImGui::GetIO().Fonts->AddFontFromFileTTF(
      FONTS_PATH "/JetBrainsMonoNerdFont-Regular.ttf", 24);
  font_H3 = ImGui::GetIO().Fonts->AddFontFromFileTTF(
      FONTS_PATH "/JetBrainsMonoNerdFont-Regular.ttf", 22);

  static const ImWchar icons_ranges[] = {ICON_MIN_FA, ICON_MAX_FA, 0};
  ImFontConfig icons_config;
  icons_config.MergeMode = true;
  icons_config.PixelSnapH = true;
  ImGui::GetIO().Fonts->AddFontFromFileTTF(FONTS_PATH "/fa-solid-900.ttf", 18,
                                           &icons_config, icons_ranges);

  Simulation skidpad("skidpad");
  loadSimulation(skidpad);
  SimulationData skidpadD;
  skidpadD.simulation = skidpad;
  skidpadD.simCount++;

  DemoSigmaPoints demoSigmaPoints;
  std::shared_ptr<Demo1> demo1 = std::make_shared<Demo1>();
  std::shared_ptr<Demo2> demo2 = std::make_shared<Demo2>();
  std::shared_ptr<Demo3> demo3 = std::make_shared<Demo3>();
  std::shared_ptr<Demo4> demo4 = std::make_shared<Demo4>();

  Simulator simulator;
  simulator.setSystems({demo1, demo2, demo3, demo4});

  SimulationManager simulationManager;
  simulationManager.simulateAll();

  MainMenu mainMenu;

  double lastTime = TimeBase::getTimestampSeconds();
  while (app.IsOpen()) {
    app.BeginFrame();

    mainMenu.draw();

    if (TimeBase::getTimestampSeconds() - lastTime >= 1.0 / 20.0) {
      simulator.step();
      lastTime = TimeBase::getTimestampSeconds();
    }

    if (ImGui::Begin("Simulation Manager")) {
      simulationManager.draw();
    }
    ImGui::End();

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
        if (ImGui::BeginTabItem("Demo 3")) {
          demo3->draw(simulationManager.getByName("Ball"));
          ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Demo 4")) {
          demo4->draw(skidpadD);
          ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Extra")) {
          demoSigmaPoints.draw();
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
