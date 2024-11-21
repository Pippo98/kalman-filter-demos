#include "demo_2.hpp"

#include <memory>
#include "imgui.h"
#include "implot.h"
#include "utils/csv.hpp"

Demo2::Demo2() {}
void Demo2::draw() {
  ImVec2 reg = ImGui::GetContentRegionAvail();
  drawCSVTable("real", realStates, ImVec2(reg.x / 2.0, reg.y / 5.0));
  ImGui::SameLine();
  drawCSVTable("measured", measuredStates, ImVec2(reg.x / 2.0, reg.y / 5.0));

  reg = ImGui::GetContentRegionAvail();
  if (!realStates.empty() &&
      ImPlot::BeginPlot("2D Cart position", reg, ImPlotFlags_Equal)) {
    ImPlot::SetupAxes("x", "y", ImPlotAxisFlags_AutoFit,
                      ImPlotAxisFlags_AutoFit);

    ImPlot::PlotLine("x", &realStates[1].front().value,
                     &realStates[2].front().value, realStates.front().size(), 0,
                     0, sizeof(Real));

    ImPlot::PlotScatter("x measured", &measuredStates[1].front().value,
                        &measuredStates[2].front().value,
                        measuredStates.front().size(), 0, 0, sizeof(Real));

    ImVec2 groundPoints[3] = {{0.0, 5.0},
                              {10.0, 5.0},
                              {float(10.0 + 30.0 * std::cos(cart.alpha)),
                               float(5.0 - 30 * std::sin(cart.alpha))}};
    ImPlot::PlotLine("ground", &groundPoints[0].x, &groundPoints[0].y, 3, 0, 0,
                     sizeof(ImVec2));
    ImPlot::EndPlot();
  }
}
