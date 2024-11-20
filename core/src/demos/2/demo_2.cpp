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

  // if (!realStates.empty() && ImPlot::BeginPlot("1D Cart", ImVec2(-1, 0))) {
  //   ImPlot::SetupAxes("x", "y", ImPlotAxisFlags_AutoFit,
  //                     ImPlotAxisFlags_AutoFit);
  //   ImVec2 realPos{(float)realStates[1].back().value, 1.0f};
  //   ImVec2 measuredPos{(float)measuredStates[1].back().value, 1.0f};
  //   ImPlot::SetupAxisLimits(ImAxis_X1, -1, realPos.x + 2, ImPlotCond_Always);
  //
  //   ImPlot::PlotScatter("Real Position", &realPos.x, &realPos.y, 1);
  //   ImPlot::PlotScatter("Measured Position", &measuredPos.x, &measuredPos.y,
  //   1);
  //
  //   ImPlot::EndPlot();
  // }
  reg = ImGui::GetContentRegionAvail();
  if (!realStates.empty() &&
      ImPlot::BeginPlot("2D Cart position", reg, ImPlotFlags_Equal)) {
    ImPlot::SetupAxes("position", "time", ImPlotAxisFlags_AutoFit,
                      ImPlotAxisFlags_AutoFit);

    ImPlot::PlotLine("x", &realStates[1].front().value,
                     &realStates[2].front().value, realStates.front().size(), 0,
                     0, sizeof(Real));

    ImPlot::PlotScatter("x measured", &(measuredStates[1].front().value),
                        &measuredStates[2].front().value,
                        measuredStates.front().size(), 0, 0, sizeof(Real));
    ImPlot::EndPlot();
  }
}
