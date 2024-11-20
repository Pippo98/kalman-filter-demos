#include "demo_1.hpp"
#include <memory>
#include "imgui.h"
#include "implot.h"
#include "utils/csv.hpp"

Demo1::Demo1() {}
void Demo1::draw() {
  ImVec2 reg = ImGui::GetContentRegionAvail();
  drawCSVTable("real", realStates, ImVec2(reg.x / 2.0, reg.y / 5.0));
  ImGui::SameLine();
  drawCSVTable("measured", measuredStates, ImVec2(reg.x / 2.0, reg.y / 5.0));

  if (!realStates.empty() && ImPlot::BeginPlot("1D Cart", ImVec2(-1, 0))) {
    ImPlot::SetupAxes("x", "y", ImPlotAxisFlags_AutoFit,
                      ImPlotAxisFlags_AutoFit);
    ImVec2 realPos{(float)realStates[1].back().value, 1.0f};
    ImVec2 measuredPos{(float)measuredStates[1].back().value, 1.0f};
    ImPlot::SetupAxisLimits(ImAxis_X1, -1, realPos.x + 2, ImPlotCond_Always);

    ImPlot::PlotScatter("Real Position", &realPos.x, &realPos.y, 1);
    ImPlot::PlotScatter("Measured Position", &measuredPos.x, &measuredPos.y, 1);

    ImPlot::EndPlot();
  }
  if (!realStates.empty() &&
      ImPlot::BeginPlot("1D Cart position", ImVec2(-1, 0))) {
    ImPlot::SetupAxes("Time", "x", ImPlotAxisFlags_AutoFit,
                      ImPlotAxisFlags_AutoFit);

    ImPlot::SetupAxisLimits(ImAxis_X1, -1, realStates[1].back().value + 2,
                            ImPlotCond_Always);

    ImPlot::PlotLine("x", &(realStates[1].front().value),
                     &(realStates[0].front().value), realStates.front().size(),
                     0, 0, sizeof(Real));

    ImPlot::PlotScatter("x measured", &(measuredStates[1].front().value),
                        &(measuredStates[0].front().value),
                        measuredStates.front().size(), 0, 0, sizeof(Real));
    ImPlot::EndPlot();
  }
}
