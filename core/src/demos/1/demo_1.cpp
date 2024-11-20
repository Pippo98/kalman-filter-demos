#include "demo_1.hpp"
#include <memory>
#include "imgui.h"
#include "implot.h"

void drawCSVTable(const char *id, const std::vector<std::vector<Real>> &values,
                  ImVec2 size) {
  if (!values.empty() &&
      ImGui::BeginTable(id, values.size(), ImGuiTableFlags_ScrollY, size)) {
    for (const auto &col : values) {
      ImGui::TableSetupColumn(col.front().name.c_str());
    }
    ImGui::TableHeadersRow();

    auto rows = values.front().size();
    for (size_t row = 0; row < rows; row++) {
      ImGui::TableNextRow();
      for (size_t col = 0; col < values.size(); col++) {
        ImGui::TableNextColumn();
        ImGui::Text("%0.3f\n", values[col][rows - row - 1].value);
      }
    }

    ImGui::EndTable();
  }
}

Demo1::Demo1() {}
void Demo1::draw() {
  if (ImGui::Begin("Demo 1")) {
    ImVec2 reg = ImGui::GetContentRegionAvail();
    drawCSVTable("real", realStates, ImVec2(reg.x / 2.0, reg.y / 3.0));
    ImGui::SameLine();
    drawCSVTable("measured", measuredStates, ImVec2(reg.x / 2.0, reg.y / 3.0));

    if (!realStates.empty() && ImPlot::BeginPlot("1D Cart", ImVec2(-1, 0))) {
      ImPlot::SetupAxes("x", "y", ImPlotAxisFlags_AutoFit,
                        ImPlotAxisFlags_AutoFit);
      ImVec2 realPos{(float)realStates.front().back().value, 1.0f};
      ImVec2 measuredPos{(float)measuredStates.front().back().value, 1.0f};
      ImPlot::SetupAxisLimits(ImAxis_X1, -1, realPos.x + 2, ImPlotCond_Always);

      ImPlot::PlotScatter("Real Position", &realPos.x, &realPos.y, 1);
      ImPlot::PlotScatter("Measured Position", &measuredPos.x, &measuredPos.y,
                          1);

      ImPlot::EndPlot();
    }
    if (!realStates.empty() &&
        ImPlot::BeginPlot("1D Cart position", ImVec2(-1, 0))) {
      ImPlot::SetupAxes("Time", "x", ImPlotAxisFlags_AutoFit,
                        ImPlotAxisFlags_AutoFit);
      ImPlot::PlotLine("x", &(realStates.front().front().value),
                       realStates.front().size(), 1.0, 0.0, 0, 0, sizeof(Real));
      ImPlot::PlotScatter("x measured", &(measuredStates.front().front().value),
                          measuredStates.front().size(), 1.0, 0.0, 0, 0,
                          sizeof(Real));
      ImPlot::EndPlot();
    }
  }
  ImGui::End();
}
