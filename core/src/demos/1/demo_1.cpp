#include "demo_1.hpp"
#include <memory>
#include "imgui.h"
#include "implot.h"
#include "utils/csv.hpp"

Demo1::Demo1() {}
void Demo1::draw(SimulationData& sim) {
  if (!sim.simulation.data.empty() &&
      ImPlot::BeginPlot("1D Cart", ImVec2(-1, 0))) {
    ImPlot::SetupAxes("x", "y", ImPlotAxisFlags_AutoFit,
                      ImPlotAxisFlags_AutoFit);
    ImVec2 realPos{(float)sim.simulation.data[1].back().value, 1.0f};
    ImVec2 measuredPos{(float)sim.simulation.dataWithNoise[1].back().value,
                       1.0f};
    ImPlot::SetupAxisLimits(ImAxis_X1, -1, realPos.x + 2, ImPlotCond_Always);

    ImPlot::PlotScatter("Real Position", &realPos.x, &realPos.y, 1);
    ImPlot::PlotScatter("Measured Position", &measuredPos.x, &measuredPos.y, 1);

    ImPlot::EndPlot();
  }
  if (!sim.simulation.data.empty() &&
      ImPlot::BeginPlot("1D Cart position", ImVec2(-1, 0))) {
    ImPlot::SetupAxes("position", "time", ImPlotAxisFlags_AutoFit,
                      ImPlotAxisFlags_AutoFit);

    ImPlot::SetupAxisLimits(ImAxis_X1, -1,
                            sim.simulation.data[1].back().value + 2,
                            ImPlotCond_Always);

    ImPlot::PlotLine("x", &(sim.simulation.data[1].front().value),
                     &(sim.simulation.data[0].front().value),
                     sim.simulation.data.front().size(), 0, 0, sizeof(Real));

    ImPlot::PlotScatter(
        "x measured", &(sim.simulation.dataWithNoise[1].front().value),
        &(sim.simulation.dataWithNoise[0].front().value),
        sim.simulation.dataWithNoise.front().size(), 0, 0, sizeof(Real));
    ImPlot::EndPlot();
  }
}
