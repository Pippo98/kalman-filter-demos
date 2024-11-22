#include "demo_3.hpp"

#include <memory>
#include "imgui.h"
#include "implot.h"
#include "models/cart_2d.hpp"
#include "simulator/simulation_manager.hpp"
#include "utils/csv.hpp"

Demo3::Demo3() {}
void Demo3::draw(SimulationData &sim) {
  ImVec2 reg = ImGui::GetContentRegionAvail();

  if (!sim.simulation.data.empty() &&
      ImPlot::BeginPlot("Ball position", reg, ImPlotFlags_Equal)) {
    ImPlot::SetupAxes("x", "y", ImPlotAxisFlags_AutoFit,
                      ImPlotAxisFlags_AutoFit);

    ImPlot::PlotLine("x", &sim.simulation.data[1].front().value,
                     &sim.simulation.data[2].front().value,
                     sim.simulation.data.front().size(), 0, 0, sizeof(Real));

    ImPlot::PlotScatter(
        "x measured", &sim.simulation.dataWithNoise[1].front().value,
        &sim.simulation.dataWithNoise[2].front().value,
        sim.simulation.dataWithNoise.front().size(), 0, 0, sizeof(Real));

    ImPlot::EndPlot();
  }
}
