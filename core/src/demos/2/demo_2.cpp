#include "demo_2.hpp"

#include <memory>
#include "imgui.h"
#include "implot.h"
#include "models/cart_2d.hpp"
#include "simulator/simulation_manager.hpp"
#include "utils/csv.hpp"

Demo2::Demo2() {}
void Demo2::draw(SimulationData &sim) {
  Cart2D &cart = *dynamic_cast<Cart2D *>(sim.simulatable);
  ImVec2 reg = ImGui::GetContentRegionAvail();

  if (!sim.simulation.data.empty() &&
      ImPlot::BeginPlot("2D Cart position", reg, ImPlotFlags_Equal)) {
    ImPlot::SetupAxes("x", "y", ImPlotAxisFlags_AutoFit,
                      ImPlotAxisFlags_AutoFit);

    ImPlot::PlotLine("x", &sim.simulation.data[1].front().value,
                     &sim.simulation.data[2].front().value,
                     sim.simulation.data.front().size(), 0, 0, sizeof(Real));

    ImPlot::PlotScatter(
        "x measured", &sim.simulation.dataWithNoise[1].front().value,
        &sim.simulation.dataWithNoise[2].front().value,
        sim.simulation.dataWithNoise.front().size(), 0, 0, sizeof(Real));

    ImVec2 groundPoints[3] = {
        {0.0, 5.0},
        {float(cart.planeInclinationX), 5.0},
        {float(cart.planeInclinationX + 30.0 * std::cos(cart.alpha)),
         float(5.0 - 30 * std::sin(cart.alpha))}};
    ImPlot::PlotLine("ground", &groundPoints[0].x, &groundPoints[0].y, 3, 0, 0,
                     sizeof(ImVec2));
    ImPlot::EndPlot();
  }
}
