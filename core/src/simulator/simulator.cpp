#include "simulator.hpp"
#include <cassert>
#include "utils/time_base.hpp"

Simulator::Simulator() { time = TimeBase::getTimestampSeconds(); }
void Simulator::setSystems(
    std::initializer_list<std::shared_ptr<Simulatable>> _systems) {
  systems = _systems;
}
void Simulator::step() {
  double dt = TimeBase::getTimestampSeconds() - time;
  time = TimeBase::getTimestampSeconds();
  step(dt);
}
void Simulator::step(double dt) {
  for (std::shared_ptr<Simulatable> &sim : systems) {
    assert(sim && "Simulatable not initialized");
    sim->step(dt);
  }
}
void Simulator::resetTime() { time = TimeBase::getTimestampSeconds(); }
