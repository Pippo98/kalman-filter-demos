#include "simulator/simulator.hpp"

class Cart1D : public Simulatable {
 public:
  virtual void step(double dt) final;
};
