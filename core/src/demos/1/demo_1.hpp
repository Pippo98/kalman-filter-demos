#pragma once

#include "external/kflib/src/ukf.hpp"
#include "utils/utils.hpp"
#include "models/cart_1d.hpp"

class Demo1 : public Simulatable {
 public:
  Demo1();

  void draw();

  virtual void step(double dt) final {
    cart.step(dt);
    rowToMatrix(realStates, cart.getValues());
    rowToMatrix(measuredStates, cart.getValuesWithNoise());
  }

 private:
  Cart1D cart{0.0, 1.0};

  StatesValues realStates;
  StatesValues measuredStates;
};
