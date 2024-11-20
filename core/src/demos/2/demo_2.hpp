#pragma once

#include "external/kflib/src/ukf.hpp"
#include "models/cart_2d.hpp"
#include "utils/utils.hpp"

class Demo2 : public Simulatable {
 public:
  Demo2();

  void draw();

  virtual void step(double dt) final {
    cart.step(dt);
    rowToMatrix(realStates, cart.getValues());
    rowToMatrix(measuredStates, cart.getValuesWithNoise());
  }

 private:
  Cart2D cart{0.0, 5.0, 1.0};

  StatesValues realStates;
  StatesValues measuredStates;
};
