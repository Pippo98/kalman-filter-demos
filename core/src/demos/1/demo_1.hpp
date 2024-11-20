#include <map>
#include "external/kflib/src/ukf.hpp"
#include "models/cart_1d.hpp"

class Demo1 : public Simulatable {
 public:
  Demo1();

  void draw();

  virtual void step(double dt) final {
    cart.step(dt);
    toHistory(realStates, cart.getValues());
    toHistory(measuredStates, cart.getValuesWithNoise());
  }

 private:
  Cart1D cart{0.0, 1.0};

  using StatesValues = std::vector<std::vector<Real>>;

  StatesValues realStates;
  StatesValues measuredStates;

  void toHistory(StatesValues &hist, const std::vector<Real> values) {
    for (size_t i = 0; i < values.size(); i++) {
      if (hist.size() <= i) {
        hist.push_back(std::vector<Real>());
      }
      hist[i].push_back(values[i]);
    }
  }
};
