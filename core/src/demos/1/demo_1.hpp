#include "external/kflib/src/ukf.hpp"
#include "models/cart_1d.hpp"

class Demo1 {
 public:
  Demo1() {};

  void draw();

 private:
  Cart1D cart{0.0, 1.0};
};
