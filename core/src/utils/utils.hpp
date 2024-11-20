#pragma once

#include <vector>
#include "type_and_name.hpp"

using StatesValues = std::vector<std::vector<Real>>;
inline void rowToMatrix(StatesValues &hist, const std::vector<Real> values) {
  for (size_t i = 0; i < values.size(); i++) {
    if (hist.size() <= i) {
      hist.push_back(std::vector<Real>());
    }
    hist[i].push_back(values[i]);
  }
}
