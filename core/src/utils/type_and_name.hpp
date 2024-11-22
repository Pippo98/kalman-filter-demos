#pragma once

#include <random>
#include <string>

class RandomGenerator {
 private:
  RandomGenerator() {};
  std::default_random_engine engine;

 public:
  static std::default_random_engine &get() {
    static RandomGenerator uniqueInstance;
    return uniqueInstance.engine;
  }
};

struct Real {
 public:
  std::string name;
  double value;
  std::normal_distribution<double> noise;

  Real(const char *_name) : Real(_name, 0.0) {};
  Real(const char *_name, double std)
      : name(_name), value(0.0), noise(0.0, std) {}
  Real getWithNoise() const {
    Real ret = *this;
    ret.value += ret.noise(RandomGenerator::get());
    return ret;
  };
  const Real &addNoise() {
    value += getWithNoise().value;
    return *this;
  }

  double operator=(double val) { return value = val; }
  double operator+(double val) { return value + val; }
  double operator-(double val) { return value - val; }
  double operator*(double val) { return value * val; }
  double operator/(const double &val) { return value / val; }

  operator double() const { return value; }
  operator double &() { return value; }
  operator const double &() const { return value; }

  template <size_t I>
  auto &get() & {
    if constexpr (I == 0)
      return name;
    else if constexpr (I == 1)
      return value;
    else if constexpr (I == 2)
      return noise;
  }

  template <size_t I>
  auto const &get() const & {
    if constexpr (I == 0)
      return name;
    else if constexpr (I == 1)
      return value;
    else if constexpr (I == 2)
      return noise;
  }
};

namespace std {
template <>
struct tuple_size<Real> : std::integral_constant<size_t, 3> {};

template <>
struct tuple_element<0, Real> {
  using type = decltype(Real::name);
};
template <>
struct tuple_element<1, Real> {
  using type = decltype(Real::value);
};
template <>
struct tuple_element<2, Real> {
  using type = decltype(Real::noise);
};
}  // namespace std

#define REAL_TYPE(variableName, variance) \
  Real variableName{#variableName, variance};
