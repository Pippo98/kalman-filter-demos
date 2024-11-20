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
  Real(const char *_name, double std) : name(_name), noise(0.0, std) {}

  double operator=(double val) { return value = val; }
  double operator+(double val) { return value + val; }
  double operator-(double val) { return value - val; }
  double operator*(double val) { return value * val; }
  double operator/(const double &val) { return value / val; }

  operator double() const { return value; }
  operator double &() { return value; }
  operator const double &() const { return value; }

  Real getWithNoise() const {
    Real ret = *this;
    ret.value += ret.noise(RandomGenerator::get());
    return ret;
  };
  const Real &addNoise() {
    value += getWithNoise().value;
    return *this;
  }
};

#define REAL_TYPE(variableName, variance) \
  Real variableName{#variableName, variance};
