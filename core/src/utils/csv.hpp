#include <memory>
#include <string>
#include <vector>
#include "type_and_name.hpp"

class CSV {
 public:
  bool open(const std::string &path, const std::string &openMode) {
    csv = fopen(path.c_str(), openMode.c_str());
    firstLine = true;
  }
  void write(const std::vector<Real> &values) {
    assert(csv, "File not opened");
    if (firstLine) {
      for (size_t i = 0; i < values.size(); i++) {
        if (i != values.size() - 1) {
          fwrite(csv, "%s,", values[i].name);
        } else {
          fwrite(csv, "%s\n", values[i].name);
        }
      }
    }
    for (size_t i = 0; i < values.size(); i++) {
      if (i != values.size() - 1) {
        fwrite(csv, "%f,", values[i]);
      } else {
        fwrite(csv, "%f\n", values[i]);
      }
    }
  }

 private:
  bool firstLine;
  RAIIFile csv;
};
