#pragma once

#include <memory>
#include <string>
#include <vector>
#include "imgui.h"
#include "type_and_name.hpp"
#include "raii_file.hpp"

void drawCSVTable(const char *id, const std::vector<std::vector<Real>> &values,
                  ImVec2 size) {
  if (!values.empty() &&
      ImGui::BeginTable(id, values.size(), ImGuiTableFlags_ScrollY, size)) {
    for (const auto &col : values) {
      ImGui::TableSetupColumn(col.front().name.c_str());
    }
    ImGui::TableHeadersRow();

    auto rows = values.front().size();
    for (size_t row = 0; row < rows; row++) {
      ImGui::TableNextRow();
      for (size_t col = 0; col < values.size(); col++) {
        ImGui::TableNextColumn();
        ImGui::Text("%0.3f\n", values[col][rows - row - 1].value);
      }
    }

    ImGui::EndTable();
  }
}

class CSV {
 public:
  bool open(const std::string &path, const std::string &openMode) {
    csv = std::unique_ptr<FILE, FileCloser>(
        fopen(path.c_str(), openMode.c_str()));
    firstLine = true;
    return csv.get() != nullptr;
  }
  void write(const std::vector<Real> &values) {
    assert(csv && "File not opened");
    if (firstLine) {
      for (size_t i = 0; i < values.size(); i++) {
        if (i != values.size() - 1) {
          fprintf(csv.get(), "%s,", values[i].name.c_str());
        } else {
          fprintf(csv.get(), "%s\n", values[i].name.c_str());
        }
      }
    }
    for (size_t i = 0; i < values.size(); i++) {
      if (i != values.size() - 1) {
        fprintf(csv.get(), "%f,", values[i].value);
      } else {
        fprintf(csv.get(), "%f\n", values[i].value);
      }
    }
  }

 private:
  bool firstLine;
  RAIIFile csv;
};
