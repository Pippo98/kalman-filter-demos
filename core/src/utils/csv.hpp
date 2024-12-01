#pragma once

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include "imgui.h"
#include "type_and_name.hpp"
#include "raii_file.hpp"
#include "utils/styles.hpp"

inline void drawCSVTable(const char *id,
                         const std::vector<std::vector<Real>> &values,
                         ImVec2 size) {
  if (!values.empty() && !values.front().empty() &&
      ImGui::BeginTable(id, values.size(),
                        ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg |
                            ImGuiTableFlags_Borders |
                            ImGuiTableFlags_HighlightHoveredColumn,
                        size)) {
    for (const auto &col : values) {
      ImGui::TableSetupColumn(col.front().name.c_str());
    }
    ImGui::TableSetupScrollFreeze(values.size(), 1);
    ImGui::PushStyleColor(ImGuiCol_Text, DRACULA_ACCENT);
    ImGui::PushFont(font_H3);
    ImGui::TableHeadersRow();
    ImGui::PopFont();
    ImGui::PopStyleColor();

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
    header.clear();
    return csv.get() != nullptr;
  }
  void close() { csv = nullptr; }
  void writeLine(const std::vector<Real> &values) {
    assert(csv && "File not opened");
    assert(!values.empty() && "Empty line");
    if (header.empty()) {
      header = values;
      for (size_t i = 0; i < values.size(); i++) {
        fprintf(csv.get(), "%s@%f", values[i].name.c_str(),
                values[i].noise.stddev());
        if (i != values.size() - 1) {
          fprintf(csv.get(), ",");
        } else {
          fprintf(csv.get(), "\n");
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
  std::vector<Real> readLine() {
    assert(csv && "File not opened");
    std::vector<Real> row;
    std::string cell;
    cell.reserve(100);
    size_t column = 0;
    if (!header.empty()) {
      row = header;
    }
    while (true) {
      char c = fgetc(csv.get());
      if (c <= 0) {
        break;
      }
      if (c == ',' || c == '\n') {
        if (header.empty()) {
          auto pos = cell.find('@');
          if (pos == std::string::npos) {
            row.push_back(Real(cell.c_str(), 0.0));
          } else {
            std::string name = cell.substr(0, pos);
            std::string stdStr = cell.substr(pos + 1);
            row.push_back(
                Real(name.c_str(), std::strtod(stdStr.c_str(), nullptr)));
          }
        } else {
          row[column].value = std::strtod(cell.c_str(), nullptr);
        }
        column++;
        cell.clear();

        if (c == '\n') {
          break;
        }
      } else {
        cell += c;
      }
    }
    if (header.empty()) {
      header = row;
    }
    if (column != header.size()) {
      row.clear();
    }
    return row;
  }

 private:
  std::vector<Real> header;
  RAIIFile csv;
};
