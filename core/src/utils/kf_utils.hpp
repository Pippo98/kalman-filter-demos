#pragma once

#include <algorithm>
#include <map>
#include <vector>
#include "Eigen/Core"
#include "imgui.h"
#include "implot.h"
#include "kflib/src/ukf.hpp"
#include "utils/csv.hpp"
#include "utils/type_and_name.hpp"

struct KFData {
  UnscentedKalmanFilter ukf;
  std::vector<std::vector<Real>> states;
  std::vector<std::vector<Real>> cov;
  Eigen::VectorXd X0;  // initial state
  Eigen::MatrixXd P0;  // initial state covariance
  Eigen::MatrixXd Q;   // process covariance
  Eigen::MatrixXd R;   // measureent covariance
  std::vector<std::string> statesNames;
  std::vector<std::string> measurementsNames;

  std::map<std::string, std::vector<Real>> residuals;

  void clearData() {
    states.clear();
    cov.clear();
    states.resize(ukf.getState().size() + 1);
    cov.resize(ukf.getState().size() + 1);
  }

  bool hasStates() { return !states.empty() && !states.front().empty(); }
  bool hasCov() { return !cov.empty() && !cov.front().empty(); }

  void initializeMatrices(const std::vector<std::string> &_statesNames,
                          const std::vector<std::string> &_measurementsNames) {
    statesNames = _statesNames;
    measurementsNames = _measurementsNames;
    X0 = Eigen::VectorXd(statesNames.size());
    P0 = Eigen::MatrixXd(statesNames.size(), statesNames.size());
    Q = Eigen::MatrixXd(statesNames.size(), statesNames.size());
    R = Eigen::MatrixXd(measurementsNames.size(), measurementsNames.size());
    X0.setZero();
    P0.setZero();
    Q.setZero();
    R.setZero();
  }
  void setMatrices() {
    ukf.setState(X0);
    ukf.setStateCovariance(P0);
    ukf.setProcessCovariance(Q);
    ukf.setMeasurementCovariance(R);
  }
  void addStateAndCovariance(double timestamp) {
    auto state = ukf.getState();
    const auto &covariance = ukf.getCovariance();

    states[0].push_back(Real("t", timestamp, 0.0));
    cov[0].push_back(Real("t", timestamp, 0.0));

    for (size_t i = 0; i < statesNames.size(); i++) {
      const std::string &n = statesNames[i];
      states[i + 1].push_back(Real(n.c_str(), state(i), 0.0));
      cov[i + 1].push_back(Real((n + "-" + n).c_str(), covariance(i, i), 0.0));
    }
  }
  void calculateResiduals(const std::string &state,
                          const std::vector<Real> &groundtruth) {
    auto itr = std::find(statesNames.begin(), statesNames.end(), state);
    if (itr == statesNames.end()) {
      return;
    }
    size_t idx = std::distance(statesNames.begin(), itr) + 1;
    const auto &vec = states[idx];
    if (vec.size() != groundtruth.size()) {
      return;
    }
    std::vector<Real> ret;
    for (size_t i = 0; i < vec.size(); i++) {
      ret.push_back(vec[i]);
      ret[i] = std::sqrt(std::pow(vec[i].value - groundtruth[i].value, 2));
    }
    residuals[state] = ret;
  }
};

template <typename EigenVecOrMatrix>
bool drawMatrix(const char *id, EigenVecOrMatrix &mat,
                const std::vector<std::string> &colNames,
                ImVec2 size = ImVec2(0.0f, 0.0f)) {
  bool modified = false;
  ImGui::BeginGroup();
  ImGui::TextUnformatted(id);
  bool showColNames = !colNames.empty() &&
                      std::is_same<EigenVecOrMatrix, Eigen::MatrixXd>::value;
  if (size.y == 0.0) {
    ImGui::PushFont(font_H3);
    size.y = showColNames *
                 (ImGui::GetFrameHeight() + ImGui::GetStyle().CellPadding.y) +
             mat.rows() * (ImGui::GetTextLineHeight() +
                           ImGui::GetStyle().CellPadding.y *
                               (2.0f + (showColNames ? 0.0 : 1.0)));
    ImGui::PopFont();
  }
  if (ImGui::BeginTable(id, mat.cols() + 1,
                        ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollX |
                            ImGuiTableFlags_HighlightHoveredColumn,
                        size)) {
    if (showColNames) {
      ImGui::TableSetupColumn("");
      for (const auto &col : colNames) {
        ImGui::TableSetupColumn(col.c_str());
      }
      ImGui::TableSetupScrollFreeze(1, 1);
      ImGui::PushStyleColor(ImGuiCol_Text, DRACULA_ACCENT);
      ImGui::PushFont(font_H3);
      ImGui::TableHeadersRow();
      ImGui::PopFont();
      ImGui::PopStyleColor();
    }
    for (int row = 0; row < mat.rows(); row++) {
      ImGui::PushID(row);
      ImGui::TableNextRow();
      for (int col = -1; col < mat.cols(); col++) {
        ImGui::PushID(col);
        ImGui::TableNextColumn();

        if (col == -1) {
          ImGui::PushStyleColor(ImGuiCol_Text, DRACULA_ACCENT);
          ImGui::PushFont(font_H3);
          ImGui::TextUnformatted(colNames[row].c_str());
          ImGui::PopFont();
          ImGui::PopStyleColor();
        } else {
          ImGui::SetNextItemWidth(120);
          if (ImGui::InputDouble("##cell", &mat(row, col), 0.1, 1.0, "%0.6f")) {
            modified = true;
          }
          if (mat.cols() == mat.rows()) {
            mat(col, row) = mat(row, col);
          }
        }
        ImGui::PopID();
      }
      ImGui::PopID();
    }

    ImGui::EndTable();
  }
  ImGui::EndGroup();
  return modified;
}
inline bool drawKFData(KFData &kfData) {
  ImVec2 reg = ImGui::GetContentRegionAvail();
  bool kfModified = false;
  if (ImGui::CollapsingHeader("KF states")) {
    drawCSVTable("KF states", kfData.states, {reg.x, reg.y / 5});
  }
  if (ImGui::CollapsingHeader("KF covariance")) {
    drawCSVTable("KF covariance", kfData.cov, {reg.x, reg.y / 5});
  }

  if (ImGui::CollapsingHeader("Matrices")) {
    kfModified |= drawMatrix("Initial State", kfData.X0, kfData.statesNames,
                             {reg.x / 4.0f, 0.0f});
    ImGui::SameLine();
    kfModified |= drawMatrix("Initial State Cov", kfData.P0, kfData.statesNames,
                             {reg.x / 4.0f, 0.0f});
    ImGui::SameLine();
    kfModified |= drawMatrix("Process Cov", kfData.Q, kfData.statesNames,
                             {reg.x / 4.0f, 0.0f});
    ImGui::SameLine();
    kfModified |= drawMatrix("Measurement Cov", kfData.R,
                             kfData.measurementsNames, {reg.x / 4.0f, 0.0f});
  }

  return kfModified;
}
inline void plotStatesOrCovariance(
    const std::string &prependName, const std::string &stateName,
    const std::vector<std::string> &names,
    const std::vector<std::vector<Real>> &statesOrCov) {
  auto iter = std::find(names.begin(), names.end(), stateName);
  if (iter == names.end()) {
    return;
  }
  size_t idx = std::distance(names.begin(), iter) + 1;
  const auto &xVec = statesOrCov[0];
  const auto &yVec = statesOrCov[idx];
  ImPlot::PlotLine((prependName + "." + stateName).c_str(), &xVec[0].value,
                   &yVec[0].value, xVec.size(), 0, 0, sizeof(xVec[0]));
}
inline void plotKFState(const std::string &prependName,
                        const std::string &stateName, KFData &kfData) {
  if (!kfData.hasStates()) {
    return;
  }
  plotStatesOrCovariance(prependName, stateName, kfData.statesNames,
                         kfData.states);
}
inline void plotKFCovariance(const std::string &prependName,
                             const std::string &stateName, KFData &kfData) {
  if (!kfData.hasCov()) {
    return;
  }
  plotStatesOrCovariance(prependName, stateName, kfData.statesNames,
                         kfData.cov);
}
