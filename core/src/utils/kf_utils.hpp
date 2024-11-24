#pragma once

#include <vector>
#include "Eigen/Core"
#include "imgui.h"
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
};

template <typename EigenVecOrMatrix>
bool drawMatrix(const char *id, EigenVecOrMatrix &mat,
                ImVec2 size = ImVec2(0.0f, 0.0f)) {
  bool modified = false;
  ImGui::BeginGroup();
  ImGui::TextUnformatted(id);
  if (ImGui::BeginTable(id, mat.cols(), ImGuiTableFlags_Borders, size)) {
    for (int row = 0; row < mat.rows(); row++) {
      ImGui::PushID(row);
      ImGui::TableNextRow();
      for (int col = 0; col < mat.cols(); col++) {
        ImGui::PushID(col);
        ImGui::TableNextColumn();
        ImGui::SetNextItemWidth(100);
        if (ImGui::InputDouble("##cell", &mat(row, col))) {
          modified = true;
        }
        if (mat.cols() == mat.rows()) {
          mat(col, row) = mat(row, col);
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

  {
    kfModified |= drawMatrix("Initial State", kfData.X0, {reg.x / 4.0f, 0.0f});
    ImGui::SameLine();
    kfModified |=
        drawMatrix("Initial State Cov", kfData.P0, {reg.x / 4.0f, 0.0f});
    ImGui::SameLine();
    kfModified |= drawMatrix("Process Cov", kfData.Q, {reg.x / 4.0f, 0.0f});
    ImGui::SameLine();
    kfModified |= drawMatrix("Measurement Cov", kfData.R, {reg.x / 4.0f, 0.0f});
  }
  return kfModified;
}
