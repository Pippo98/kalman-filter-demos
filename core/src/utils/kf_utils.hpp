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

  void clearData() {
    states.clear();
    cov.clear();
    states.resize(ukf.getState().size() + 1);
    cov.resize(ukf.getState().size() + 1);
  }

  bool hasStates() { return !states.empty() && !states.front().empty(); }
  bool hasCov() { return !cov.empty() && !cov.front().empty(); }

  void initializeMatrices(size_t states, size_t measurements) {
    X0 = Eigen::VectorXd(states);
    P0 = Eigen::MatrixXd(states, states);
    Q = Eigen::MatrixXd(states, states);
    R = Eigen::MatrixXd(measurements, measurements);
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
