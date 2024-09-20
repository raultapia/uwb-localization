#ifndef UWB_LOCALIZATION_UWB_LOCALIZATION_HPP
#define UWB_LOCALIZATION_UWB_LOCALIZATION_HPP

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "uwb_utils/UWB.h"
#include <array>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <std_msgs/Header.h>

constexpr unsigned int NUM_ANCHORS = 6;

class EKF {
public:
  EKF(float x, float y, float z, float vx, float vy, float vz);
  void prediction();
  void update(const uwb_utils::UWB &uwb_msg);
  geometry_msgs::PointStamped asPointStamped();
  geometry_msgs::PoseStamped asPoseStamped();
  geometry_msgs::PoseWithCovarianceStamped asPoseWithCovarianceStamped();
  nav_msgs::Odometry asOdometry();
  nav_msgs::Path asPath();
  std::array<Eigen::Matrix<float, 3, 1>, NUM_ANCHORS> landmarks;

private:
  void update_(float rho2, float rho_p, int id, float dt);
  void update_(float rho2, int id);
  Eigen::Matrix<float, 2, 6> computeJacobian(float dt, int id);
  Eigen::Matrix<float, 1, 6> computeJacobian(int id);
  Eigen::Matrix<float, 2, 1> state2z(int id, float dt);
  Eigen::Matrix<float, 1, 1> state2z(int id);
  Eigen::Matrix<float, 6, 1> nu; // [x, y, z, vx, vy, vz]
  Eigen::Matrix<float, 6, 6> Sigma;
  Eigen::Matrix<float, 6, 6> G;
  Eigen::Matrix<float, 6, 6> R;
  Eigen::Matrix<float, 2, 2> Q;
  std::array<float, NUM_ANCHORS> last_rhos{};
  std::array<std::chrono::high_resolution_clock::time_point, NUM_ANCHORS> t_last_update;

  std::chrono::high_resolution_clock::time_point t_last_prediction;

  std_msgs::Header last_header;
};

#endif // UWB_LOCALIZATION_UWB_LOCALIZATION_HPP
