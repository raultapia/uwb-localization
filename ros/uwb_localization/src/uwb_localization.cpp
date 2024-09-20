#include "uwb_localization/uwb_localization.hpp"
#include <algorithm>
#include <cmath>
#include <ctime>
#include <eigen3/Eigen/LU>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/param.h>
#include <string>
#include <uwb_utils/UWB.h>
#include <vector>

extern float time_out_vel_upd;

EKF::EKF(float x, float y, float z, float vx, float vy, float vz) {
  float pos_std{0}, vel_std{0}, rho_std{0};
  ros::param::get("/pos_std", pos_std);
  ros::param::get("/vel_std", vel_std);
  ros::param::get("/rho_std", rho_std);

  nu << x, y, z, vx, vy, vz;

  G = Eigen::Matrix<float, 6, 6>::Identity();

  const float pos_var = powf(pos_std, 2);
  const float vel_var = powf(vel_std, 2);
  R << pos_var, 0, 0, 0, 0, 0,
      0, pos_var, 0, 0, 0, 0,
      0, 0, pos_var, 0, 0, 0,
      0, 0, 0, vel_var, 0, 0,
      0, 0, 0, 0, vel_var, 0,
      0, 0, 0, 0, 0, vel_var;

  Sigma = 10 * R;

  Q << powf(rho_std, 2), 0, 0, powf(rho_std, 2);

  t_last_prediction = std::chrono::high_resolution_clock::now();
  std::fill(t_last_update.begin(), t_last_update.end(), t_last_prediction);
  for(unsigned int i = 0; i < NUM_ANCHORS; i++) {
    const Eigen::Matrix<float, 3, 1> p = nu.block(0, 0, 3, 1);
    const Eigen::Matrix<float, 3, 1> l = landmarks.at(static_cast<std::size_t>(i));
    last_rhos.at(static_cast<std::size_t>(i)) = (p - l).transpose() * (p - l);
  }
}

Eigen::Matrix<float, 2, 6> EKF::computeJacobian(float dt, int id) {
  Eigen::Matrix<float, 3, 1> l = landmarks.at(static_cast<std::size_t>(id - 1));
  Eigen::Matrix<float, 2, 6> H;
  H << (nu(0) - l(0)), (nu(1) - l(1)), (nu(2) - l(2)), 0, 0, 0, (nu(0) - (nu(3) * dt) - l(0)), (nu(1) - (nu(4) * dt) - l(1)), (nu(2) - (nu(5) * dt) - l(2)), -dt * (nu(0) - nu(3) * dt - l(0)), -dt * (nu(1) - nu(4) * dt - l(1)), -dt * (nu(2) - nu(5) * dt - l(2));
  return 2 * H;
}

Eigen::Matrix<float, 1, 6> EKF::computeJacobian(int id) {
  Eigen::Matrix<float, 3, 1> l = landmarks.at(static_cast<std::size_t>(id - 1));
  Eigen::Matrix<float, 1, 6> H;
  H << (nu(0) - l(0)), (nu(1) - l(1)), (nu(2) - l(2)), 0, 0, 0;
  return 2 * H;
}

Eigen::Matrix<float, 2, 1> EKF::state2z(int id, float dt) {
  const Eigen::Matrix<float, 3, 1> p = nu.block(0, 0, 3, 1);
  const Eigen::Matrix<float, 3, 1> p_p = nu.block(0, 0, 3, 1) - nu.block(3, 0, 3, 1) * dt;
  const Eigen::Matrix<float, 3, 1> l = landmarks.at(static_cast<std::size_t>(id - 1));
  Eigen::Matrix<float, 2, 1> z_;
  z_ << (p - l).transpose() * (p - l), (p_p - l).transpose() * (p_p - l);
  return z_;
}

Eigen::Matrix<float, 1, 1> EKF::state2z(int id) {
  const Eigen::Matrix<float, 3, 1> p = nu.block(0, 0, 3, 1);
  const Eigen::Matrix<float, 3, 1> l = landmarks.at(static_cast<std::size_t>(id - 1));
  const Eigen::Matrix<float, 1, 1> z_;
  return (p - l).transpose() * (p - l);
}

void EKF::prediction() {
  const std::chrono::high_resolution_clock::time_point t = std::chrono::high_resolution_clock::now();
  const float dt = static_cast<float>(std::chrono::duration_cast<std::chrono::nanoseconds>(t - t_last_prediction).count()) * 1e-9F;

  nu(0) += nu(3) * dt;
  nu(1) += nu(4) * dt;
  nu(2) += nu(5) * dt;

  G(0, 3) = dt;
  G(1, 4) = dt;
  G(2, 5) = dt;

  Sigma = G * Sigma * G.transpose() + R;
  t_last_prediction = t;
}

void EKF::update_(float rho, float rho_p, int id, float dt) {
  Eigen::Matrix<float, 2, 1> z;
  z << powf(rho, 2), powf(rho_p, 2);
  Eigen::Matrix<float, 2, 6> H = computeJacobian(dt, id);
  const Eigen::Matrix<float, 2, 1> z_ = state2z(id, dt);
  Eigen::Matrix<float, 6, 2> K;
  K = Sigma * H.transpose() * (H * Sigma * H.transpose() + Q).inverse();
  nu = nu + K * (z - z_);
  Sigma = (Eigen::Matrix<float, 6, 6>::Identity() - K * H) * Sigma;
}

void EKF::update_(float rho, int id) {
  Eigen::Matrix<float, 1, 1> z;
  z << powf(rho, 2);
  Eigen::Matrix<float, 1, 6> H = computeJacobian(id);
  const Eigen::Matrix<float, 1, 1> z_ = state2z(id);
  Eigen::Matrix<float, 6, 1> K;
  K = Sigma * H.transpose() * (H * Sigma * H.transpose() + Q.block(0, 0, 1, 1)).inverse();
  nu = nu + K * (z - z_);
  Sigma = (Eigen::Matrix<float, 6, 6>::Identity() - K * H) * Sigma;
}

void EKF::update(const uwb_utils::UWB &uwb_msg) {
  const std::chrono::high_resolution_clock::time_point t = std::chrono::high_resolution_clock::now();
  const float dt = static_cast<float>(std::chrono::duration_cast<std::chrono::nanoseconds>(t - t_last_update.at(static_cast<std::size_t>(uwb_msg.id - 1))).count()) * 1e-9F;
  if(dt > time_out_vel_upd) {
    update_(uwb_msg.range, uwb_msg.id);
  } else {
    update_(uwb_msg.range, last_rhos.at(static_cast<std::size_t>(uwb_msg.id - 1)), uwb_msg.id, dt);
  }
  last_rhos.at(static_cast<std::size_t>(uwb_msg.id - 1)) = uwb_msg.range;
  t_last_update.at(static_cast<std::size_t>(uwb_msg.id - 1)) = t;
  last_header = uwb_msg.header;
}

// Messages:

geometry_msgs::PointStamped EKF::asPointStamped() {
  geometry_msgs::PointStamped point;
  point.header = last_header;
  point.header.frame_id = "map";
  point.point.x = nu(0);
  point.point.y = nu(1);
  point.point.z = nu(2);
  return point;
}

geometry_msgs::PoseStamped EKF::asPoseStamped() {
  geometry_msgs::PoseStamped p;
  const geometry_msgs::PointStamped point = asPointStamped();
  p.header = point.header;
  p.pose.position = point.point;
  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0;
  p.pose.orientation.w = 1;
  return p;
}

geometry_msgs::PoseWithCovarianceStamped EKF::asPoseWithCovarianceStamped() {
  geometry_msgs::PoseWithCovarianceStamped pc;
  const geometry_msgs::PoseStamped p = asPoseStamped();
  pc.header = p.header;
  pc.pose.pose = p.pose;
  pc.pose.covariance[0] = Sigma(0, 0);
  pc.pose.covariance[1] = Sigma(0, 1);
  pc.pose.covariance[2] = Sigma(0, 2);
  pc.pose.covariance[6] = Sigma(1, 0);
  pc.pose.covariance[7] = Sigma(1, 1);
  pc.pose.covariance[8] = Sigma(1, 2);
  pc.pose.covariance[12] = Sigma(2, 0);
  pc.pose.covariance[13] = Sigma(2, 1);
  pc.pose.covariance[14] = Sigma(2, 2);
  pc.pose.covariance[21] = 1;
  pc.pose.covariance[28] = 1;
  pc.pose.covariance[35] = 1;
  return pc;
}

nav_msgs::Odometry EKF::asOdometry() {
  nav_msgs::Odometry odom;
  const geometry_msgs::PoseWithCovarianceStamped pc = asPoseWithCovarianceStamped();
  odom.header = pc.header;
  odom.pose = pc.pose;
  return odom;
}

nav_msgs::Path EKF::asPath() {
  static nav_msgs::Path path;
  const geometry_msgs::PoseStamped p = asPoseStamped();

  if(!path.poses.empty()) {
    if(abs(p.header.stamp.toSec() - path.poses.back().header.stamp.toSec()) > 2) {
      path.poses.clear();
    }
  }

  path.header = p.header;
  path.poses.push_back(p);
  if(path.poses.size() > 50000) {
    path.poses.erase(path.poses.begin());
  }
  return path;
}
