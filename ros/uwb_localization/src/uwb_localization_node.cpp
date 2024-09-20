#include "uwb_localization/uwb_localization.hpp"
#include <array>
#include <cstddef>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/publisher.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <string>
#include <uwb_utils/AnchorService.h>
#include <uwb_utils/UWB.h>

float time_out_vel_upd;

EKF *uwbEKF;
ros::Publisher position_pub, covariance_pub, odometry_pub, path_pub;

void uwbCallback(const uwb_utils::UWB::ConstPtr &msg) {
  uwbEKF->prediction();
  uwbEKF->update(*msg);
  position_pub.publish(uwbEKF->asPointStamped());
  covariance_pub.publish(uwbEKF->asPoseWithCovarianceStamped());
  odometry_pub.publish(uwbEKF->asOdometry());
  path_pub.publish(uwbEKF->asPath());
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "estimator");

  ros::NodeHandle nh("~");

  std::string uwb_topic;
  ros::param::get("/topic", uwb_topic);
  ros::param::get("/time_out_vel_upd", time_out_vel_upd);

  // Filter object. Input: [x_0 y_0 z_0 vx_0 vy_0 vz_0]
  uwbEKF = new EKF(4.37F, 4.37F, 0.0, 0, 0, 0);

  ros::ServiceClient landmark_client = nh.serviceClient<uwb_utils::AnchorService>("/uwb/anchor_position_server");
  landmark_client.waitForExistence();

  // Set landmark positions
  uwb_utils::AnchorService landmark_srv;
  for(unsigned int i = 1; i <= NUM_ANCHORS; i++) {
    landmark_srv.request.id = static_cast<unsigned char>(i);
    landmark_client.call(landmark_srv);
    if(!static_cast<bool>(landmark_srv.response.success)) {
      continue;
    }
    Eigen::Matrix<float, 3, 1> l;
    l << static_cast<float>(landmark_srv.response.position.x), static_cast<float>(landmark_srv.response.position.y), static_cast<float>(landmark_srv.response.position.z);
    uwbEKF->landmarks.at(static_cast<std::size_t>(i - 1)) = l;
  }

  const ros::Subscriber sub = nh.subscribe(uwb_topic, 1, uwbCallback);

  position_pub = nh.advertise<geometry_msgs::PointStamped>("position", 0);
  covariance_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("covariance", 0);
  odometry_pub = nh.advertise<nav_msgs::Odometry>("odometry", 0);
  path_pub = nh.advertise<nav_msgs::Path>("path", 0);

  ros::spin();

  return 0;
}
