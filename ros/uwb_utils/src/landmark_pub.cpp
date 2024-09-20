#include "uwb_utils/UWB.h"
#include "uwb_utils/parse_anchor_position.hpp"
#include <chrono>
#include <cstddef>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <memory>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <string>
#include <thread>
#include <unordered_map>
#include <visualization_msgs/Marker.h>

ros::Publisher ranges_pub;                                 // TODO: make it non global
geometry_msgs::Point robot_position;                       // TODO: make it non global
std::unordered_map<int, geometry_msgs::Point> coordinates; // TODO: make it non global

void robot_position_callback(const geometry_msgs::PointStamped::ConstPtr &msg) {
  robot_position = msg->point;
}

void range_callback(const uwb_utils::UWB::ConstPtr &msg) {
  if(coordinates.find(msg->id) == coordinates.end()) {
    return;
  }
  geometry_msgs::Point a = coordinates[msg->id];
  geometry_msgs::Point b = robot_position;
  geometry_msgs::Point u;
  u.x = (b.x - a.x);
  u.y = (b.y - a.y);
  u.z = (b.z - a.z);
  double k = msg->range / sqrt(u.x * u.x + u.y * u.y + u.z * u.z);
  b.x = a.x + k * u.x;
  b.y = a.y + k * u.y;
  b.z = a.z + k * u.z;

  visualization_msgs::Marker range_marker;
  range_marker.header.frame_id = "map";
  range_marker.header.stamp = ros::Time::now();
  range_marker.ns = "ranges";
  range_marker.id = msg->id;
  range_marker.lifetime = ros::Duration(0.5);
  range_marker.action = visualization_msgs::Marker::ADD;
  range_marker.type = visualization_msgs::Marker::LINE_STRIP;
  range_marker.points.push_back(a);
  range_marker.points.push_back(b);
  range_marker.scale.x = 0.01;
  range_marker.scale.y = 0.01;
  range_marker.scale.z = 0.01;
  range_marker.color.r = 1.0;
  range_marker.color.a = 1.0;
  ranges_pub.publish(range_marker);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "map_visualization");
  ros::NodeHandle nh;
  ros::Publisher landmark_pub = nh.advertise<visualization_msgs::Marker>("/uwb/visualization/landmarks", 10);
  ranges_pub = nh.advertise<visualization_msgs::Marker>("/uwb/visualization/ranges", 10);
  ros::Subscriber range_sub = nh.subscribe("/uwb/ranges", 10, range_callback);
  ros::Subscriber robot_sub = nh.subscribe("/uwb_localization/position", 10, robot_position_callback);

  visualization_msgs::Marker landmark_marker;
  landmark_marker.header.frame_id = "map";
  landmark_marker.header.stamp = ros::Time::now();
  landmark_marker.ns = "landmarks";
  landmark_marker.action = visualization_msgs::Marker::ADD;
  landmark_marker.type = visualization_msgs::Marker::POINTS;
  landmark_marker.id = 0;
  landmark_marker.scale.x = 0.1;
  landmark_marker.scale.y = 0.1;
  landmark_marker.color.g = 1.0;
  landmark_marker.color.a = 1.0;
  landmark_marker.points = get_anchor_positions(true);
  for(std::size_t i = 0; i < landmark_marker.points.size(); i++) {
    coordinates[static_cast<int>(i + 1)] = landmark_marker.points[i];
  }

  std::thread thread([&landmark_marker, &landmark_pub] {
    while(ros::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      landmark_pub.publish(landmark_marker);
    }
  });
  ros::spin();
  thread.join();

  return 0;
}
