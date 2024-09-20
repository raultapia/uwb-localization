#include "ros/ros.h"
#include <XmlRpcValue.h>
#include <geometry_msgs/Point.h>

std::vector<geometry_msgs::Point> get_anchor_positions(bool debug) {
  XmlRpc::XmlRpcValue anchors_param;
  ros::param::get("/anchors", anchors_param);
  std::vector<geometry_msgs::Point> ret(anchors_param.size());
  for(int i = 0; i < anchors_param.size(); ++i) {
    geometry_msgs::Point point;
    int id = static_cast<int>(anchors_param[i]["id"]);
    point.x = static_cast<double>(anchors_param[i]["x"]);
    point.y = static_cast<double>(anchors_param[i]["y"]);
    point.z = static_cast<double>(anchors_param[i]["z"]);
    ret[id - 1] = point;
    if(debug) ROS_INFO("ID: %d, X: %f, Y: %f, Z: %f", id, point.x, point.y, point.z);
  }
  return ret;
}
