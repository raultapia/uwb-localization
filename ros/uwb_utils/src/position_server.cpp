#include "uwb_utils/AnchorService.h"
#include "uwb_utils/parse_anchor_position.hpp"
#include <cstddef>
#include <geometry_msgs/Point.h>
#include <memory>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <string>
#include <vector>

std::vector<geometry_msgs::Point> coordinates;

bool callback(uwb_utils::AnchorService::Request &req, uwb_utils::AnchorService::Response &res) {
  if(req.id > 0 && req.id <= coordinates.size()) {
    res.position = coordinates[static_cast<std::size_t>(req.id - 1)];
    res.success = true;
  } else {
    res.position.x = -1;
    res.position.y = -1;
    res.position.z = -1;
    res.success = false;
  }
  return res.success;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "anchor_position_server");
  ros::NodeHandle nh;
  coordinates = get_anchor_positions(false);
  nh.advertiseService("/uwb/anchor_position_server", callback);
  ros::spin();
  return 0;
}
