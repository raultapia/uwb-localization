#include "serial/serial.h"
#include "uwb_utils/UWB.h"
#include <chrono>
#include <iostream>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <sstream>
#include <stdint.h>
#include <string>
#include <thread>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "serial_arduino_node");
  ros::NodeHandle nh;
  std::string port = "/dev/ttyACM0";
  ros::param::get("~port", port);

  ros::Publisher rangesPub = nh.advertise<uwb_utils::UWB>("/uwb/ranges", 1000);

  serial::Serial my_serial(port, 115200, serial::Timeout::simpleTimeout(3000));

  if(my_serial.isOpen()) {
    std::cout << "Port opened succesfully" << std::endl;
  } else {
    std::cout << "Port failed to open" << std::endl;
  }

  uwb_utils::UWB uwb_msg;

  while(true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    my_serial.flushInput();
    std::string response = my_serial.read(8);
    // std::cout << response << std::endl;

    std::stringstream ss(response);

    uint8_t id;
    char atSymbol;
    float range;

    ss >> id >> atSymbol >> range;
    if(atSymbol != '@') {
      continue;
    }

    std::cout << "id: " << id << " range: " << range << std::endl;

    uwb_msg.header.stamp = ros::Time::now();
    uwb_msg.id = static_cast<unsigned char>(id - 48);
    uwb_msg.range = range;

    rangesPub.publish(uwb_msg);
  }
}
