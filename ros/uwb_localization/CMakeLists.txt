cmake_minimum_required(VERSION 3.5)
project(uwb_localization)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs uwb_utils)
catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs nav_msgs uwb_utils)

add_executable(uwb_localization_node src/uwb_localization_node.cpp src/uwb_localization.cpp)
add_dependencies(uwb_localization_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(uwb_localization_node ${catkin_LIBRARIES})
target_include_directories(uwb_localization_node PRIVATE include)
target_include_directories(uwb_localization_node SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
