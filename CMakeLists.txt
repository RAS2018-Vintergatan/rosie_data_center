cmake_minimum_required(VERSION 2.8.3)
project(rosie_data_center)

find_package(catkin REQUIRED COMPONENTS
	 message_generation
	 roscpp
	 std_msgs
	 phidgets
	 nav_msgs
	 geometry_msgs
	 sensor_msgs
	 tf
	 visualization_msgs
	 pcl_ros
	)


add_message_files(
  FILES
)

add_service_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp std_msgs phidgets geometry_msgs sensor_msgs nav_msgs tf  pcl_ros visualization_msgs message_runtime
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosie_testing src/rosie_brain_testing.cpp)
add_executable(rosie_fetching src/rosie_brain_fetching.cpp)
#add_executable(rosie_exploring src/rosie_brain_exploring.cpp)
target_link_libraries(rosie_fetching ${catkin_LIBRARIES})
target_link_libraries(rosie_testing ${catkin_LIBRARIES})
#target_link_libraries(rosie_exploring ${catkin_LIBRARIES})
add_dependencies(rosie_fetching ${catkin_EXPORTED_TARGETS})
add_dependencies(rosie_testing ${catkin_EXPORTED_TARGETS})
#add_dependencies(rosie_exploring ${catkin_EXPORTED_TARGETS})
#std_msgs nav_msgs phidgets geometry_msgs sensor_msgs tf pcl_ros visualization_msgs
