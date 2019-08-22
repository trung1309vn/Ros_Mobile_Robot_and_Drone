cmake_minimum_required(VERSION 3.5.1)
project(subt_seed)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

set (SUBT_MAJOR_VERSION 0)
set (SUBT_MINOR_VERSION 1)
set (SUBT_PATCH_VERSION 0)
set (SUBT_VERSION_FULL
  ${SUBT_MAJOR_VERSION}.${SUBT_MINOR_VERSION}.${SUBT_PATCH_VERSION})

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    subt_ign
    roscpp
)

### find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

########################
## Message generation ##
########################

# Add here SubT ROS messages.

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

## Declare a C++ executable
add_executable(robot_move_freely src/robot_move_freely.cc)
add_dependencies(robot_move_freely ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_move_freely ${catkin_LIBRARIES})

add_executable(robot_move_entrance src/robot_move_entrance.cc)
add_dependencies(robot_move_entrance ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_move_entrance ${catkin_LIBRARIES})

add_executable(drone_move_freely src/drone_move_freely.cc)
add_dependencies(drone_move_freely ${catkin_EXPORTED_TARGETS})
target_link_libraries(drone_move_freely ${catkin_LIBRARIES})

add_executable(drone_move_entrance src/drone_move_entrance.cc)
add_dependencies(drone_move_entrance ${catkin_EXPORTED_TARGETS})
target_link_libraries(drone_move_entrance ${catkin_LIBRARIES})

###########
## Tests ##
###########


#############
## Install ##
#############

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Mark executables and/or libraries for installation
install(TARGETS robot_move_freely robot_move_entrance drone_move_freely drone_move_entrance
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
