cmake_minimum_required(VERSION 3.4)
project(turtlebot_mhe)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 1)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs aruco_localization)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_msgs nav_msgs aruco_localization
    )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIRS}
    ${CHOLMOD_INCLUDE_DIRS}
    include/
    )

file(GLOB_RECURSE PROJ_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} 
    FOLLOW_SYMLINKS src/*.cpp)
file(GLOB_RECURSE PROJ_INC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} 
    FOLLOW_SYMLINKS include/turtlebot_MHE/*.h)

add_executable(${PROJECT_NAME}
    ${PROJ_SRC}
    ${PROJ_INC}
    )
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
    )
