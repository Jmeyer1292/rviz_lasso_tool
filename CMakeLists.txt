cmake_minimum_required(VERSION 2.8.3)
project(rviz_lasso_tool)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# include_directories(${catkin_INCLUDE_DIRS})
# link_directories(${catkin_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS
  rviz
  pcl_ros
  eigen_conversions
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  UserSelection.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rviz message_runtime pcl_ros eigen_conversions geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${pcl_INCLUDE_DIRS}
)

# QT Stuff
add_definitions(-DQT_NO_KEYWORDS)

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

# QMOC
qt5_wrap_cpp(MOC_FILES include/${PROJECT_NAME}/rviz_lasso_tool.h)

add_library(${PROJECT_NAME}
 src/${PROJECT_NAME}/rviz_lasso_tool.cpp
 src/${PROJECT_NAME}/selection_visualization.cpp
 src/${PROJECT_NAME}/ramer_douglas_peucker_simplification.cpp
 src/${PROJECT_NAME}/point_cloud_test.cpp
 ${MOC_FILES}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
