cmake_minimum_required(VERSION 2.8.3)
project(path_planning)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  message_generation
  pcl_ros
  pcl_conversions
  dynamic_reconfigure
  geographic_msgs
  octomap_msgs
)

find_package(Boost)
find_package(PCL 1.8 REQUIRED COMPONENTS
    common
    sample_consensus
    io
    ros
    segmentation
    filters)

find_package(octomap 1.7.0 REQUIRED)

if (octomap_FOUND)
    message(STATUS "---- FOUND OctoMap ----")
    message(STATUS "OctoMap VERSION: ${OCTOMAP_VERSION}")
    message(${octomap_VERSION})
else()
    message(FATAL_ERROR "OctoMap not found!")
endif()
add_definitions(-DOCTOMAP_NODEBUGOUT)
find_package(cmake_modules REQUIRED)

find_package(dlib REQUIRED)

generate_messages(
    DEPENDENCIES
    std_msgs
    geographic_msgs
)

catkin_package(
    INCLUDE_DIRS
    LIBRARIES 
    CATKIN_DEPENDS 
    pcl_ros
    DEPENDS octomap PCL)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    )

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${OCTOMAP_LIBRARY_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Dlib_LIBRARIES})
add_definitions(
    ${PCL_DEFINITIONS})  

set(LINK_LIBS
${catkin_LIBRARIES}
	
    ${OCTOMAP_LIBRARIES}
    
    ${PCL_LIBRARIES}
)
##
add_executable(planning src/planning_a.cpp)
target_link_libraries(planning  ${LINK_LIBS})

add_executable(b_planning src/planning_b.cpp)
target_link_libraries(b_planning  ${LINK_LIBS})

add_executable(planning_c src/planning_c.cpp)
target_link_libraries(planning_c  ${LINK_LIBS})

add_executable(planning_d src/planning_d.cpp)
target_link_libraries(planning_d  ${LINK_LIBS})
##
add_executable(submap src/submap.cpp)
target_link_libraries(submap  ${LINK_LIBS})
##
add_executable(test_planning_c src/test_planning_c.cpp)
target_link_libraries(test_planning_c  ${LINK_LIBS})

add_executable(test_planning_d src/test_planning_d.cpp)
target_link_libraries(test_planning_d  ${LINK_LIBS})
##
add_executable(read_octomapFile src/read_octomapFile.cpp)
target_link_libraries(read_octomapFile  ${LINK_LIBS})

add_executable(check_cell src/check_cell.cpp)
target_link_libraries(check_cell  ${LINK_LIBS})

add_executable(visual_cell src/visual_cell.cpp)
target_link_libraries(visual_cell  ${LINK_LIBS})

add_executable(convert src/convert.cpp)
target_link_libraries(convert  ${LINK_LIBS})

install(DIRECTORY launch src config include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS src/script/save_data.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)