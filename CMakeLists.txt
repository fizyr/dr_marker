cmake_minimum_required(VERSION 3.8)
project(dr_marker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
	dr_eigen
	dr_param
	roscpp
	visualization_msgs
	geometry_msgs
	interactive_markers
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
)

include_directories(include/${PROJECT_NAME})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
	src/marker.cpp
	src/pose_interactive_marker.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(interactive_tf_marker
	tools/interactive_tf_marker.cpp
)
target_link_libraries(interactive_tf_marker ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(mesh_publisher
	tools/mesh_publisher.cpp
)
target_link_libraries(mesh_publisher ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
	catkin_add_gtest(${PROJECT_NAME}_test_marker test/marker.cpp)
endif()

target_link_libraries(${PROJECT_NAME}_test_marker ${catkin_LIBRARIES} ${PROJECT_NAME})

install(
	TARGETS "${PROJECT_NAME}" interactive_tf_marker mesh_publisher
	ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)

install(
	DIRECTORY "include/${PROJECT_NAME}/"
	DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
)
