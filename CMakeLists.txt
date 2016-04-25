cmake_minimum_required(VERSION 2.8.3)
project(dr_marker)

find_package(catkin REQUIRED COMPONENTS
	dr_base
	dr_eigen
	dr_param
	roscpp
	visualization_msgs
	geometry_msgs
	interactive_markers
	eigen_conversions
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
)

dr_include_directories(
	include/${PROJECT_NAME}
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/marker.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(interactive_tf_marker
	tools/interactive_tf_marker.cpp
	src/marker.cpp
)

target_link_libraries(interactive_tf_marker ${catkin_LIBRARIES})

dr_add_gtest(marker test/marker.cpp)

target_link_libraries(${PROJECT_NAME}_test_marker ${catkin_LIBRARIES} ${PROJECT_NAME})

install(
	TARGETS "${PROJECT_NAME}" interactive_tf_marker
	ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)

install(
	DIRECTORY "include/${PROJECT_NAME}/"
	DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
)
