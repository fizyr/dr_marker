#pragma once

#include <dr_eigen/eigen.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace dr {

/// Creates a spherical marker
visualization_msgs::Marker createSphereMarker(
	std::string frame_id,                           ///< The frame id in which the position is defined
	Eigen::Vector3d position,                       ///< The position of the marker in frame_id
	double radius = 0.1,                            ///< The radius of the sphere
	std::string ns = "",                            ///< The namespace of the marker
	ros::Duration lifetime = ros::Duration(0),      ///< The lifetime of the marker
	std::array<float, 4> color = {{ 1, 0, 0, 1 }},  ///< The color of the sphere in RGBA
	int id = 0                                      ///< The id of this marker
);

/// Creates a cylindrical marker
visualization_msgs::Marker createCylinderMarker(
	std::string frame_id,                           ///< The frame id in which the position is defined
	Eigen::Isometry3d pose,                         ///< The pose of the marker in frame_id
	double radius,                                  ///< The radius of the cylinder
	double height,                                  ///< The height of the cylinder
	std::string ns,                                 ///< The namespace of the marker
	ros::Duration lifetime = ros::Duration(0),      ///< The lifetime of the marker
	std::array<float, 4> color = {{ 1, 0, 0, 1 }},  ///< The color of the cylinder in RGBA
	int id = 0                                      ///< The id of this marker
);

/// Creates a box marker
visualization_msgs::Marker createBoxMarker(
	std::string frame_id,                           ///< The frame id in which the position is defined
	Eigen::AlignedBox3d box,                        ///< The position of the marker in frame_id
	std::string ns = "",                            ///< The namespace of the marker
	ros::Duration lifetime = ros::Duration(0),      ///< The lifetime of the marker
	std::array<float, 4> color = {{ 1, 0, 0, 1 }},  ///< The color of the sphere in RGBA
	int id = 0                                      ///< The id of this marker
);

visualization_msgs::MarkerArray createAxesMarker(
	std::string frame_id = "/world",                             ///< The frame id in which the position is defined
	Eigen::Isometry3d pose = Eigen::Isometry3d::Identity(),      ///< The position of the marker in frame_id
	std::string ns = "",                                         ///< The namespace of the marker
	ros::Duration lifetime = ros::Duration(0)                    ///< The lifetime of the marker
);


}
