#pragma once

#include <dr_eigen/eigen.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace dr {

/// Creates a spherical marker
visualization_msgs::Marker createSphereMarker(
	std::string const & frame_id,                          ///< The frame id in which the position is defined
	Eigen::Vector3d const & position,                      ///< The position of the marker in frame_id
	double radius = 0.1,                                   ///< The radius of the sphere
	std::string const & ns = "",                           ///< The namespace of the marker
	ros::Duration const & lifetime = ros::Duration(0),     ///< The lifetime of the marker
	std::array<float, 4> const & color = {{ 1, 0, 0, 1 }}, ///< The color of the marker in RGBA
	int id = 0                                             ///< The id of this marker
);

/// Creates a cylindrical marker
visualization_msgs::Marker createCylinderMarker(
	std::string const & frame_id,                          ///< The frame id in which the position is defined
	Eigen::Isometry3d const & pose,                        ///< The position of the marker in frame_id
	double radius,                                         ///< The radius of the cylinder
	double height,                                         ///< The height of the cylinder
	std::string const & ns,                                ///< The namespace of the marker
	ros::Duration const &lifetime = ros::Duration(0),      ///< The lifetime of the marker
	std::array<float, 4> const & color = {{ 1, 0, 0, 1 }}, ///< The color of the marker in RGBA
	int id = 0                                             ///< The id of this marker
);

/// Creates a box marker
visualization_msgs::Marker createBoxMarker(
	std::string const & frame_id,                          ///< The frame id in which the position is defined
	Eigen::AlignedBox3d const & box,                       ///< The position of the marker in frame_id
	std::string const & ns = "",                           ///< The namespace of the marker
	ros::Duration const & lifetime = ros::Duration(0),     ///< The lifetime of the marker
	std::array<float, 4> const & color = {{ 1, 0, 0, 1 }}, ///< The color of the marker in RGBA
	int id = 0                                             ///< The id of this marker
);

/// Creates a mesh marker
visualization_msgs::Marker createMeshMarker(
	std::string const & frame_id,                          ///< The frame id in which the position is defined
	Eigen::Isometry3d const & pose,                        ///< The pose of the marker in frame_id
	std::string const & mesh_resource,                     ///< Mesh resource of the marker (in "package://" syntax)
	std::string const & ns = "",                           ///< The namespace of the marker
	ros::Duration const & lifetime = ros::Duration(0),     ///< The lifetime of the marker
	std::array<float, 4> const & color = {{ 1, 0, 0, 1 }}, ///< The color of the marker in RGBA
	int id = 0                                             ///< The id of this marker
);

}
