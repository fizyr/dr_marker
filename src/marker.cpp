/**
 * Copyright 2014-2018 Fizyr BV. https://fizyr.com
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/

#include "marker.hpp"
#include <dr_eigen/ros.hpp>

namespace dr {

/// Creates a spherical marker
visualization_msgs::Marker createSphereMarker(
	std::string const & frame_id,       ///< The frame id in which the position is defined
	Eigen::Vector3d const & position,   ///< The position of the marker in frame_id
	double radius,                      ///< The radius of the sphere
	std::string const & ns,             ///< The namespace of the marker
	ros::Duration const & lifetime,     ///< The lifetime of the marker
	std::array<float, 4> const & color, ///< The color of the sphere in RGBA
	int id                              ///< The id of this marker
) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id;
	marker.header.stamp    = ros::Time::now();
	marker.id              = id;
	marker.pose.position.x = position.x();
	marker.pose.position.y = position.y();
	marker.pose.position.z = position.z();
	marker.color.r         = color[0];
	marker.color.g         = color[1];
	marker.color.b         = color[2];
	marker.color.a         = color[3];
	marker.scale.x         = radius;
	marker.scale.y         = radius;
	marker.scale.z         = radius;
	marker.lifetime        = lifetime;
	marker.type            = visualization_msgs::Marker::SPHERE;
	marker.action          = visualization_msgs::Marker::ADD;
	marker.ns              = ns;

	return marker;
}

/// Creates a cylindrical marker
visualization_msgs::Marker createCylinderMarker(
	std::string const & frame_id,       ///< The frame id in which the position is defined
	Eigen::Isometry3d const & pose,     ///< The position of the marker in frame_id
	double radius,                      ///< The radius of the cylinder
	double height,                      ///< The height of the cylinder
	std::string const & ns,             ///< The namespace of the marker
	ros::Duration const & lifetime,     ///< The lifetime of the marker
	std::array<float, 4> const & color, ///< The color of the cylinder in RGBA
	int id                              ///< The id of this marker
) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id;
	marker.header.stamp    = ros::Time::now();
	marker.id              = id;
	marker.pose            = toRosPose(pose);
	marker.color.r         = color[0];
	marker.color.g         = color[1];
	marker.color.b         = color[2];
	marker.color.a         = color[3];
	marker.scale.x         = radius * 2;
	marker.scale.y         = radius * 2;
	marker.scale.z         = height;
	marker.lifetime        = lifetime;
	marker.type            = visualization_msgs::Marker::CYLINDER;
	marker.action          = visualization_msgs::Marker::ADD;
	marker.ns              = ns;

	return marker;
}

/// Creates a box marker
visualization_msgs::Marker createBoxMarker(
	std::string const & frame_id,        ///< The frame id in which the position is defined
	Eigen::AlignedBox3d const & box,     ///< The position of the marker in frame_id
	std::string const & ns,              ///< The namespace of the marker
	ros::Duration const & lifetime,      ///< The lifetime of the marker
	std::array<float, 4> const & color,  ///< The color of the sphere in RGBA
	int id                               ///< The id of this marker
) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id;
	marker.header.stamp    = ros::Time::now();
	marker.id              = id;
	marker.pose.position.x = box.center().x();
	marker.pose.position.y = box.center().y();
	marker.pose.position.z = box.center().z();
	marker.color.r         = color[0];
	marker.color.g         = color[1];
	marker.color.b         = color[2];
	marker.color.a         = color[3];
	marker.scale.x         = box.sizes().x();
	marker.scale.y         = box.sizes().y();
	marker.scale.z         = box.sizes().z();
	marker.lifetime        = lifetime;
	marker.type            = visualization_msgs::Marker::CUBE;
	marker.action          = visualization_msgs::Marker::ADD;
	marker.ns              = ns;

	return marker;
}

/// Creates a mesh marker
visualization_msgs::Marker createMeshMarker(
	std::string const & frame_id,       ///< The frame id in which the position is defined
	Eigen::Isometry3d const & pose,     ///< The pose of the marker in frame_id
	std::string const & mesh_resource,  ///< Mesh resource of the marker (in "package://" syntax)
	std::string const & ns,             ///< The namespace of the marker
	ros::Duration const & lifetime,     ///< The lifetime of the marker
	std::array<float, 4> const & color, ///< The color of the marker in RGBA
	int id                              ///< The id of this marker
) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id;
	marker.header.stamp    = ros::Time::now();
	marker.id              = id;
	marker.pose            = toRosPose(pose);
	marker.color.r         = color[0];
	marker.color.g         = color[1];
	marker.color.b         = color[2];
	marker.color.a         = color[3];
	marker.scale.x         = 1.0;
	marker.scale.y         = 1.0;
	marker.scale.z         = 1.0;
	marker.lifetime        = lifetime;
	marker.type            = visualization_msgs::Marker::MESH_RESOURCE;
	marker.mesh_resource   = mesh_resource;
	marker.action          = visualization_msgs::Marker::ADD;
	marker.ns              = ns;

	return marker;
}

}
