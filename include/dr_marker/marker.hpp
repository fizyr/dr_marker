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
