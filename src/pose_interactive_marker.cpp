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

#include "pose_interactive_marker.hpp"
#include "marker.hpp"

namespace dr {

visualization_msgs::InteractiveMarkerControl createDefaultController(
	std::string const & frame_id,
	std::string const & mesh
) {
	visualization_msgs::InteractiveMarkerControl control;
	control.always_visible   = true;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
	control.name             = "button";
	if (mesh != "") {
		control.markers.push_back(createMeshMarker(frame_id, Eigen::Isometry3d::Identity(), mesh));
	} else {
		control.markers.push_back(createSphereMarker(frame_id, {0, 0, 0}, 0.005));
	}

	return control;
}

visualization_msgs::InteractiveMarker createInteractivePoseMarker(
	std::string const & frame_id,
	std::string const & name
) {
	// create an interactive marker for our server
	visualization_msgs::InteractiveMarker marker;
	marker.scale           = 0.1;
	marker.header.frame_id = frame_id;
	marker.header.stamp    = ros::Time::now();
	marker.name            = name;
	marker.description     = name;

	// create a non-interactive control which contains the markers
	visualization_msgs::InteractiveMarkerControl control;
	control.always_visible   = true;

	// add the control to the interactive marker
	marker.controls.push_back(control);

	// Move and rotate X
	control.orientation.w    = 1;
	control.orientation.x    = 1;
	control.orientation.y    = 0;
	control.orientation.z    = 0;
	control.name             = "move_x";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	marker.controls.push_back(control);

	control.name             = "rotate_x";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	marker.controls.push_back(control);

	// Move and rotate Y
	control.orientation.w    = 1;
	control.orientation.x    = 0;
	control.orientation.y    = 1;
	control.orientation.z    = 0;
	control.name             = "move_y";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	marker.controls.push_back(control);

	control.name             = "rotate_y";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	marker.controls.push_back(control);

	// Move and rotate Z
	control.orientation.w    = 1;
	control.orientation.x    = 0;
	control.orientation.y    = 0;
	control.orientation.z    = 1;
	control.name             = "move_z";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	marker.controls.push_back(control);

	control.name             = "rotate_z";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	marker.controls.push_back(control);

	return marker;
}

}
