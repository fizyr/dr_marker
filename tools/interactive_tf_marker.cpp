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

#include <ros/ros.h>

#include "pose_interactive_marker.hpp"
#include <dr_eigen/yaml.hpp>
#include <dr_eigen/ros.hpp>
#include <dr_param/param.hpp>

void processFeedback(visualization_msgs::InteractiveMarkerFeedbackConstPtr const & feedback) {
	if (feedback->control_name == "button" && feedback->event_type == feedback->MOUSE_DOWN) {
		ROS_INFO_STREAM(feedback->marker_name << " (frame_id: '" << feedback->header.frame_id << "') is now at \n" <<
			dr::toYaml(dr::toEigen(feedback->pose)) <<
			"\nTransform: \n" <<
			dr::toYaml(dr::toEigen(feedback->pose).inverse());
		);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "interactive_tf_marker");
	ros::NodeHandle node("~");

	std::string frame_id = dr::getParam<std::string>(node, "frame_id", "/world");

	// create marker and marker server
	visualization_msgs::InteractiveMarker interactive_marker = dr::createInteractivePoseMarker(frame_id);
	interactive_marker.controls.push_back(dr::createDefaultController(frame_id));
	interactive_markers::InteractiveMarkerServer server("pose");

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	server.insert(interactive_marker, processFeedback);

	// 'commit' changes and send to all clients
	server.applyChanges();

	ROS_INFO_STREAM("Node interactive_tf_marker started");

	// start the ROS main loop
	ros::spin();
}
