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
