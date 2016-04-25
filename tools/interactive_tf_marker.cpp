#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <dr_eigen/yaml.hpp>
#include <dr_eigen/ros.hpp>
#include <dr_param/param.hpp>

#include "marker.hpp"

void processFeedback(
	const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	ROS_INFO_STREAM(feedback->marker_name << " (frame_id: '" << feedback->header.frame_id << "') is now at \n" <<
		dr::toYaml(dr::toEigen(feedback->pose)) <<
		"\nTransform: \n" <<
		dr::toYaml(dr::toEigen(feedback->pose).inverse());
	);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "interactive_tf_marker");
	ros::NodeHandle node("~");

	std::string frame_id = dr::getParam<std::string>(node, "frame_id", "/world");

	ROS_INFO_STREAM("Node interactive_tf_marker started");

	// create an interactive marker server on the topic namespace interactive_tf_marker
	interactive_markers::InteractiveMarkerServer server("interactive_tf_marker");

	// create an interactive marker for our server
	visualization_msgs::InteractiveMarker int_marker;
	int_marker.header.frame_id = frame_id;
	int_marker.header.stamp=ros::Time::now();
	int_marker.name = "axes_marker";
	int_marker.description = "Pose control";

	// create a grey marker array (each axis one marker)
	visualization_msgs::MarkerArray marker = dr::createAxesMarker(frame_id);

	// create a non-interactive control which contains the markers
	visualization_msgs::InteractiveMarkerControl control;
	control.always_visible = true;
	control.markers = marker.markers;

	// add the control to the interactive marker
	int_marker.controls.push_back( control );

	// create a control which will move the markers
	// this control does not contain any markers,
	// which will cause RViz to insert two arrows
	visualization_msgs::InteractiveMarkerControl rotate_control;

	// Move and rotate X
	rotate_control.orientation.w = 1;
	rotate_control.orientation.x = 1;
	rotate_control.orientation.y = 0;
	rotate_control.orientation.z = 0;
	rotate_control.name = "move_x";
	rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(rotate_control);
	rotate_control.name = "rotate_x";
	rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(rotate_control);

	// Move and rotate Y
	rotate_control.orientation.w = 1;
	rotate_control.orientation.x = 0;
	rotate_control.orientation.y = 1;
	rotate_control.orientation.z = 0;
	rotate_control.name = "move_y";
	rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(rotate_control);
	rotate_control.name = "rotate_y";
	rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(rotate_control);

	// Move and rotate Z
	rotate_control.orientation.w = 1;
	rotate_control.orientation.x = 0;
	rotate_control.orientation.y = 0;
	rotate_control.orientation.z = 1;
	rotate_control.name = "move_z";
	rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(rotate_control);
	rotate_control.name = "rotate_z";
	rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(rotate_control);

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	server.insert(int_marker, &processFeedback);

	// 'commit' changes and send to all clients
	server.applyChanges();

	// start the ROS main loop
	ros::spin();
}
