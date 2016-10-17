#pragma once

#include <interactive_markers/interactive_marker_server.h>

namespace dr {

using InteractivePoseMarkerCallback = std::function<void(visualization_msgs::InteractiveMarkerFeedbackConstPtr const & feedback)>;

visualization_msgs::InteractiveMarkerControl createDefaultController(
	std::string const & frame_id,
	std::string const & mesh = ""
);

visualization_msgs::InteractiveMarker createInteractivePoseMarker(
	std::string const & frame_id,
	std::string const & name = "pose marker"
);

}
