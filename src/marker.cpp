#include "marker.hpp"
#include <dr_eigen/ros.hpp>
#include <eigen_conversions/eigen_msg.h>

namespace dr {

/// Creates a spherical marker
visualization_msgs::Marker createSphereMarker(
	std::string frame_id,        ///< The frame id in which the position is defined
	Eigen::Vector3d position,    ///< The position of the marker in frame_id
	double radius,               ///< The radius of the sphere
	std::string ns,              ///< The namespace of the marker
	ros::Duration lifetime,      ///< The lifetime of the marker
	std::array<float, 4> color,  ///< The color of the sphere in RGBA
	int id                       ///< The id of this marker
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
	std::string frame_id,        ///< The frame id in which the position is defined
	Eigen::Isometry3d pose,      ///< The position of the marker in frame_id
	double radius,               ///< The radius of the cylinder
	double height,               ///< The height of the cylinder
	std::string ns,              ///< The namespace of the marker
	ros::Duration lifetime,      ///< The lifetime of the marker
	std::array<float, 4> color,  ///< The color of the cylinder in RGBA
	int id                       ///< The id of this marker
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
	std::string frame_id,        ///< The frame id in which the position is defined
	Eigen::AlignedBox3d box,     ///< The position of the marker in frame_id
	std::string ns,              ///< The namespace of the marker
	ros::Duration lifetime,      ///< The lifetime of the marker
	std::array<float, 4> color,  ///< The color of the sphere in RGBA
	int id                       ///< The id of this marker
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

visualization_msgs::MarkerArray createAxesMarker(
	std::string frame_id,        ///< The frame id in which the position is defined
	Eigen::Isometry3d pose,      ///< The position of the marker in frame_id
	std::string ns,              ///< The namespace of the marker
	ros::Duration lifetime       ///< The lifetime of the marker
){
	// creating rviz markers
	visualization_msgs::Marker z_axes, y_axes, x_axes, line;
	visualization_msgs::MarkerArray markers_msg;
	
	z_axes.type = y_axes.type = x_axes.type = visualization_msgs::Marker::LINE_LIST;
	z_axes.ns = y_axes.ns = x_axes.ns = ns;
	z_axes.action = y_axes.action = x_axes.action = visualization_msgs::Marker::ADD;
	z_axes.lifetime = y_axes.lifetime = x_axes.lifetime = lifetime;
	z_axes.header.frame_id = y_axes.header.frame_id = x_axes.header.frame_id = frame_id;
	z_axes.scale.x = y_axes.scale.x = x_axes.scale.x = 0.01;
	
	// z properties
	z_axes.id = 0;
	z_axes.color.r = 0;
	z_axes.color.g = 0;
	z_axes.color.b = 1;
	z_axes.color.a = 1;
	
	// y properties
	y_axes.id = 1;
	y_axes.color.r = 0;
	y_axes.color.g = 1;
	y_axes.color.b = 0;
	y_axes.color.a = 1;
	
	// x properties
	x_axes.id = 2;
	x_axes.color.r = 1;
	x_axes.color.g = 0;
	x_axes.color.b = 0;
	x_axes.color.a = 1;
	
	geometry_msgs::Point p_start,p_end;
	tf::pointEigenToMsg(pose.translation(),p_start);
	
	Eigen::Affine3d moved_along_x = pose * Eigen::Translation3d(0.1, 0,0);
	tf::pointEigenToMsg(moved_along_x.translation(),p_end);
	x_axes.points.push_back(p_start);
	x_axes.points.push_back(p_end);

	Eigen::Affine3d moved_along_y = pose * Eigen::Translation3d(0, 0.1, 0);
	tf::pointEigenToMsg(moved_along_y.translation(),p_end);
	y_axes.points.push_back(p_start);
	y_axes.points.push_back(p_end);

	Eigen::Affine3d moved_along_z = pose * Eigen::Translation3d(0,0, 0.1);
	tf::pointEigenToMsg(moved_along_z.translation(),p_end);
	z_axes.points.push_back(p_start);
	z_axes.points.push_back(p_end);

	markers_msg.markers.push_back(x_axes);
	markers_msg.markers.push_back(y_axes);
	markers_msg.markers.push_back(z_axes);
	return markers_msg;
}

}
