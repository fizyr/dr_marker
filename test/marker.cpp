#include "marker.hpp"

#include <gtest/gtest.h>
#include <dr_eigen/ros.hpp>
#include <dr_eigen/eigen.hpp>
#include <dr_eigen/test/compare.hpp>
#include <dr_util/geometry.hpp>

int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(MarkerTest, makeSphereMarker) {
	ros::Time::init();

	ros::Time now = ros::Time::now();

	std::string frame_id = "some_frame";
	Eigen::Vector3d position(0.5, 0.5, 0.5); // some position
	double radius = 0.5;
	std::string ns = "some_namespace";
	ros::Duration lifetime = ros::Duration(0.5); // 0.5s lifetime
	std::array<float, 4> color({{0.3f, 0.4f, 0.5f, 0.6f}});
	int id = 100;

	visualization_msgs::Marker marker = createSphereMarker(
		"some_frame",
		position,
		radius,
		ns,
		lifetime,
		color,
		id
	);

	// expect the supplied output to be the same
	EXPECT_EQ(marker.header.frame_id, frame_id);
	EXPECT_TRUE(marker.header.stamp.toSec() >= now.toSec());
	EXPECT_TRUE(marker.header.stamp.toSec() <= ros::Time::now().toSec());
	EXPECT_EQ(marker.id, id);

	EXPECT_TRUE(testEqual(position, toEigen(marker.pose.position)));

	EXPECT_FLOAT_EQ(marker.color.r, color[0]);
	EXPECT_FLOAT_EQ(marker.color.g, color[1]);
	EXPECT_FLOAT_EQ(marker.color.b, color[2]);
	EXPECT_FLOAT_EQ(marker.color.a, color[3]);

	EXPECT_DOUBLE_EQ(marker.scale.x, radius);
	EXPECT_DOUBLE_EQ(marker.scale.y, radius);
	EXPECT_DOUBLE_EQ(marker.scale.z, radius);

	EXPECT_DOUBLE_EQ(marker.lifetime.toSec(), 0.5);
	EXPECT_EQ(marker.type, visualization_msgs::Marker::SPHERE);
	EXPECT_EQ(marker.action, visualization_msgs::Marker::ADD);
	EXPECT_EQ(marker.ns, ns);
}

TEST(MarkerTest, makeCylinderMarker) {
	ros::Time::init();

	ros::Time now = ros::Time::now();

	std::string frame_id = "some_frame";
	Eigen::AngleAxisd rotation = rotate_y(dr::pi());
	Eigen::Isometry3d pose = translate(0.5, 0.5, 0.5) * rotation;
	double radius = 0.5;
	double height = 1.0;
	std::string ns = "some_namespace";
	ros::Duration lifetime = ros::Duration(0.5); // 0.5s lifetime
	std::array<float, 4> color({{0.3f, 0.4f, 0.5f, 0.6f}});
	int id = 100;

	visualization_msgs::Marker marker = createCylinderMarker(
		"some_frame",
		pose,
		radius,
		height,
		ns,
		lifetime,
		color,
		id
	);

	// expect the supplied output to be the same
	EXPECT_EQ(marker.header.frame_id, frame_id);
	EXPECT_TRUE(marker.header.stamp.toSec() >= now.toSec());
	EXPECT_TRUE(marker.header.stamp.toSec() <= ros::Time::now().toSec());
	EXPECT_EQ(marker.id, id);

	EXPECT_TRUE(testEqual(pose.translation(), toEigen(marker.pose.position)));

	EXPECT_FLOAT_EQ(marker.color.r, color[0]);
	EXPECT_FLOAT_EQ(marker.color.g, color[1]);
	EXPECT_FLOAT_EQ(marker.color.b, color[2]);
	EXPECT_FLOAT_EQ(marker.color.a, color[3]);

	EXPECT_DOUBLE_EQ(marker.scale.x, radius * 2);
	EXPECT_DOUBLE_EQ(marker.scale.y, radius * 2);
	EXPECT_DOUBLE_EQ(marker.scale.z, radius);

	EXPECT_DOUBLE_EQ(marker.lifetime.toSec(), 0.5);
	EXPECT_EQ(marker.type, visualization_msgs::Marker::CYLINDER);
	EXPECT_EQ(marker.action, visualization_msgs::Marker::ADD);
	EXPECT_EQ(marker.ns, ns);
}

TEST(MarkerTest, makeBoxMarker) {
	ros::Time::init();

	ros::Time now = ros::Time::now();

	std::string frame_id = "some_frame";
	Eigen::AlignedBox3d box;
	std::string ns = "some_namespace";
	ros::Duration lifetime = ros::Duration(0.5); // 0.5s lifetime
	std::array<float, 4> color({{0.3f, 0.4f, 0.5f, 0.6f}});
	int id = 100;

	visualization_msgs::Marker marker = createBoxMarker(
		"some_frame",
		box,
		ns,
		lifetime,
		color,
		id
	);

	// expect the supplied output to be the same
	EXPECT_EQ(marker.header.frame_id, frame_id);
	EXPECT_TRUE(marker.header.stamp.toSec() >= now.toSec());
	EXPECT_TRUE(marker.header.stamp.toSec() <= ros::Time::now().toSec());
	EXPECT_EQ(marker.id, id);

	EXPECT_TRUE(testEqual(box.center(), toEigen(marker.pose.position)));

	EXPECT_FLOAT_EQ(marker.color.r, color[0]);
	EXPECT_FLOAT_EQ(marker.color.g, color[1]);
	EXPECT_FLOAT_EQ(marker.color.b, color[2]);
	EXPECT_FLOAT_EQ(marker.color.a, color[3]);

	EXPECT_DOUBLE_EQ(marker.scale.x, box.sizes().x());
	EXPECT_DOUBLE_EQ(marker.scale.y, box.sizes().y());
	EXPECT_DOUBLE_EQ(marker.scale.z, box.sizes().z());

	EXPECT_DOUBLE_EQ(marker.lifetime.toSec(), 0.5);
	EXPECT_EQ(marker.type, visualization_msgs::Marker::CUBE);
	EXPECT_EQ(marker.action, visualization_msgs::Marker::ADD);
	EXPECT_EQ(marker.ns, ns);
}

}
