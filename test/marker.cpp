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

#include <gtest/gtest.h>
#include <dr_eigen/ros.hpp>
#include <dr_eigen/eigen.hpp>
#include <dr_eigen/test/compare.hpp>

#include <cmath>

int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(MarkerTest, makeSphereMarker) {
	ros::Time::init();

	ros::Time now = ros::Time::now();

	Eigen::Vector3d position(0.5, 0.5, 0.5); // some position
	std::string frame_id   = "some_frame";
	double radius          = 0.5;
	std::string ns         = "some_namespace";
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

	std::string frame_id       = "some_frame";
	Eigen::AngleAxisd rotation = rotateY(M_PI);
	Eigen::Isometry3d pose     = translate(0.5, 0.5, 0.5) * rotation;
	double radius              = 0.5;
	double height              = 1.0;
	std::string ns             = "some_namespace";
	ros::Duration lifetime     = ros::Duration(0.5); // 0.5s lifetime
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
	EXPECT_DOUBLE_EQ(marker.scale.z, height);

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
