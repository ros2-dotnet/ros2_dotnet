#include <string>
#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "transform_listener.h"



void* native_construct_buffer() {
	auto clock = std::make_shared<rclcpp::Clock>();
	return new tf2_ros::Buffer(clock);
}

void native_delete_buffer(void* buf) {
	auto casted = (tf2_ros::Buffer*)buf;
	delete casted;
}

void* native_construct_listener(void* buf) {
	return new tf2_ros::TransformListener(*((tf2_ros::Buffer*)buf));
}

void native_delete_listener(void* listener) {
	auto casted = (tf2_ros::TransformListener*)listener;
	delete listener;
}

void* native_construct_time(int sec, int nano) {
	return new rclcpp::Time(sec, nano);
}

void* native_lookup_transform(void* buf,
	char* from, char* to, void* t) {
	try {
		auto outputVar = new geometry_msgs::msg::TransformStamped(
			((tf2_ros::Buffer*)buf)->lookupTransform((char*)from, (char*)to, *((rclcpp::Time*)t))
		);
		return outputVar;
	}
	catch (tf2::TransformException &ex) {
		return nullptr;
	}
}

bool native_retrieve_translation(void* tf, TfVector3Ptr vec) {
	try {
		auto castedTf = (geometry_msgs::msg::TransformStamped*)tf;
		vec->x = castedTf->transform.translation.x;
		vec->y = castedTf->transform.translation.y;
		vec->z = castedTf->transform.translation.z;
		return true;
	}
	catch (...) {
		return false;
	}
	return false;
}

bool native_retrieve_rotation(void* tf, TfQuaternionPtr quat) {
	try {
		auto castedTf = (geometry_msgs::msg::TransformStamped*)tf;
		quat->x = castedTf->transform.rotation.x;
		quat->y = castedTf->transform.rotation.y;
		quat->z = castedTf->transform.rotation.z;
		quat->w = castedTf->transform.rotation.w;
		return true;
	}
	catch (...) {
		return false;
	}
	return false;
}