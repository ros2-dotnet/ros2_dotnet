#include <rclcpp/rclcpp.hpp>

#include "rclcppdotnet.h"

void native_rclcpp_init() {
	rclcpp::init(0, nullptr);  // no args
}

void native_rclcpp_shutdown() {
	if (rclcpp::ok()) {
		rclcpp::shutdown();
	}
}

