#ifndef RCLCPP_DOTNET_H
#define RCLCPP_DOTNET_H

#include "rclcppdotnet_macros.h"

#ifdef __cplusplus
extern "C" {
#endif

	RCLCPPDOTNET_EXPORT
	void RCLCPPDOTNET_CDECL native_rclcpp_init();

    RCLCPPDOTNET_EXPORT
	void RCLCPPDOTNET_CDECL native_rclcpp_shutdown();

#ifdef __cplusplus
}
#endif

#endif  // RCLCPP_DOTNET_H