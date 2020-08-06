#ifndef RCL_DOTNET_H
#define RCL_DOTNET_H

#ifdef __cplusplus
extern "C" {
#endif

	__declspec(dllexport)
	void __cdecl native_rclcpp_init();

    __declspec(dllexport)
	void __cdecl native_rclcpp_shutdown();

#ifdef __cplusplus
}
#endif

#endif  // RCL_DOTNET_H