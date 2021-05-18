#ifndef TRANSFORM_LISTENER_H
#define TRANSFORM_LISTENER_H
#include "rclcppdotnet_macros.h"

typedef struct TfVector3 {
    double x;
    double y;
    double z;
} *TfVector3Ptr;

typedef struct TfQuaternion {
    double x;
    double y;
    double z;
    double w;
} *TfQuaternionPtr;


#ifdef __cplusplus
extern "C" {
#endif

	RCLCPPDOTNET_EXPORT
	void* RCLCPPDOTNET_CDECL native_construct_buffer();

	RCLCPPDOTNET_EXPORT
	void RCLCPPDOTNET_CDECL native_delete_buffer(void* buf);

	RCLCPPDOTNET_EXPORT
	void* RCLCPPDOTNET_CDECL native_construct_listener(void* buf);

	RCLCPPDOTNET_EXPORT
	void RCLCPPDOTNET_CDECL native_delete_listener(void* listener);

	RCLCPPDOTNET_EXPORT
	void* RCLCPPDOTNET_CDECL native_construct_time(int sec, int nano);

	RCLCPPDOTNET_EXPORT
	void* RCLCPPDOTNET_CDECL native_lookup_transform(void* buf,
		char* from, char* to, void* t);

	RCLCPPDOTNET_EXPORT
	bool RCLCPPDOTNET_CDECL native_retrieve_translation(void* tf, TfVector3Ptr vec);

    RCLCPPDOTNET_EXPORT
	bool RCLCPPDOTNET_CDECL native_retrieve_rotation(void* tf, TfQuaternionPtr quat);

#ifdef __cplusplus
}
#endif


#endif // TRANSFORM_LISTENER_H