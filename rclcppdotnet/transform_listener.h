#ifndef RCL_DOTNET_TFL
#define RCL_DOTNET_TFL

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

	__declspec(dllexport)
	void* __cdecl native_construct_buffer();

	__declspec(dllexport)
	void* __cdecl native_construct_listener(void* buf);

	__declspec(dllexport)
	void* __cdecl native_construct_time(int sec, int nano);

	__declspec(dllexport)
	void* __cdecl native_lookup_transform(void* buf,
		char* from, char* to, void* t);

	__declspec(dllexport)
	bool __cdecl native_retrieve_translation(void* tf, TfVector3Ptr vec);

    __declspec(dllexport)
	bool __cdecl native_retrieve_rotation(void* tf, TfQuaternionPtr quat);

#ifdef __cplusplus
}
#endif


#endif // RCL_DOTNET_TFL