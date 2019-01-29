@{
msg_prefix = "RCLDOTNET_{0}_{1}_{2}".format(package_name, subfolder, convert_camel_case_to_lower_case_underscore(type_name)).upper()
header_guard = "{0}_H".format(msg_prefix)
}@
#ifndef @(header_guard)
#define @(header_guard)

#if defined(_MSC_VER)
    //  Microsoft
    #define @(msg_prefix)_EXPORT __declspec(dllexport)
    #define @(msg_prefix)_IMPORT __declspec(dllimport)
    #define @(msg_prefix)_CDECL __cdecl
#elif defined(__GNUC__)
    //  GCC
    #define @(msg_prefix)_EXPORT __attribute__((visibility("default")))
    #define @(msg_prefix)_IMPORT
    #define @(msg_prefix)_CDECL __attribute__((__cdecl__))
#else
    //  do nothing and hope for the best?
    #define @(msg_prefix)_EXPORT
    #define @(msg_prefix)_IMPORT
    #define @(msg_prefix)_CDECL
    #pragma warning Unknown dynamic link import/export semantics.
#endif

@(msg_prefix)_EXPORT
const void * @(msg_prefix)_CDECL native_get_typesupport();

@(msg_prefix)_EXPORT
void * @(msg_prefix)_CDECL native_create_native_message();

@(msg_prefix)_EXPORT
void @(msg_prefix)_CDECL native_destroy_native_message(void *);

@[for field in spec.fields]@
@[    if field.type.is_array]@
@(msg_prefix)_EXPORT
void * @(msg_prefix)_CDECL native_get_field_@(field.name)_message(void *, int);
@(msg_prefix)_EXPORT
void * @(msg_prefix)_CDECL native_init_field_@(field.name)_message(void *, int);
@[        if field.type.is_primitive_type()]@
@(msg_prefix)_EXPORT
void native_write_field_@(field.name)(void *, @(primitive_msg_type_to_c(field.type.type)));
@[        end if]@
@[    else]@
@[        if field.type.is_primitive_type()]@
@(msg_prefix)_EXPORT
@(primitive_msg_type_to_c(field.type.type)) @(msg_prefix)_CDECL native_read_field_@(field.name)(void *);

@(msg_prefix)_EXPORT
void native_write_field_@(field.name)(void *, @(primitive_msg_type_to_c(field.type.type)));
@[        else]@
@(msg_prefix)_EXPORT
void * @(msg_prefix)_CDECL native_get_field_@(field.name)_message(void * raw_ros_message);
@[        end if]@
@[    end if]@
@[end for]@

#endif // @(header_guard)
