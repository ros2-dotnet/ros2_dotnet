@{
type_name = service.namespaced_type.name
msg_typename = '%s__%s' % ('__'.join(service.namespaced_type.namespaces), type_name)
msg_prefix = "RCLDOTNET_{0}_{1}".format(package_name, '_'.join(service.namespaced_type.namespaces)).upper()
header_guard = "{0}_H".format(msg_prefix)
}@
#ifndef @(header_guard)
#define @(header_guard)

#if defined(WIN32)
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

@{
TEMPLATE(
    'srv.msg.h.em',
    package_name=package_name, msg_prefix=msg_prefix, message=service.request_message)
}@

@{
TEMPLATE(
    'srv.msg.h.em',
    package_name=package_name, msg_prefix=msg_prefix, message=service.response_message)
}@

@(msg_prefix)_EXPORT
const void * @(msg_prefix)_CDECL @(msg_typename)__get_typesupport();

#endif // @(header_guard)
