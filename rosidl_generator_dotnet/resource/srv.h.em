@{
type_name = service.namespaced_type.name
srv_typename = '%s__%s' % ('__'.join(service.namespaced_type.namespaces), type_name)
srv_prefix = "RCLDOTNET_{0}_{1}_{2}".format(package_name, '_'.join(service.namespaced_type.namespaces), type_name).upper()
}@
#if defined(_MSC_VER)
    //  Microsoft
    #define @(srv_prefix)_EXPORT __declspec(dllexport)
    #define @(srv_prefix)_IMPORT __declspec(dllimport)
    #define @(srv_prefix)_CDECL __cdecl
#elif defined(__GNUC__)
    //  GCC
    #define @(srv_prefix)_EXPORT __attribute__((visibility("default")))
    #define @(srv_prefix)_IMPORT
    #define @(srv_prefix)_CDECL __attribute__((__cdecl__))
#else
    //  do nothing and hope for the best?
    #define @(srv_prefix)_EXPORT
    #define @(srv_prefix)_IMPORT
    #define @(srv_prefix)_CDECL
    #pragma warning Unknown dynamic link import/export semantics.
#endif

@(srv_prefix)_EXPORT
const void * @(srv_prefix)_CDECL @(srv_typename)__get_typesupport(void);
