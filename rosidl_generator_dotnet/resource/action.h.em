@{
type_name = action.namespaced_type.name
action_typename = '%s__%s' % ('__'.join(action.namespaced_type.namespaces), type_name)
action_prefix = "RCLDOTNET_{0}_{1}_{2}".format(package_name, '_'.join(action.namespaced_type.namespaces), type_name).upper()
}@
#if defined(_MSC_VER)
    //  Microsoft
    #define @(action_prefix)_EXPORT __declspec(dllexport)
    #define @(action_prefix)_IMPORT __declspec(dllimport)
    #if defined(_M_IX86)
        #define @(action_prefix)_CDECL __cdecl
    #else
        #define @(action_prefix)_CDECL
    #endif
#elif defined(__GNUC__)
    //  GCC
    #define @(action_prefix)_EXPORT __attribute__((visibility("default")))
    #define @(action_prefix)_IMPORT
    #if defined(__i386__)
        #define @(action_prefix)_CDECL __attribute__((__cdecl__))
    #else
        #define @(action_prefix)_CDECL
    #endif
#else
    //  do nothing and hope for the best?
    #define @(action_prefix)_EXPORT
    #define @(action_prefix)_IMPORT
    #define @(action_prefix)_CDECL
    #pragma warning Unknown dynamic link import/export semantics.
#endif

@(action_prefix)_EXPORT
const void * @(action_prefix)_CDECL @(action_typename)__get_typesupport(void);
