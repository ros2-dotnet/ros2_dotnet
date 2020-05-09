@{
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import NamespacedType
from rosidl_generator_dotnet import msg_type_to_c

type_name = message.structure.namespaced_type.name
msg_typename = '%s__%s' % ('__'.join(message.structure.namespaced_type.namespaces), type_name)
msg_prefix = "RCLDOTNET_{0}_{1}_{2}".format(package_name, '_'.join(message.structure.namespaced_type.namespaces), type_name).upper()
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
const void * @(msg_prefix)_CDECL @(msg_typename)__get_typesupport();

@(msg_prefix)_EXPORT
void * @(msg_prefix)_CDECL @(msg_typename)__create_native_message();

@(msg_prefix)_EXPORT
void @(msg_prefix)_CDECL @(msg_typename)__destroy_native_message(void *);

@[for member in message.structure.members]@
@[    if isinstance(member.type, Array)]@
@(msg_prefix)_EXPORT
void * @(msg_prefix)_CDECL @(msg_typename)_get_field_@(member.name)_message(void *, int);
@(msg_prefix)_EXPORT
void * @(msg_prefix)_CDECL @(msg_typename)_native_init_field_@(member.name)_message(void *, int);
@(msg_prefix)_EXPORT
int @(msg_prefix)_CDECL @(msg_typename)_native_getsize_array_field_@(member.name)_message(void *);

@[    elif isinstance(member.type, AbstractSequence)]@
// TODO: Sequence types are not supported
@[    elif isinstance(member.type, AbstractWString)]@
// TODO: Unicode types are not supported
@[    elif isinstance(member.type, BasicType) or isinstance(member.type, AbstractString)]@
@(msg_prefix)_EXPORT
@(msg_type_to_c(member.type)) @(msg_prefix)_CDECL @(msg_typename)__read_field_@(member.name)(void *);

@(msg_prefix)_EXPORT
void @(msg_typename)__write_field_@(member.name)(void *, @(msg_type_to_c(member.type)));
@[    else]@
@(msg_prefix)_EXPORT
void * @(msg_prefix)_CDECL @(msg_typename)__get_field_@(member.name)_HANDLE(void *);
@[    end if]@
@[end for]@

#endif // @(header_guard)
