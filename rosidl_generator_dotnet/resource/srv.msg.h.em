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
}@
@(msg_prefix)_EXPORT
void * @(msg_prefix)_CDECL @(msg_typename)__create_native_message();

@(msg_prefix)_EXPORT
const void * @(msg_prefix)_CDECL @(msg_typename)__get_typesupport();

@(msg_prefix)_EXPORT
void * @(msg_prefix)_CDECL @(msg_typename)__create_native_message();

@(msg_prefix)_EXPORT
void @(msg_prefix)_CDECL @(msg_typename)__destroy_native_message(void *);

@[for member in message.structure.members]@
@[    if isinstance(member.type, Array)]@
@(msg_prefix)_EXPORT
void * @(msg_prefix)_CDECL @(msg_typename)__get_field_@(member.name)_message(void *, int);
@(msg_prefix)_EXPORT
int @(msg_prefix)_CDECL @(msg_typename)__getsize_array_field_@(member.name)_message();

@[        if isinstance(member.type.value_type, BasicType)]@
@(msg_prefix)_EXPORT
void @(msg_typename)__write_field_@(member.name)(void *, @(msg_type_to_c(member.type.value_type)));
@(msg_prefix)_EXPORT
@(msg_type_to_c(member.type.value_type)) @(msg_prefix)_CDECL @(msg_typename)__read_field_@(member.name)(void *);
@[        end if]@

@[    elif isinstance(member.type, AbstractSequence)]@
@[        if isinstance(member.type.value_type, BasicType) or isinstance(member.type.value_type, NamespacedType)]@
@(msg_prefix)_EXPORT
void * @(msg_prefix)_CDECL @(msg_typename)__get_field_@(member.name)_message(void *, int);
@(msg_prefix)_EXPORT
int @(msg_prefix)_CDECL @(msg_typename)__getsize_sequence_field_@(member.name)_message();
@(msg_prefix)_EXPORT
int @(msg_prefix)_CDECL @(msg_typename)__resize_sequence_field_@(member.name)_message(void *, int);
@[            if isinstance(member.type.value_type, BasicType)]@
@(msg_prefix)_EXPORT
void @(msg_typename)__write_field_@(member.name)(void *, @(msg_type_to_c(member.type.value_type)));
@(msg_prefix)_EXPORT
@(msg_type_to_c(member.type.value_type)) @(msg_prefix)_CDECL @(msg_typename)__read_field_@(member.name)(void *);
@[            end if]@
@[        else]@
// TODO: sequences of this type not yet supported
@[        end if]@
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

@(msg_prefix)_EXPORT
void @(msg_prefix)_CDECL @(msg_typename)__destroy_native_message(void * raw_ros_message);