@{
from rosidl_generator_dotnet import get_field_name
from rosidl_generator_dotnet import get_dotnet_type
from rosidl_generator_dotnet import get_builtin_dotnet_type
from rosidl_generator_dotnet import constant_value_to_dotnet

from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import NamespacedType

type_name = message.structure.namespaced_type.name
msg_typename = '%s__%s' % ('__'.join(message.structure.namespaced_type.namespaces), type_name)
}
using System;
using System.Runtime.InteropServices;

using ROS2.Interfaces;
using ROS2.Utils;

@[for ns in message.structure.namespaced_type.namespaces]@
namespace @(ns)
{
@[end for]@

public class @(type_name) : IMessage {
    private static readonly DllLoadUtils dllLoadUtils;

    public @(type_name)()
    {
@[for member in message.structure.members]@
@[    if isinstance(member.type, Array)]@
// TODO: Array types are not supported
@[    elif isinstance(member.type, AbstractSequence)]@
// TODO: Sequence types are not supported
@[    elif isinstance(member.type, AbstractWString)]@
// TODO: Unicode types are not supported
@[    elif isinstance(member.type, BasicType)]@
@[    elif isinstance(member.type, AbstractString)]@
        @(get_field_name(type_name, member.name)) = "";
@[    else]@
        @(get_field_name(type_name, member.name)) = new @(member.type.namespaces[0]).@(member.type.namespaces[1]).@(member.type.name)(); 
@[    end if]@
@[end for]@
    }

    static @(type_name)()
    {
        dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
        IntPtr nativelibrary = dllLoadUtils.LoadLibrary("@(package_name)__dotnetext");

        IntPtr native_get_typesupport_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "@(msg_typename)__get_typesupport");

        @(type_name).native_get_typesupport = (NativeGetTypeSupportType)Marshal.GetDelegateForFunctionPointer(
            native_get_typesupport_ptr, typeof(NativeGetTypeSupportType));

        IntPtr native_create_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "@(msg_typename)__create_native_message");

        @(type_name).native_create_native_message = (NativeCreateNativeMessageType)Marshal.GetDelegateForFunctionPointer(
            native_create_native_message_ptr, typeof(NativeCreateNativeMessageType));

        IntPtr native_destroy_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "@(msg_typename)__destroy_native_message");

        @(type_name).native_destroy_native_message = (NativeDestroyNativeMessageType)Marshal.GetDelegateForFunctionPointer(
            native_destroy_native_message_ptr, typeof(NativeDestroyNativeMessageType));

@[for member in message.structure.members]@
@[    if isinstance(member.type, Array)]@
// TODO: Array types are not supported
@[    elif isinstance(member.type, AbstractSequence)]@
// TODO: Sequence types are not supported
@[    elif isinstance(member.type, AbstractWString)]@
// TODO: Unicode types are not supported
@[    elif isinstance(member.type, BasicType) or isinstance(member.type, AbstractString)]@
        IntPtr native_read_field_@(member.name)_ptr =
            dllLoadUtils.GetProcAddress(nativelibrary, "@(msg_typename)__read_field_@(member.name)");
        @(type_name).native_read_field_@(member.name) =
            (NativeReadField@(get_field_name(type_name, member.name))Type)Marshal.GetDelegateForFunctionPointer(
            native_read_field_@(member.name)_ptr, typeof(NativeReadField@(get_field_name(type_name, member.name))Type));

        IntPtr native_write_field_@(member.name)_ptr =
            dllLoadUtils.GetProcAddress(nativelibrary, "@(msg_typename)__write_field_@(member.name)");
        @(type_name).native_write_field_@(member.name) =
            (NativeWriteField@(get_field_name(type_name, member.name))Type)Marshal.GetDelegateForFunctionPointer(
            native_write_field_@(member.name)_ptr, typeof(NativeWriteField@(get_field_name(type_name, member.name))Type));
@[    else]@
        IntPtr native_get_field_@(member.name)_HANDLE_ptr =
            dllLoadUtils.GetProcAddress(nativelibrary, "@(msg_typename)__get_field_@(member.name)_HANDLE");
        
        @(type_name).native_get_field_@(member.name)_HANDLE =
            (NativeGetField@(get_field_name(type_name, member.name))Type)Marshal.GetDelegateForFunctionPointer(
            native_get_field_@(member.name)_HANDLE_ptr, typeof(NativeGetField@(get_field_name(type_name, member.name))Type));
@[    end if]@
@[end for]@
    }

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate IntPtr NativeGetTypeSupportType();

    private static NativeGetTypeSupportType native_get_typesupport = null;

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate IntPtr NativeCreateNativeMessageType();

    private static NativeCreateNativeMessageType native_create_native_message = null;

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate void NativeDestroyNativeMessageType(IntPtr messageHandle);

    private static NativeDestroyNativeMessageType native_destroy_native_message = null;

@[for member in message.structure.members]@
@[    if isinstance(member.type, Array)]@
// TODO: Array types are not supported
@[    elif isinstance(member.type, AbstractSequence)]@
// TODO: Sequence types are not supported
@[   elif isinstance(member.type, AbstractWString)]@
// TODO: Unicode types are not supported
@[    elif isinstance(member.type, BasicType) or isinstance(member.type, AbstractString)]@
@[        if isinstance(member.type, AbstractString)]@
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate IntPtr NativeReadField@(get_field_name(type_name, member.name))Type(IntPtr messageHandle);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate void NativeWriteField@(get_field_name(type_name, member.name))Type(
        IntPtr messageHandle, [MarshalAs (UnmanagedType.LPStr)] string value);
@[        else]@
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate @(get_dotnet_type(member.type)) NativeReadField@(get_field_name(type_name, member.name))Type(
        IntPtr messageHandle);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate void NativeWriteField@(get_field_name(type_name, member.name))Type(
        IntPtr messageHandle, @(get_dotnet_type(member.type)) value);
@[        end if]@
    private static NativeReadField@(get_field_name(type_name, member.name))Type native_read_field_@(member.name) = null;

    private static NativeWriteField@(get_field_name(type_name, member.name))Type native_write_field_@(member.name) = null;
@[    else]@
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate IntPtr NativeGetField@(get_field_name(type_name, member.name))Type(IntPtr messageHandle);

    private static NativeGetField@(get_field_name(type_name, member.name))Type native_get_field_@(member.name)_HANDLE = null;
@[    end if]@
@[end for]@

    public static IntPtr _GET_TYPE_SUPPORT() {
        return native_get_typesupport();
    }

    public IntPtr _CREATE_NATIVE_MESSAGE() {
        return native_create_native_message();
    }

    public void _READ_HANDLE(IntPtr messageHandle) {
@[for member in message.structure.members]@
@[    if isinstance(member.type, Array)]@
// TODO: Array types are not supported
@[    elif isinstance(member.type, AbstractSequence)]@
// TODO: Sequence types are not supported
@[    elif isinstance(member.type, AbstractWString)]@
// TODO: Unicode types are not supported
@[    elif isinstance(member.type, BasicType) or isinstance(member.type, AbstractString)]@
@[        if isinstance(member.type, AbstractString)]@
        IntPtr pStr_@(get_field_name(type_name, member.name)) = native_read_field_@(member.name)(messageHandle);
        @(get_field_name(type_name, member.name)) = Marshal.PtrToStringAnsi(pStr_@(get_field_name(type_name, member.name)));
@[        else]@
        @(get_field_name(type_name, member.name)) = native_read_field_@(member.name)(messageHandle);
@[        end if]@
@[    else]@
        @(get_field_name(type_name, member.name))._READ_HANDLE(native_get_field_@(member.name)_HANDLE(messageHandle));
@[    end if]@
@[end for]@
    }

    public void _WRITE_HANDLE(IntPtr messageHandle) {
@[for member in message.structure.members]@
@[    if isinstance(member.type, Array)]@
// TODO: Array types are not supported
@[    elif isinstance(member.type, AbstractSequence)]@
// TODO: Sequence types are not supported
@[    elif isinstance(member.type, AbstractWString)]@
// TODO: Unicode types are not supported
@[    elif isinstance(member.type, BasicType) or isinstance(member.type, AbstractString)]@
        native_write_field_@(member.name)(messageHandle, @(get_field_name(type_name, member.name)));
@[    else]@
        @(get_field_name(type_name, member.name))._WRITE_HANDLE(native_get_field_@(member.name)_HANDLE(messageHandle));
@[    end if]@
@[end for]@
    }

    public void _DESTROY_NATIVE_MESSAGE(IntPtr messageHandle) {
        native_destroy_native_message(messageHandle);
    }

@[for constant in message.constants]@
    public static readonly @(get_dotnet_type(constant.type)) @(constant.name) =
        @(constant_value_to_dotnet(constant.type, constant.value));
@[end for]@

@[for member in message.structure.members]@
@[    if isinstance(member.type, Array)]@
// TODO: Array types are not supported
@[    elif isinstance(member.type, AbstractSequence)]@
// TODO: Sequence types are not supported
@[    elif isinstance(member.type, AbstractWString)]@
// TODO: Unicode types are not supported
@[    elif isinstance(member.type, BasicType) or isinstance(member.type, AbstractString)]@
    public @(get_dotnet_type(member.type)) @(get_field_name(type_name, member.name)) { get; set; }
@[    else]@
    public @(member.type.namespaces[0]).@(member.type.namespaces[1]).@(member.type.name) @(get_field_name(type_name, member.name)) { get; set; }
@[    end if]@
@[end for]@
}

@[for ns in reversed(message.structure.namespaced_type.namespaces)]@
}  // namespace @(ns)
@[end for]@