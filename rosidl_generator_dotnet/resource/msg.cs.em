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
using System.Collections.Generic;

using ROS2.Interfaces;
using ROS2.Utils;

@[for ns in message.structure.namespaced_type.namespaces]@
namespace @(ns)
{
@[end for]@

public class @(type_name) : IMessage {
    private static readonly DllLoadUtils dllLoadUtils;

@[for member in message.structure.members]@
@[    if isinstance(member.type, Array)]@
    public const int @(get_field_name(type_name, member.name))_Count = @(member.type.size);
@[    end if]@
@[end for]@

    public @(type_name)()
    {
@[for member in message.structure.members]@
@[    if isinstance(member.type, Array)]@
@[        if isinstance(member.type.value_type, BasicType)]@       
        // @(member.type.value_type) @(member.type.value_type.typename)
@[        end if]@       
        @(get_field_name(type_name, member.name)) = new List<@(get_dotnet_type(member.type.value_type))>(@(member.type.size));

@[    elif isinstance(member.type, AbstractSequence)]@
// TODO: Sequence types are not supported
@[    elif isinstance(member.type, AbstractWString)]@
// TODO: Unicode types are not supported
@[    elif isinstance(member.type, BasicType)]@
@[    elif isinstance(member.type, AbstractString)]@
        @(get_field_name(type_name, member.name)) = "";
@[    else]@
        @(get_field_name(type_name, member.name)) = new @(get_dotnet_type(member.type))();
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

        @(type_name).native_create_native_message = (NativeCreateNativeType)Marshal.GetDelegateForFunctionPointer(
            native_create_native_message_ptr, typeof(NativeCreateNativeType));

        IntPtr native_destroy_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "@(msg_typename)__destroy_native_message");

        @(type_name).native_destroy_native_message = (NativeDestroyNativeType)Marshal.GetDelegateForFunctionPointer(
            native_destroy_native_message_ptr, typeof(NativeDestroyNativeType));

@[for member in message.structure.members]@
@[    if isinstance(member.type, Array)]@
        IntPtr native_get_field_@(member.name)_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "@(msg_typename)__get_field_@(member.name)_message");
@[        if isinstance(member.type.value_type, BasicType) or isinstance(member.type.value_type, AbstractString)]@
        IntPtr native_write_field_@(member.name)_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "@(msg_typename)__write_field_@(member.name)");
        IntPtr native_read_field_@(member.name)_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "@(msg_typename)__read_field_@(member.name)");
@[        end if]@

        @(type_name).native_get_field_@(member.name)_message = (NativeGetField@(get_field_name(type_name, member.name))Type)Marshal.GetDelegateForFunctionPointer(
            native_get_field_@(member.name)_message_ptr, typeof(NativeGetField@(get_field_name(type_name, member.name))Type));

@[        if isinstance(member.type.value_type, BasicType) or isinstance(member.type.value_type, AbstractString)]@
        @(type_name).native_write_field_@(member.name) = (NativeWriteField@(get_field_name(type_name, member.name))Type)Marshal.GetDelegateForFunctionPointer(
            native_write_field_@(member.name)_ptr, typeof(NativeWriteField@(get_field_name(type_name, member.name))Type));
        @(type_name).native_read_field_@(member.name) = (NativeReadField@(get_field_name(type_name, member.name))Type)Marshal.GetDelegateForFunctionPointer(
            native_read_field_@(member.name)_ptr, typeof(NativeReadField@(get_field_name(type_name, member.name))Type));
@[        end if]@

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
    private delegate IntPtr NativeCreateNativeType();

    private static NativeCreateNativeType native_create_native_message = null;

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate void NativeDestroyNativeType(IntPtr messageHandle);

    private static NativeDestroyNativeType native_destroy_native_message = null;

@[for member in message.structure.members]@
@[    if isinstance(member.type, Array)]@
    private static NativeGetField@(get_field_name(type_name, member.name))Type native_get_field_@(member.name)_message = null;
@[        if isinstance(member.type.value_type, BasicType) or isinstance(member.type.value_type, AbstractString)]@
    private static NativeWriteField@(get_field_name(type_name, member.name))Type native_write_field_@(member.name) = null;
    private static NativeReadField@(get_field_name(type_name, member.name))Type native_read_field_@(member.name) = null;
@[        end if]@
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate IntPtr NativeGetField@(get_field_name(type_name, member.name))Type(
        IntPtr messageHandle, int index);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate int NativeGetSizeArrayField@(get_field_name(type_name, member.name))Type(IntPtr messageHandle);
@[            if isinstance(member.type.value_type, AbstractString)]@
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate void NativeWriteField@(get_field_name(type_name, member.name))Type(
        IntPtr messageHandle, [MarshalAs (UnmanagedType.LPStr)] string value);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate IntPtr NativeReadField@(get_field_name(type_name, member.name))Type(IntPtr messageHandle);
@[            else]@
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate @(get_dotnet_type(member.type.value_type)) NativeReadField@(get_field_name(type_name, member.name))Type(
        IntPtr messageHandle);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate void NativeWriteField@(get_field_name(type_name, member.name))Type(
        IntPtr messageHandle, @(get_dotnet_type(member.type.value_type)) value);
@[            end if]@
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

      {
          @(get_field_name(type_name, member.name)).Clear();
          for (int i = 0; i < @(member.type.size); i++)
          {
@[        if isinstance(member.type.value_type, BasicType)]
              @(get_field_name(type_name, member.name)).Add(native_read_field_@(member.name)(native_get_field_@(member.name)_message(messageHandle, i)));
@[        elif isinstance(member.type.value_type, AbstractString)]
              IntPtr pStr_@(get_field_name(type_name, member.name)) = native_read_field_@(member.name)(native_get_field_@(member.name)_message(messageHandle, i));
              @(get_field_name(type_name, member.name)).Add(Marshal.PtrToStringAnsi(pStr_@(get_field_name(type_name, member.name))));
          @[        elif isinstance(member.type.value_type, AbstractWString)]
              // TODO: Unicode types are not supported  
          @[        else]
              @(get_field_name(type_name, member.name)).Add(new @(get_dotnet_type(member.type.value_type))());
              @(get_field_name(type_name, member.name))[@(get_field_name(type_name, member.name)).Count-1]._READ_HANDLE(native_get_field_@(member.name)_message(messageHandle, i));
@[        end if]
          }
      }

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
        {
            int count = 0;
            foreach(@(get_dotnet_type(member.type.value_type)) value in @(get_field_name(type_name, member.name)))
            {
@[        if isinstance(member.type.value_type, BasicType) or isinstance(member.type.value_type, AbstractString)]@
                native_write_field_@(member.name)(native_get_field_@(member.name)_message(messageHandle, count++), value);
@[        elif isinstance(member.type.value_type, AbstractWString)]
// TODO: Unicode types are not supported  
@[        else]@
                value._WRITE_HANDLE(native_get_field_@(member.name)_message(messageHandle, count++));
@[        end if]@
            }
        }

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
    public List<@(get_dotnet_type(member.type.value_type))> @(get_field_name(type_name, member.name));
@[    elif isinstance(member.type, AbstractSequence)]@
// TODO: Sequence types are not supported
@[    elif isinstance(member.type, AbstractWString)]@
// TODO: Unicode types are not supported
@[    else]@
    public @(get_dotnet_type(member.type)) @(get_field_name(type_name, member.name)) { get; set; }
@[    end if]@
@[end for]@
}

@[for ns in reversed(message.structure.namespaced_type.namespaces)]@
}  // namespace @(ns)
@[end for]@