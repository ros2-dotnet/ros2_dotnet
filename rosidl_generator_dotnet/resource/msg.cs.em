using System;
using System.Runtime.InteropServices;

using ROS2.Interfaces;
using ROS2.Utils;

namespace @(package_name)
{
namespace @(subfolder)
{

public class @(type_name) : IMessage {
    private static readonly DllLoadUtils dllLoadUtils;

    public @(type_name)() {}

    static @(type_name)()
    {
        dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
        IntPtr nativelibrary = dllLoadUtils.LoadLibrary("@(spec.base_type.pkg_name)_@(type_name)__rosidl_typesupport_c");

        IntPtr native_get_typesupport_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "native_get_typesupport");

        @(type_name).native_get_typesupport = (NativeGetTypeSupportType)Marshal.GetDelegateForFunctionPointer(
            native_get_typesupport_ptr, typeof(NativeGetTypeSupportType));

        IntPtr native_create_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "native_create_native_message");

        @(type_name).native_create_native_message = (NativeCreateNativeMessageType)Marshal.GetDelegateForFunctionPointer(
            native_create_native_message_ptr, typeof(NativeCreateNativeMessageType));

        IntPtr native_destroy_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "native_destroy_native_message");

        @(type_name).native_destroy_native_message = (NativeDestroyNativeMessageType)Marshal.GetDelegateForFunctionPointer(
            native_destroy_native_message_ptr, typeof(NativeDestroyNativeMessageType));

@[for field in spec.fields]@
@[    if field.type.is_array]@
// TODO(esteve): Arrays are not supported
@[    else]@
@[        if field.type.is_primitive_type()]@
        IntPtr native_read_field_@(field.name)_ptr =
            dllLoadUtils.GetProcAddress(nativelibrary, "native_read_field_@(field.name)");
        @(type_name).native_read_field_@(field.name) =
            (NativeReadField@(get_field_name(type_name, field.name))Type)Marshal.GetDelegateForFunctionPointer(
            native_read_field_@(field.name)_ptr, typeof(NativeReadField@(get_field_name(type_name, field.name))Type));

        IntPtr native_write_field_@(field.name)_ptr =
            dllLoadUtils.GetProcAddress(nativelibrary, "native_write_field_@(field.name)");
        @(type_name).native_write_field_@(field.name) =
            (NativeWriteField@(get_field_name(type_name, field.name))Type)Marshal.GetDelegateForFunctionPointer(
            native_write_field_@(field.name)_ptr, typeof(NativeWriteField@(get_field_name(type_name, field.name))Type));
@[        else]@
// TODO(esteve): Nested types are not supported
@[        end if]@
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

@[for field in spec.fields]@
@[    if field.type.is_array]@
// TODO(esteve): Arrays are not supported
@[    else]@
@[        if field.type.is_primitive_type()]@
@[            if field.type.type == 'string']@
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate IntPtr NativeReadField@(get_field_name(type_name, field.name))Type(IntPtr messageHandle);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate void NativeWriteField@(get_field_name(type_name, field.name))Type(
        IntPtr messageHandle, [MarshalAs (UnmanagedType.LPStr)] string value);
@[            else]@
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate @(get_dotnet_type(field.type)) NativeReadField@(get_field_name(type_name, field.name))Type(
        IntPtr messageHandle);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate void NativeWriteField@(get_field_name(type_name, field.name))Type(
        IntPtr messageHandle, @(get_dotnet_type(field.type)) value);
@[            end if]@
    private static NativeReadField@(get_field_name(type_name, field.name))Type native_read_field_@(field.name) = null;

    private static NativeWriteField@(get_field_name(type_name, field.name))Type native_write_field_@(field.name) = null;
@[        else]@
// TODO(esteve): Nested types are not supported
@[        end if]@
@[    end if]@
@[end for]@

    public static IntPtr _GET_TYPE_SUPPORT() {
        return native_get_typesupport();
    }

    public IntPtr _CREATE_NATIVE_MESSAGE() {
        return native_create_native_message();
    }

    public void _READ_HANDLE(IntPtr messageHandle) {
@[for field in spec.fields]@
@[    if field.type.is_array]@
@[    else]@
@[        if field.type.is_primitive_type()]@
@[            if field.type.type == 'string']@
        {
            IntPtr pStr = native_read_field_@(field.name)(messageHandle);
            @(get_field_name(type_name, field.name)) = Marshal.PtrToStringAnsi(pStr);
        }
@[            else]@
        @(get_field_name(type_name, field.name)) = native_read_field_@(field.name)(messageHandle);
@[            end if]@
@[        else]@
@[        end if]@
@[    end if]@
@[end for]@
    }

    public void _WRITE_HANDLE(IntPtr messageHandle) {
@[for field in spec.fields]@
@[    if field.type.is_array]@
@[    else]@
@[        if field.type.is_primitive_type()]@
        native_write_field_@(field.name)(messageHandle, @(get_field_name(type_name, field.name)));
@[        else]@
        @(get_field_name(type_name, field.name))._WRITE_HANDLE(messageHandle);
@[        end if]@
@[    end if]@
@[end for]@
    }

    public void _DESTROY_NATIVE_MESSAGE(IntPtr messageHandle) {
        native_destroy_native_message(messageHandle);
    }

@[for constant in spec.constants]@
    public static readonly @(get_builtin_dotnet_type(constant.type)) @(constant.name) =
        @(constant_value_to_dotnet(constant.type, constant.value));
@[end for]@

@[for field in spec.fields]@
@[    if field.type.is_array]@
// TODO(esteve): Arrays are not supported
@[    else]@
@[        if field.type.is_primitive_type()]@
    public @(get_dotnet_type(field.type)) @(get_field_name(type_name, field.name)) { get; set; }
@[        else]@
    public @(get_dotnet_type(field.type)) @(get_field_name(type_name, field.name)) { get; set; }
@[        end if]@
@[    end if]@
@[end for]@
}

} // @(subfolder)

} // @(package_name)
