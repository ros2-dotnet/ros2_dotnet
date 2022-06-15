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
namespace @('.'.join(message.structure.namespaced_type.namespaces))
{

public class @(type_name) : global::ROS2.IRosMessage {
    private static readonly DllLoadUtils dllLoadUtils;

@[for member in message.structure.members]@
@[    if isinstance(member.type, Array)]@
    public const int @(get_field_name(type_name, member.name))Length = @(member.type.size);
@[    elif isinstance(member.type, AbstractSequence) and member.type.has_maximum_size()]@
    public const int @(get_field_name(type_name, member.name))MaxCount = @(member.type.maximum_size);
@[    end if]@
@[end for]@

    public @(type_name)()
    {
@[for member in message.structure.members]@
@[    if isinstance(member.type, Array)]@
        @(get_field_name(type_name, member.name)) = new @(get_dotnet_type(member.type.value_type))[@(member.type.size)];
@[          if isinstance(member.type.value_type, AbstractString)]@
        for (var i__local_variable = 0; i__local_variable < @(member.type.size); i__local_variable++)
        {
            @(get_field_name(type_name, member.name))[i__local_variable] = "";
        }
@[          elif isinstance(member.type.value_type, BasicType)]@
@# Basic types get initialized by the array constructor.
@[          elif isinstance(member.type.value_type, AbstractWString)]@
// TODO: Unicode types are not supported
@[          else]@
        for (var i__local_variable = 0; i__local_variable < @(member.type.size); i__local_variable++)
        {
            @(get_field_name(type_name, member.name))[i__local_variable] = new @(get_dotnet_type(member.type.value_type))();
        }
@[          end if]@
@[    elif isinstance(member.type, AbstractSequence)]@
        @(get_field_name(type_name, member.name)) = new List<@(get_dotnet_type(member.type.value_type))>();
@[    elif isinstance(member.type, AbstractWString)]@
// TODO: Unicode types are not supported
@[    elif isinstance(member.type, BasicType)]@
@[        if member.has_annotation('default')]@
        @(get_field_name(type_name, member.name)) = @(constant_value_to_dotnet(member.type, member.get_annotation_value('default')['value']));
@[        end if]@
@[    elif isinstance(member.type, AbstractString)]@
@[        if member.has_annotation('default')]@
        @(get_field_name(type_name, member.name)) = @(constant_value_to_dotnet(member.type, member.get_annotation_value('default')['value']));
@[        else]@
        @(get_field_name(type_name, member.name)) = "";
@[        end if]@
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
@[    if isinstance(member.type, Array) or isinstance(member.type, AbstractSequence)]@
        IntPtr native_get_field_@(member.name)_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "@(msg_typename)__get_field_@(member.name)_message");
@[        if isinstance(member.type, AbstractSequence)]@
        IntPtr native_getsize_field_@(member.name)_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "@(msg_typename)__getsize_field_@(member.name)_message");
        IntPtr native_init_sequence_field_@(member.name)_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "@(msg_typename)__init_sequence_field_@(member.name)_message");
@[        end if]@
@[        if isinstance(member.type.value_type, BasicType) or isinstance(member.type.value_type, AbstractString)]@
        IntPtr native_write_field_@(member.name)_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "@(msg_typename)__write_field_@(member.name)");
        IntPtr native_read_field_@(member.name)_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "@(msg_typename)__read_field_@(member.name)");
@[        end if]@

        @(type_name).native_get_field_@(member.name)_message = (NativeGetField@(get_field_name(type_name, member.name))Type)Marshal.GetDelegateForFunctionPointer(
            native_get_field_@(member.name)_message_ptr, typeof(NativeGetField@(get_field_name(type_name, member.name))Type));

@[        if isinstance(member.type, AbstractSequence)]@
        @(type_name).native_getsize_field_@(member.name)_message = (NativeGetSizeField@(get_field_name(type_name, member.name))Type)Marshal.GetDelegateForFunctionPointer(
            native_getsize_field_@(member.name)_message_ptr, typeof(NativeGetSizeField@(get_field_name(type_name, member.name))Type));
        @(type_name).native_init_seqence_field_@(member.name)_message = (NativeInitSequenceField@(get_field_name(type_name, member.name))Type)Marshal.GetDelegateForFunctionPointer(
            native_init_sequence_field_@(member.name)_message_ptr, typeof(NativeInitSequenceField@(get_field_name(type_name, member.name))Type));
@[        end if]@
@[        if isinstance(member.type.value_type, BasicType) or isinstance(member.type.value_type, AbstractString)]@
        @(type_name).native_write_field_@(member.name) = (NativeWriteField@(get_field_name(type_name, member.name))Type)Marshal.GetDelegateForFunctionPointer(
            native_write_field_@(member.name)_ptr, typeof(NativeWriteField@(get_field_name(type_name, member.name))Type));
        @(type_name).native_read_field_@(member.name) = (NativeReadField@(get_field_name(type_name, member.name))Type)Marshal.GetDelegateForFunctionPointer(
            native_read_field_@(member.name)_ptr, typeof(NativeReadField@(get_field_name(type_name, member.name))Type));
@[        end if]@

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
    private delegate Safe@(type_name)Handle NativeCreateNativeType();

    private static NativeCreateNativeType native_create_native_message = null;

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate void NativeDestroyNativeType(IntPtr messageHandle);

    private static NativeDestroyNativeType native_destroy_native_message = null;

@[for member in message.structure.members]@
@[    if isinstance(member.type, Array) or isinstance(member.type, AbstractSequence)]@
    private static NativeGetField@(get_field_name(type_name, member.name))Type native_get_field_@(member.name)_message = null;
@[        if isinstance(member.type, AbstractSequence)]@
    private static NativeGetSizeField@(get_field_name(type_name, member.name))Type native_getsize_field_@(member.name)_message = null;
    private static NativeInitSequenceField@(get_field_name(type_name, member.name))Type native_init_seqence_field_@(member.name)_message = null;
@[        end if]@
@[        if isinstance(member.type.value_type, BasicType) or isinstance(member.type.value_type, AbstractString)]@
    private static NativeWriteField@(get_field_name(type_name, member.name))Type native_write_field_@(member.name) = null;
    private static NativeReadField@(get_field_name(type_name, member.name))Type native_read_field_@(member.name) = null;
@[        end if]@
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate IntPtr NativeGetField@(get_field_name(type_name, member.name))Type(
        IntPtr messageHandle, int index);
@[        if isinstance(member.type, AbstractSequence)]@
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate int NativeGetSizeField@(get_field_name(type_name, member.name))Type(
        IntPtr messageHandle);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate bool NativeInitSequenceField@(get_field_name(type_name, member.name))Type(
        IntPtr messageHandle, int size);
@[        end if]@
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

@# This method gets called via reflection as static abstract interface members are not supported yet.
    [global::System.ComponentModel.EditorBrowsable(global::System.ComponentModel.EditorBrowsableState.Never)]
    public static global::System.IntPtr __GetTypeSupport() => native_get_typesupport();

@# This method gets called via reflection as static abstract interface members are not supported yet.
    [global::System.ComponentModel.EditorBrowsable(global::System.ComponentModel.EditorBrowsableState.Never)]
    public static global::System.Runtime.InteropServices.SafeHandle __CreateMessageHandle() => native_create_native_message();

    [global::System.ComponentModel.EditorBrowsable(global::System.ComponentModel.EditorBrowsableState.Never)]
    public void __ReadFromHandle(global::System.IntPtr messageHandle) {
@[for member in message.structure.members]@
@[    if isinstance(member.type, Array)]@
      {
          @(get_field_name(type_name, member.name)) = new @(get_dotnet_type(member.type.value_type))[@(member.type.size)];
          for (int i__local_variable = 0; i__local_variable < @(member.type.size); i__local_variable++)
          {
@[        if isinstance(member.type.value_type, BasicType)]@
              @(get_field_name(type_name, member.name))[i__local_variable] = native_read_field_@(member.name)(native_get_field_@(member.name)_message(messageHandle, i__local_variable));
@[        elif isinstance(member.type.value_type, AbstractString)]@
              IntPtr pStr_@(get_field_name(type_name, member.name)) = native_read_field_@(member.name)(native_get_field_@(member.name)_message(messageHandle, i__local_variable));
              @(get_field_name(type_name, member.name))[i__local_variable] = Marshal.PtrToStringAnsi(pStr_@(get_field_name(type_name, member.name)));
@[        elif isinstance(member.type.value_type, AbstractWString)]@
              // TODO: Unicode types are not supported  
@[        else]@
              @(get_field_name(type_name, member.name))[i__local_variable] = new @(get_dotnet_type(member.type.value_type))();
              @(get_field_name(type_name, member.name))[i__local_variable].__ReadFromHandle(native_get_field_@(member.name)_message(messageHandle, i__local_variable));
@[        end if]
          }
      }
@[    elif isinstance(member.type, AbstractSequence)]@
      {
          int size__local_variable = native_getsize_field_@(member.name)_message(messageHandle);
          @(get_field_name(type_name, member.name)) = new List<@(get_dotnet_type(member.type.value_type))>(size__local_variable);
          for (int i__local_variable = 0; i__local_variable < size__local_variable; i__local_variable++)
          {
@[        if isinstance(member.type.value_type, BasicType)]@
              @(get_field_name(type_name, member.name)).Add(native_read_field_@(member.name)(native_get_field_@(member.name)_message(messageHandle, i__local_variable)));
@[        elif isinstance(member.type.value_type, AbstractString)]@
              IntPtr pStr_@(get_field_name(type_name, member.name)) = native_read_field_@(member.name)(native_get_field_@(member.name)_message(messageHandle, i__local_variable));
              @(get_field_name(type_name, member.name)).Add(Marshal.PtrToStringAnsi(pStr_@(get_field_name(type_name, member.name))));
@[        elif isinstance(member.type.value_type, AbstractWString)]@
              // TODO: Unicode types are not supported  
@[        else]@
              @(get_field_name(type_name, member.name)).Add(new @(get_dotnet_type(member.type.value_type))());
              @(get_field_name(type_name, member.name))[i__local_variable].__ReadFromHandle(native_get_field_@(member.name)_message(messageHandle, i__local_variable));
@[        end if]
          }
      }
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
        @(get_field_name(type_name, member.name)).__ReadFromHandle(native_get_field_@(member.name)_HANDLE(messageHandle));
@[    end if]@
@[end for]@
    }

    [global::System.ComponentModel.EditorBrowsable(global::System.ComponentModel.EditorBrowsableState.Never)]
    public void __WriteToHandle(global::System.IntPtr messageHandle) {
@[for member in message.structure.members]@
@[    if isinstance(member.type, Array) or isinstance(member.type, AbstractSequence)]@
        {
@[        if isinstance(member.type, Array)]@
            var count__local_variable = @(get_field_name(type_name, member.name)).Length;
            if (count__local_variable != @(member.type.size))
            {
                throw new Exception("Invalid size of array '@(get_field_name(type_name, member.name))'.");
            }
@[        elif isinstance(member.type, AbstractSequence)]@
            var count__local_variable = @(get_field_name(type_name, member.name)).Count;
@[            if member.type.has_maximum_size()]@
            if (count__local_variable > @(member.type.maximum_size))
            {
                throw new Exception("Invalid size of bounded sequence '@(get_field_name(type_name, member.name))'.");
            }
@[            end if]@
            if (!native_init_seqence_field_@(member.name)_message(messageHandle, count__local_variable))
            {
                throw new Exception("The method 'native_init_seqence_field_@(member.name)_message()' failed.");
            }
@[        end if]@
            for (var i__local_variable = 0; i__local_variable < count__local_variable; i__local_variable++)
            {
                var value__local_variable = @(get_field_name(type_name, member.name))[i__local_variable];
@[        if isinstance(member.type.value_type, BasicType) or isinstance(member.type.value_type, AbstractString)]@
                native_write_field_@(member.name)(native_get_field_@(member.name)_message(messageHandle, i__local_variable), value__local_variable);
@[        elif isinstance(member.type.value_type, AbstractWString)]
// TODO: Unicode types are not supported  
@[        else]@
                value__local_variable.__WriteToHandle(native_get_field_@(member.name)_message(messageHandle, i__local_variable));
@[        end if]@
            }
        }

@[    elif isinstance(member.type, AbstractWString)]@
// TODO: Unicode types are not supported
@[    elif isinstance(member.type, BasicType) or isinstance(member.type, AbstractString)]@
        native_write_field_@(member.name)(messageHandle, @(get_field_name(type_name, member.name)));
@[    else]@
        @(get_field_name(type_name, member.name)).__WriteToHandle(native_get_field_@(member.name)_HANDLE(messageHandle));
@[    end if]@
@[end for]@
    }

@[for constant in message.constants]@
    public const @(get_dotnet_type(constant.type)) @(constant.name) =
        @(constant_value_to_dotnet(constant.type, constant.value));
@[end for]@

@[for member in message.structure.members]@
@[    if isinstance(member.type, Array)]@
    public @(get_dotnet_type(member.type.value_type))[] @(get_field_name(type_name, member.name)) { get; set; }
@[    elif isinstance(member.type, AbstractSequence)]@
    public List<@(get_dotnet_type(member.type.value_type))> @(get_field_name(type_name, member.name)) { get; set; }
@[    elif isinstance(member.type, AbstractWString)]@
// TODO: Unicode types are not supported
@[    else]@
    public @(get_dotnet_type(member.type)) @(get_field_name(type_name, member.name)) { get; set; }
@[    end if]@
@[end for]@

    private sealed class Safe@(type_name)Handle : global::System.Runtime.InteropServices.SafeHandle
    {
        public Safe@(type_name)Handle() : base(global::System.IntPtr.Zero, true) { }

        public override bool IsInvalid => handle == global::System.IntPtr.Zero;

        protected override bool ReleaseHandle()
        {
            native_destroy_native_message(handle);
            return true;
        }
    }
}

}
