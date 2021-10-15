@{
type_name = service.namespaced_type.name
msg_typename = '%s__%s' % ('__'.join(service.namespaced_type.namespaces), type_name)
}@

using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;

using ROS2.Interfaces;
using ROS2.Utils;

@{
TEMPLATE(
    'srv.msg.cs.em',
    package_name=package_name, message=service.request_message)
}@

@{
TEMPLATE(
    'srv.msg.cs.em',
    package_name=package_name, message=service.response_message)
}@

@[for ns in service.namespaced_type.namespaces]@
namespace @(ns)
{
@[end for]@

public class @(type_name) {
    private static readonly DllLoadUtils dllLoadUtils;

    public @(type_name)() { }

    static @(type_name)()
    {
        dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
        IntPtr nativelibrary = dllLoadUtils.LoadLibrary("@(package_name)__dotnetext");

        IntPtr native_get_typesupport_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "@(msg_typename)__get_typesupport");

        @(type_name).native_get_typesupport = (NativeGetTypeSupportType)Marshal.GetDelegateForFunctionPointer(
            native_get_typesupport_ptr, typeof(NativeGetTypeSupportType));
    }

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate IntPtr NativeGetTypeSupportType();

    private static NativeGetTypeSupportType native_get_typesupport = null;

    public static IntPtr _GET_TYPE_SUPPORT() {
        return native_get_typesupport();
    }
}

@[for ns in reversed(service.namespaced_type.namespaces)]@
}  // namespace @(ns)
@[end for]@