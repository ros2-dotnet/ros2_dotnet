@{
from rosidl_generator_dotnet import get_field_name
from rosidl_generator_dotnet import get_dotnet_type
from rosidl_generator_dotnet import get_dotnet_type_for_message
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

type_name = action.namespaced_type.name

goal_type_name = get_dotnet_type_for_message(action.goal)
result_type_name = get_dotnet_type_for_message(action.result)
feedback_type_name = get_dotnet_type_for_message(action.feedback)

send_goal_request_type_name = get_dotnet_type_for_message(action.send_goal_service.request_message)
send_goal_response_type_name = get_dotnet_type_for_message(action.send_goal_service.response_message)
get_result_request_type_name = get_dotnet_type_for_message(action.get_result_service.request_message)
get_result_response_type_name = get_dotnet_type_for_message(action.get_result_service.response_message)
feedback_message_type_name = get_dotnet_type_for_message(action.feedback_message)

action_typename = '%s__%s' % ('__'.join(action.namespaced_type.namespaces), type_name)
}
namespace @('.'.join(action.namespaced_type.namespaces))
{
@# sealed class with private constructor -> no instance can be created
@# static classes can't implement an interface (or any other basetype),
@# but we need some type that can hold the typesupport and be passed to the Node.CreateActionServce/Client() method.
@# So sealed + private constructor is as static as it gets.
@# static abstract interface members are currently in preview, so maybe we could use the feature in the future.
@# (if hey add support to derive from static only interfaces in static classes)
@# Another option is to not use generics for passing the typesupport, but lets try this until we hit some wall.
    public sealed class @(type_name) : global::ROS2.IRosActionDefinition<
        global::@(goal_type_name),
        global::@(result_type_name),
        global::@(feedback_type_name)>
    {
        private static readonly DllLoadUtils dllLoadUtils;

        private @(type_name)()
        {
        }

        static @(type_name)()
        {
            dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
            IntPtr nativelibrary = dllLoadUtils.LoadLibrary("@(package_name)__dotnetext");

            IntPtr native_get_typesupport_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "@(action_typename)__get_typesupport");

            @(type_name).native_get_typesupport = (NativeGetTypeSupportType)Marshal.GetDelegateForFunctionPointer(
                native_get_typesupport_ptr, typeof(NativeGetTypeSupportType));
        }

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate IntPtr NativeGetTypeSupportType();

        private static NativeGetTypeSupportType native_get_typesupport = null;

@# This method gets called via reflection as static abstract interface members are not supported yet.
        [global::System.ComponentModel.EditorBrowsable(global::System.ComponentModel.EditorBrowsableState.Never)]
        public static IntPtr __GetTypeSupport() {
            return native_get_typesupport();
        }

@# This method gets called via reflection as static abstract interface members are not supported yet.
        [global::System.ComponentModel.EditorBrowsable(global::System.ComponentModel.EditorBrowsableState.Never)]
        public static global::ROS2.IRosActionSendGoalRequest<global::@(goal_type_name)> __CreateSendGoalRequest()
        {
            return new global::@(send_goal_request_type_name)();
        }

@# This method gets called via reflection as static abstract interface members are not supported yet.
        [global::System.ComponentModel.EditorBrowsable(global::System.ComponentModel.EditorBrowsableState.Never)]
        public static SafeHandle __CreateSendGoalRequestHandle()
        {
            return global::@(send_goal_request_type_name).__CreateMessageHandle();
        }

@# This method gets called via reflection as static abstract interface members are not supported yet.
        [global::System.ComponentModel.EditorBrowsable(global::System.ComponentModel.EditorBrowsableState.Never)]
        public static global::ROS2.IRosActionSendGoalResponse __CreateSendGoalResponse()
        {
            return new global::@(send_goal_response_type_name)();
        }

@# This method gets called via reflection as static abstract interface members are not supported yet.
        [global::System.ComponentModel.EditorBrowsable(global::System.ComponentModel.EditorBrowsableState.Never)]
        public static SafeHandle __CreateSendGoalResponseHandle()
        {
            return global::@(send_goal_response_type_name).__CreateMessageHandle();
        }

@# This method gets called via reflection as static abstract interface members are not supported yet.
        [global::System.ComponentModel.EditorBrowsable(global::System.ComponentModel.EditorBrowsableState.Never)]
        public static global::ROS2.IRosActionGetResultRequest __CreateGetResultRequest()
        {
            return new global::@(get_result_request_type_name)();
        }

@# This method gets called via reflection as static abstract interface members are not supported yet.
        [global::System.ComponentModel.EditorBrowsable(global::System.ComponentModel.EditorBrowsableState.Never)]
        public static SafeHandle __CreateGetResultRequestHandle()
        {
            return global::@(get_result_request_type_name).__CreateMessageHandle();
        }

@# This method gets called via reflection as static abstract interface members are not supported yet.
        [global::System.ComponentModel.EditorBrowsable(global::System.ComponentModel.EditorBrowsableState.Never)]
        public static global::ROS2.IRosActionGetResultResponse<global::@(result_type_name)> __CreateGetResultResponse()
        {
            return new global::@(get_result_response_type_name)();
        }

@# This method gets called via reflection as static abstract interface members are not supported yet.
        [global::System.ComponentModel.EditorBrowsable(global::System.ComponentModel.EditorBrowsableState.Never)]
        public static SafeHandle __CreateGetResultResponseHandle()
        {
            return global::@(get_result_response_type_name).__CreateMessageHandle();
        }

@# This method gets called via reflection as static abstract interface members are not supported yet.
        [global::System.ComponentModel.EditorBrowsable(global::System.ComponentModel.EditorBrowsableState.Never)]
        public static global::ROS2.IRosActionFeedbackMessage<global::@(feedback_type_name)> __CreateFeedbackMessage()
        {
            return new global::@(feedback_message_type_name)();
        }

@# This method gets called via reflection as static abstract interface members are not supported yet.
        [global::System.ComponentModel.EditorBrowsable(global::System.ComponentModel.EditorBrowsableState.Never)]
        public static SafeHandle __CreateFeedbackMessageHandle()
        {
            return global::@(feedback_message_type_name).__CreateMessageHandle();
        }
    }
}
