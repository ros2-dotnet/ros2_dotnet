/* Copyright 2016-2018 Esteve Fernandez <esteve@apache.org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

using System;
using System.Runtime.InteropServices;
using action_msgs.msg;
using action_msgs.srv;
using ROS2.Utils;

namespace ROS2
{
    internal static class RCLdotnetDelegates
    {
        internal static readonly DllLoadUtils _dllLoadUtils;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLInitType();

        internal static NativeRCLInitType native_rcl_init = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate void NativeRCLGetErrorStringType(
            byte[] buffer, int bufferSize);

        internal static NativeRCLGetErrorStringType native_rcl_get_error_string = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate void NativeRCLResetErrorType();

        internal static NativeRCLResetErrorType native_rcl_reset_error = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate int NativeRCLOkType();

        internal static NativeRCLOkType native_rcl_ok = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal delegate RCLRet NativeRCLCreateNodeHandleType(
            ref SafeNodeHandle nodeHandle, [MarshalAs(UnmanagedType.LPStr)] string nodeName, [MarshalAs(UnmanagedType.LPStr)] string nodeNamespace);

        internal static NativeRCLCreateNodeHandleType native_rcl_create_node_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal delegate RCLRet NativeRCLDestroyNodeHandleType(
            IntPtr nodeHandle);

        internal static NativeRCLDestroyNodeHandleType native_rcl_destroy_node_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLCreateGuardConditionHandleType(
            ref SafeGuardConditionHandle guardConditionHandle);

        internal static NativeRCLCreateGuardConditionHandleType native_rcl_create_guard_condition_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLDestroyGuardConditionHandleType(
            IntPtr guardConditionHandle);

        internal static NativeRCLDestroyGuardConditionHandleType native_rcl_destroy_guard_condition_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate IntPtr NativeRCLGetRMWIdentifierType();

        internal static NativeRCLGetRMWIdentifierType native_rcl_get_rmw_identifier = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLCreateWaitSetHandleType(
            ref SafeWaitSetHandle waitSetHandle, int numberOfSubscriptions,
            int numberOfGuardConditions, int numberOfTimers,
            int numberOfClients, int numberOfServices, int numberOfEvents);

        internal static NativeRCLCreateWaitSetHandleType native_rcl_create_wait_set_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLDestroyWaitSetType(IntPtr waitSetHandle);

        internal static NativeRCLDestroyWaitSetType native_rcl_destroy_wait_set_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLWaitSetClearType(SafeWaitSetHandle waitSetHandle);

        internal static NativeRCLWaitSetClearType native_rcl_wait_set_clear = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLWaitSetAddSubscriptionType(SafeWaitSetHandle waitSetHandle, SafeSubscriptionHandle subscriptionHandle);

        internal static NativeRCLWaitSetAddSubscriptionType native_rcl_wait_set_add_subscription = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLWaitSetAddServiceType(SafeWaitSetHandle waitSetHandle, SafeServiceHandle serviceHandle);

        internal static NativeRCLWaitSetAddServiceType native_rcl_wait_set_add_service = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLWaitSetAddClientType(SafeWaitSetHandle waitSetHandle, SafeClientHandle clientHandle);

        internal static NativeRCLWaitSetAddClientType native_rcl_wait_set_add_client = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLWaitSetAddGuardConditionType(SafeWaitSetHandle waitSetHandle, SafeGuardConditionHandle guardConditionHandle);

        internal static NativeRCLWaitSetAddGuardConditionType native_rcl_wait_set_add_guard_condition = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLWaitType(SafeWaitSetHandle waitSetHandle, long timeout);

        internal static NativeRCLWaitType native_rcl_wait = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate int NativeRCLWaitSetSubscriptionReady(SafeWaitSetHandle waitSetHandle, int index);

        internal static NativeRCLWaitSetSubscriptionReady native_rcl_wait_set_subscription_ready = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate int NativeRCLWaitSetClientReady(SafeWaitSetHandle waitSetHandle, int index);

        internal static NativeRCLWaitSetClientReady native_rcl_wait_set_client_ready = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate int NativeRCLWaitSetServiceReady(SafeWaitSetHandle waitSetHandle, int index);

        internal static NativeRCLWaitSetServiceReady native_rcl_wait_set_service_ready = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate int NativeRCLWaitSetGuardConditionReady(SafeWaitSetHandle waitSetHandle, int index);

        internal static NativeRCLWaitSetGuardConditionReady native_rcl_wait_set_guard_condition_ready = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLTakeType(SafeSubscriptionHandle subscriptionHandle, SafeHandle messageHandle);

        internal static NativeRCLTakeType native_rcl_take = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLCreateRequestIdHandleType(
            ref SafeRequestIdHandle requestIdHandle);

        internal static NativeRCLCreateRequestIdHandleType native_rcl_create_request_id_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLDestroyRequestIdHandleType(IntPtr requestIdHandle);

        internal static NativeRCLDestroyRequestIdHandleType native_rcl_destroy_request_id_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate long NativeRCLRequestIdGetSequenceNumberType(SafeRequestIdHandle requestIdHandle);

        internal static NativeRCLRequestIdGetSequenceNumberType native_rcl_request_id_get_sequence_number = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLTakeRequestType(SafeServiceHandle serviceHandle, SafeRequestIdHandle requestHeaderHandle, SafeHandle requestHandle);

        internal static NativeRCLTakeRequestType native_rcl_take_request = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLSendResponseType(SafeServiceHandle serviceHandle, SafeRequestIdHandle requestHeaderHandle, SafeHandle responseHandle);

        internal static NativeRCLSendResponseType native_rcl_send_response = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLTakeResponseType(SafeClientHandle clientHandle, SafeRequestIdHandle requestHeaderHandle, SafeHandle responseHandle);

        internal static NativeRCLTakeResponseType native_rcl_take_response = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionClientWaitSetGetNumEntriesType(
            SafeActionClientHandle actionClientHandle, out int numSubscriptions, out int numGuardConditions, out int numTimers, out int numClients, out int numServices);

        internal static NativeRCLActionClientWaitSetGetNumEntriesType native_rcl_action_client_wait_set_get_num_entries = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionWaitSetAddActionClientType(
            SafeWaitSetHandle waitSetHandle, SafeActionClientHandle actionClientHandle);

        internal static NativeRCLActionWaitSetAddActionClientType native_rcl_action_wait_set_add_action_client = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionClientWaitSetGetEntitiesReadyType(
            SafeWaitSetHandle waitSetHandle, SafeActionClientHandle actionClientHandle, out bool isFeedbackReady, out bool isStatusReady, out bool isGoalResponseReady, out bool isCancelResponseReady, out bool isResultResponseReady);

        internal static NativeRCLActionClientWaitSetGetEntitiesReadyType native_rcl_action_client_wait_set_get_entities_ready = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionTakeFeedbackType(
            SafeActionClientHandle actionClientHandle, SafeHandle feedbackMessageHandle);

        internal static NativeRCLActionTakeFeedbackType native_rcl_action_take_feedback = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionTakeStatusType(
            SafeActionClientHandle actionClientHandle, SafeHandle statusMessageHandle);

        internal static NativeRCLActionTakeStatusType native_rcl_action_take_status = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionTakeGoalResponseType(
            SafeActionClientHandle actionClientHandle, SafeRequestIdHandle requestHeaderHandle, SafeHandle goalResponseHandle);

        internal static NativeRCLActionTakeGoalResponseType native_rcl_action_take_goal_response = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionTakeCancelResponseType(
            SafeActionClientHandle actionClientHandle, SafeRequestIdHandle requestHeaderHandle, SafeHandle cancelResponseHandle);

        internal static NativeRCLActionTakeCancelResponseType native_rcl_action_take_cancel_response = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionTakeResultResponseType(
            SafeActionClientHandle actionClientHandle, SafeRequestIdHandle requestHeaderHandle, SafeHandle resultResponseHandle);

        internal static NativeRCLActionTakeResultResponseType native_rcl_action_take_result_response = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLCreateQosProfileHandleType(ref SafeQosProfileHandle qosProfileHandle);
        internal static NativeRCLCreateQosProfileHandleType native_rcl_create_qos_profile_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLDestroyQosProfileHandleType(IntPtr qosProfileHandle);
        internal static NativeRCLDestroyQosProfileHandleType native_rcl_destroy_qos_profile_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLWriteToQosProfileHandleType(
            SafeQosProfileHandle qosProfileHandle,
            int history,
            int depth,
            int reliability,
            int durability,
            ulong deadlineSec,
            ulong deadlineNsec,
            ulong lifespanSec,
            ulong lifespanNsec,
            int liveliness,
            ulong livelinessLeaseDurationSec,
            ulong livelinessLeaseDurationNsec,
            int avoidRosNamespaceConventions);

        internal static NativeRCLWriteToQosProfileHandleType native_rcl_write_to_qos_profile_handle = null;

        static RCLdotnetDelegates()
        {
            _dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
            IntPtr nativeLibrary = _dllLoadUtils.LoadLibrary("rcldotnet");

            IntPtr native_rcl_init_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_init");
            RCLdotnetDelegates.native_rcl_init =
                (NativeRCLInitType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_init_ptr, typeof(NativeRCLInitType));

            IntPtr native_rcl_get_error_string_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_get_error_string");
            RCLdotnetDelegates.native_rcl_get_error_string =
                (NativeRCLGetErrorStringType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_get_error_string_ptr, typeof(NativeRCLGetErrorStringType));

            IntPtr native_rcl_reset_error_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_reset_error");
            RCLdotnetDelegates.native_rcl_reset_error =
                (NativeRCLResetErrorType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_reset_error_ptr, typeof(NativeRCLResetErrorType));

            IntPtr native_rcl_get_rmw_identifier_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_get_rmw_identifier");
            RCLdotnetDelegates.native_rcl_get_rmw_identifier =
                (NativeRCLGetRMWIdentifierType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_get_rmw_identifier_ptr, typeof(NativeRCLGetRMWIdentifierType));

            IntPtr native_rcl_ok_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_ok");
            RCLdotnetDelegates.native_rcl_ok =
                (NativeRCLOkType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_ok_ptr, typeof(NativeRCLOkType));

            IntPtr native_rcl_create_node_handle_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_create_node_handle");
            RCLdotnetDelegates.native_rcl_create_node_handle =
                (NativeRCLCreateNodeHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_create_node_handle_ptr, typeof(NativeRCLCreateNodeHandleType));

            IntPtr native_rcl_destroy_node_handle_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_destroy_node_handle");
            RCLdotnetDelegates.native_rcl_destroy_node_handle =
                (NativeRCLDestroyNodeHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_destroy_node_handle_ptr, typeof(NativeRCLDestroyNodeHandleType));

            IntPtr native_rcl_create_guard_condition_handle_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_create_guard_condition_handle");
            RCLdotnetDelegates.native_rcl_create_guard_condition_handle =
                (NativeRCLCreateGuardConditionHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_create_guard_condition_handle_ptr, typeof(NativeRCLCreateGuardConditionHandleType));

            IntPtr native_rcl_destroy_guard_condition_handle_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_destroy_guard_condition_handle");
            RCLdotnetDelegates.native_rcl_destroy_guard_condition_handle =
                (NativeRCLDestroyGuardConditionHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_destroy_guard_condition_handle_ptr, typeof(NativeRCLDestroyGuardConditionHandleType));

            IntPtr native_rcl_create_wait_set_handle_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_create_wait_set_handle");
            RCLdotnetDelegates.native_rcl_create_wait_set_handle =
                (NativeRCLCreateWaitSetHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_create_wait_set_handle_ptr, typeof(NativeRCLCreateWaitSetHandleType));

            IntPtr native_rcl_destroy_wait_set_handle_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_destroy_wait_set_handle");
            RCLdotnetDelegates.native_rcl_destroy_wait_set_handle =
                (NativeRCLDestroyWaitSetType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_destroy_wait_set_handle_ptr, typeof(NativeRCLDestroyWaitSetType));

            IntPtr native_rcl_wait_set_clear_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_wait_set_clear");
            RCLdotnetDelegates.native_rcl_wait_set_clear =
                (NativeRCLWaitSetClearType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_wait_set_clear_ptr, typeof(NativeRCLWaitSetClearType));

            IntPtr native_rcl_wait_set_add_subscription_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_wait_set_add_subscription");
            RCLdotnetDelegates.native_rcl_wait_set_add_subscription =
                (NativeRCLWaitSetAddSubscriptionType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_wait_set_add_subscription_ptr, typeof(NativeRCLWaitSetAddSubscriptionType));

            IntPtr native_rcl_wait_set_add_service_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_wait_set_add_service");
            RCLdotnetDelegates.native_rcl_wait_set_add_service =
                (NativeRCLWaitSetAddServiceType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_wait_set_add_service_ptr, typeof(NativeRCLWaitSetAddServiceType));

            IntPtr native_rcl_wait_set_add_client_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_wait_set_add_client");
            RCLdotnetDelegates.native_rcl_wait_set_add_client =
                (NativeRCLWaitSetAddClientType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_wait_set_add_client_ptr, typeof(NativeRCLWaitSetAddClientType));

            IntPtr native_rcl_wait_set_add_guard_condition_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_wait_set_add_guard_condition");
            RCLdotnetDelegates.native_rcl_wait_set_add_guard_condition =
                (NativeRCLWaitSetAddGuardConditionType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_wait_set_add_guard_condition_ptr, typeof(NativeRCLWaitSetAddGuardConditionType));

            IntPtr native_rcl_wait_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_wait");
            RCLdotnetDelegates.native_rcl_wait =
                (NativeRCLWaitType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_wait_ptr, typeof(NativeRCLWaitType));

            IntPtr native_rcl_wait_set_subscription_ready_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_wait_set_subscription_ready");
            RCLdotnetDelegates.native_rcl_wait_set_subscription_ready =
                (NativeRCLWaitSetSubscriptionReady)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_wait_set_subscription_ready_ptr, typeof(NativeRCLWaitSetSubscriptionReady));

            IntPtr native_rcl_wait_set_client_ready_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_wait_set_client_ready");
            RCLdotnetDelegates.native_rcl_wait_set_client_ready =
                (NativeRCLWaitSetClientReady)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_wait_set_client_ready_ptr, typeof(NativeRCLWaitSetClientReady));

            IntPtr native_rcl_wait_set_service_ready_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_wait_set_service_ready");
            RCLdotnetDelegates.native_rcl_wait_set_service_ready =
                (NativeRCLWaitSetServiceReady)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_wait_set_service_ready_ptr, typeof(NativeRCLWaitSetServiceReady));

            IntPtr native_rcl_wait_set_guard_condition_ready_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_wait_set_guard_condition_ready");
            RCLdotnetDelegates.native_rcl_wait_set_guard_condition_ready =
                (NativeRCLWaitSetGuardConditionReady)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_wait_set_guard_condition_ready_ptr, typeof(NativeRCLWaitSetGuardConditionReady));

            IntPtr native_rcl_take_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_take");
            RCLdotnetDelegates.native_rcl_take =
                (NativeRCLTakeType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_take_ptr, typeof(NativeRCLTakeType));

            IntPtr native_rcl_create_request_id_handle_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_create_request_id_handle");
            RCLdotnetDelegates.native_rcl_create_request_id_handle =
                (NativeRCLCreateRequestIdHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_create_request_id_handle_ptr, typeof(NativeRCLCreateRequestIdHandleType));

            IntPtr native_rcl_destroy_request_id_handle_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_destroy_request_id_handle");
            RCLdotnetDelegates.native_rcl_destroy_request_id_handle =
                (NativeRCLDestroyRequestIdHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_destroy_request_id_handle_ptr, typeof(NativeRCLDestroyRequestIdHandleType));

            IntPtr native_rcl_request_id_get_sequence_number_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_request_id_get_sequence_number");
            RCLdotnetDelegates.native_rcl_request_id_get_sequence_number =
                (NativeRCLRequestIdGetSequenceNumberType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_request_id_get_sequence_number_ptr, typeof(NativeRCLRequestIdGetSequenceNumberType));

            IntPtr native_rcl_take_request_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_take_request");
            RCLdotnetDelegates.native_rcl_take_request =
                (NativeRCLTakeRequestType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_take_request_ptr, typeof(NativeRCLTakeRequestType));

            IntPtr native_rcl_send_response_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_send_response");
            RCLdotnetDelegates.native_rcl_send_response =
                (NativeRCLSendResponseType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_send_response_ptr, typeof(NativeRCLSendResponseType));

            IntPtr native_rcl_take_response_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_take_response");
            RCLdotnetDelegates.native_rcl_take_response =
                (NativeRCLTakeResponseType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_take_response_ptr, typeof(NativeRCLTakeResponseType));

            IntPtr native_rcl_action_client_wait_set_get_num_entries_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_client_wait_set_get_num_entries");
            RCLdotnetDelegates.native_rcl_action_client_wait_set_get_num_entries =
                (NativeRCLActionClientWaitSetGetNumEntriesType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_client_wait_set_get_num_entries_ptr, typeof(NativeRCLActionClientWaitSetGetNumEntriesType));

            IntPtr native_rcl_action_wait_set_add_action_client_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_wait_set_add_action_client");
            RCLdotnetDelegates.native_rcl_action_wait_set_add_action_client =
                (NativeRCLActionWaitSetAddActionClientType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_wait_set_add_action_client_ptr, typeof(NativeRCLActionWaitSetAddActionClientType));

            IntPtr native_rcl_action_client_wait_set_get_entities_ready_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_client_wait_set_get_entities_ready");
            RCLdotnetDelegates.native_rcl_action_client_wait_set_get_entities_ready =
                (NativeRCLActionClientWaitSetGetEntitiesReadyType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_client_wait_set_get_entities_ready_ptr, typeof(NativeRCLActionClientWaitSetGetEntitiesReadyType));

            IntPtr native_rcl_action_take_feedback_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_take_feedback");
            RCLdotnetDelegates.native_rcl_action_take_feedback
                = (NativeRCLActionTakeFeedbackType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_take_feedback_ptr, typeof(NativeRCLActionTakeFeedbackType));

            IntPtr native_rcl_action_take_status_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_take_status");
            RCLdotnetDelegates.native_rcl_action_take_status =
                (NativeRCLActionTakeStatusType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_take_status_ptr, typeof(NativeRCLActionTakeStatusType));

            IntPtr native_rcl_action_take_goal_response_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_take_goal_response");
            RCLdotnetDelegates.native_rcl_action_take_goal_response =
                (NativeRCLActionTakeGoalResponseType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_take_goal_response_ptr, typeof(NativeRCLActionTakeGoalResponseType));

            IntPtr native_rcl_action_take_cancel_response_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_take_cancel_response");
            RCLdotnetDelegates.native_rcl_action_take_cancel_response =
                (NativeRCLActionTakeCancelResponseType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_take_cancel_response_ptr, typeof(NativeRCLActionTakeCancelResponseType));

            IntPtr native_rcl_action_take_result_response_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_take_result_response");
            RCLdotnetDelegates.native_rcl_action_take_result_response =
                (NativeRCLActionTakeResultResponseType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_take_result_response_ptr, typeof(NativeRCLActionTakeResultResponseType));

            IntPtr native_rcl_create_qos_profile_handle_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_create_qos_profile_handle");
            RCLdotnetDelegates.native_rcl_create_qos_profile_handle =
                (NativeRCLCreateQosProfileHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_create_qos_profile_handle_ptr, typeof(NativeRCLCreateQosProfileHandleType));

            IntPtr native_rcl_destroy_qos_profile_handle_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_destroy_qos_profile_handle");
            RCLdotnetDelegates.native_rcl_destroy_qos_profile_handle =
                (NativeRCLDestroyQosProfileHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_destroy_qos_profile_handle_ptr, typeof(NativeRCLDestroyQosProfileHandleType));

            IntPtr native_rcl_write_to_qos_profile_handle_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_write_to_qos_profile_handle");
            RCLdotnetDelegates.native_rcl_write_to_qos_profile_handle =
                (NativeRCLWriteToQosProfileHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_write_to_qos_profile_handle_ptr, typeof(NativeRCLWriteToQosProfileHandleType));
        }
    }

    public static class RCLdotnet
    {
        private static bool initialized = false;
        private static readonly object syncLock = new object();

        public static bool Ok()
        {
            return RCLdotnetDelegates.native_rcl_ok() != 0;
        }

        public static Node CreateNode(string nodeName)
        {
            return CreateNode(nodeName, "");
        }

        public static Node CreateNode(string nodeName, string nodeNamespace)
        {
            var nodeHandle = new SafeNodeHandle();
            RCLRet ret = RCLdotnetDelegates.native_rcl_create_node_handle(ref nodeHandle, nodeName, nodeNamespace);
            if (ret != RCLRet.Ok)
            {
                nodeHandle.Dispose();
                throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_create_node_handle)}() failed.");
            }

            Node node = new Node(nodeHandle);
            return node;
        }

        public static void Spin(Node node)
        {
            while (Ok())
            {
                SpinOnce(node, 500);
            }
        }

        private static SafeWaitSetHandle CreateWaitSet(
            int numberOfSubscriptions,
            int numberOfGuardConditions,
            int numberOfTimers,
            int numberOfClients,
            int numberOfServices,
            int numberOfEvents)
        {
            var waitSetHandle = new SafeWaitSetHandle();
            RCLRet ret = RCLdotnetDelegates.native_rcl_create_wait_set_handle(
                ref waitSetHandle, numberOfSubscriptions, numberOfGuardConditions,
                numberOfTimers, numberOfClients, numberOfServices, numberOfEvents);
            if (ret != RCLRet.Ok)
            {
                waitSetHandle.Dispose();
                throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_create_wait_set_handle)}() failed.");
            }

            return waitSetHandle;
        }

        private static void WaitSetClear(SafeWaitSetHandle waitSetHandle)
        {
            RCLRet ret = RCLdotnetDelegates.native_rcl_wait_set_clear(waitSetHandle);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_wait_set_clear)}() failed.");
        }

        private static void WaitSetAddSubscription(SafeWaitSetHandle waitSetHandle, SafeSubscriptionHandle subscriptionHandle)
        {
            RCLRet ret = RCLdotnetDelegates.native_rcl_wait_set_add_subscription(waitSetHandle, subscriptionHandle);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_wait_set_add_subscription)}() failed.");
        }

        private static void WaitSetAddService(SafeWaitSetHandle waitSetHandle, SafeServiceHandle serviceHandle)
        {
            RCLRet ret = RCLdotnetDelegates.native_rcl_wait_set_add_service(waitSetHandle, serviceHandle);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_wait_set_add_service)}() failed.");
        }

        private static void WaitSetAddClient(SafeWaitSetHandle waitSetHandle, SafeClientHandle clientHandle)
        {
            RCLRet ret = RCLdotnetDelegates.native_rcl_wait_set_add_client(waitSetHandle, clientHandle);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_wait_set_add_client)}() failed.");
        }

        private static void WaitSetAddGuardCondition(SafeWaitSetHandle waitSetHandle, SafeGuardConditionHandle guardConditionHandle)
        {
            RCLRet ret = RCLdotnetDelegates.native_rcl_wait_set_add_guard_condition(waitSetHandle, guardConditionHandle);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_wait_set_add_guard_condition)}() failed.");
        }

        private static void WaitSetAddActionClient(SafeWaitSetHandle waitSetHandle, SafeActionClientHandle actionClientHandle)
        {
            RCLRet ret = RCLdotnetDelegates.native_rcl_action_wait_set_add_action_client(waitSetHandle, actionClientHandle);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_wait_set_add_action_client)}() failed.");
        }

        /// <summary>
        /// Block until the wait set is ready or until the timeout has been exceeded.
        /// </summary>
        /// <param name="waitSetHandle">The wait set.</param>
        /// <param name="timeout">Timeout in ms.</param>
        /// <returns>True if wait set is ready, False on timeout.</returns>
        private static bool Wait(SafeWaitSetHandle waitSetHandle, long timeout)
        {
            long nsTimeout = timeout * 1000000;
            RCLRet ret = RCLdotnetDelegates.native_rcl_wait(waitSetHandle, nsTimeout);
            if (ret == RCLRet.Ok || ret == RCLRet.Timeout)
            {
                return ret == RCLRet.Ok;
            }
            else
            {
                throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_wait)}() failed.");
            }
        }

        private static SafeRequestIdHandle CreateRequestId()
        {
            var requestIdHandle = new SafeRequestIdHandle();
            RCLRet ret = RCLdotnetDelegates.native_rcl_create_request_id_handle(ref requestIdHandle);
            if (ret != RCLRet.Ok)
            {
                requestIdHandle.Dispose();
                throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_create_request_id_handle)}() failed.");
            }

            return requestIdHandle;
        }

        private static bool Take(Subscription subscription, IRosMessage message)
        {
            using (var messageHandle = subscription.CreateMessageHandle())
            {
                RCLRet ret = RCLdotnetDelegates.native_rcl_take(subscription.Handle, messageHandle);
                switch (ret)
                {
                    case RCLRet.Ok:
                        bool mustRelease = false;
                        try
                        {
                            // Using SafeHandles for __ReadFromHandle() is very tedious as this needs to be
                            // handled in generated code across multiple assemblies.
                            // Array and collection indexing would need to create SafeHandles everywere.
                            // It's not worth it, especialy considering the extra allocations for SafeHandles in
                            // arrays or collections that don't realy represent their own native recource.
                            messageHandle.DangerousAddRef(ref mustRelease);
                            message.__ReadFromHandle(messageHandle.DangerousGetHandle());
                        }
                        finally
                        {
                            if (mustRelease)
                            {
                                messageHandle.DangerousRelease();
                            }
                        }

                        return true;

                    case RCLRet.SubscriptionTakeFailed:
                        return false;

                    default:
                        throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_take)}() failed.");
                }
            }
        }

        private static bool TakeRequest(Service service, SafeRequestIdHandle requestHeaderHandle, IRosMessage request)
        {
            using (var requestHandle = service.CreateRequestHandle())
            {
                RCLRet ret = RCLdotnetDelegates.native_rcl_take_request(service.Handle, requestHeaderHandle, requestHandle);
                switch (ret)
                {
                    case RCLRet.Ok:
                        bool mustRelease = false;
                        try
                        {
                            // Using SafeHandles for __ReadFromHandle() is very tedious as this needs to be
                            // handled in generated code across multiple assemblies.
                            // Array and collection indexing would need to create SafeHandles everywere.
                            // It's not worth it, especialy considering the extra allocations for SafeHandles in
                            // arrays or collections that don't realy represent their own native recource.
                            requestHandle.DangerousAddRef(ref mustRelease);
                            request.__ReadFromHandle(requestHandle.DangerousGetHandle());
                        }
                        finally
                        {
                            if (mustRelease)
                            {
                                requestHandle.DangerousRelease();
                            }
                        }

                        return true;

                    case RCLRet.ServiceTakeFailed:
                        return false;

                    default:
                        throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_take_request)}() failed.");
                }
            }
        }

        private static bool TakeResponse(Client client, SafeRequestIdHandle requestHeaderHandle, IRosMessage response)
        {
            using (var responseHandle = client.CreateResponseHandle())
            {
                RCLRet ret = RCLdotnetDelegates.native_rcl_take_response(client.Handle, requestHeaderHandle, responseHandle);
                switch (ret)
                {
                    case RCLRet.Ok:
                        bool mustRelease = false;
                        try
                        {
                            // Using SafeHandles for __ReadFromHandle() is very tedious as this needs to be
                            // handled in generated code across multiple assemblies.
                            // Array and collection indexing would need to create SafeHandles everywere.
                            // It's not worth it, especialy considering the extra allocations for SafeHandles in
                            // arrays or collections that don't realy represent their own native recource.
                            responseHandle.DangerousAddRef(ref mustRelease);
                            response.__ReadFromHandle(responseHandle.DangerousGetHandle());
                        }
                        finally
                        {
                            if (mustRelease)
                            {
                                responseHandle.DangerousRelease();
                            }
                        }

                        return true;

                    case RCLRet.ClientTakeFailed:
                        return false;

                    default:
                        throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_take_response)}() failed.");
                }
            }
        }

        private static bool TakeFeedbackMessage(ActionClient actionClient, IRosMessage feedbackMessage)
        {
            using (var feedbackMessageHandle = actionClient.CreateFeedbackMessageHandle())
            {
                RCLRet ret = RCLdotnetDelegates.native_rcl_action_take_feedback(actionClient.Handle, feedbackMessageHandle);
                switch (ret)
                {
                    case RCLRet.Ok:
                        bool mustRelease = false;
                        try
                        {
                            // Using SafeHandles for __ReadFromHandle() is very tedious as this needs to be
                            // handled in generated code across multiple assemblies.
                            // Array and collection indexing would need to create SafeHandles everywhere.
                            // It's not worth it, especially considering the extra allocations for SafeHandles in
                            // arrays or collections that don't really represent their own native resource.
                            feedbackMessageHandle.DangerousAddRef(ref mustRelease);
                            feedbackMessage.__ReadFromHandle(feedbackMessageHandle.DangerousGetHandle());
                        }
                        finally
                        {
                            if (mustRelease)
                            {
                                feedbackMessageHandle.DangerousRelease();
                            }
                        }

                        return true;

                    case RCLRet.SubscriptionTakeFailed:
                        return false;

                    default:
                        throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_take_feedback)}() failed.");
                }
            }
        }

        private static bool TakeStatusMessage(ActionClient actionClient, IRosMessage statusMessage)
        {
            using (var statusMessageHandle = GoalStatusArray.__CreateMessageHandle())
            {
                RCLRet ret = RCLdotnetDelegates.native_rcl_action_take_status(actionClient.Handle, statusMessageHandle);
                switch (ret)
                {
                    case RCLRet.Ok:
                        bool mustRelease = false;
                        try
                        {
                            // Using SafeHandles for __ReadFromHandle() is very tedious as this needs to be
                            // handled in generated code across multiple assemblies.
                            // Array and collection indexing would need to create SafeHandles everywhere.
                            // It's not worth it, especially considering the extra allocations for SafeHandles in
                            // arrays or collections that don't really represent their own native resource.
                            statusMessageHandle.DangerousAddRef(ref mustRelease);
                            statusMessage.__ReadFromHandle(statusMessageHandle.DangerousGetHandle());
                        }
                        finally
                        {
                            if (mustRelease)
                            {
                                statusMessageHandle.DangerousRelease();
                            }
                        }

                        return true;

                    case RCLRet.SubscriptionTakeFailed:
                        return false;

                    default:
                        throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_take_status)}() failed.");
                }
            }
        }

        private static bool TakeGoalResponse(ActionClient actionClient, SafeRequestIdHandle requestHeaderHandle, IRosMessage goalResponse)
        {
            using (var goalResponseHandle = actionClient.CreateSendGoalResponseHandle())
            {
                RCLRet ret = RCLdotnetDelegates.native_rcl_action_take_goal_response(actionClient.Handle, requestHeaderHandle, goalResponseHandle);
                switch (ret)
                {
                    case RCLRet.Ok:
                        bool mustRelease = false;
                        try
                        {
                            // Using SafeHandles for __ReadFromHandle() is very tedious as this needs to be
                            // handled in generated code across multiple assemblies.
                            // Array and collection indexing would need to create SafeHandles everywhere.
                            // It's not worth it, especially considering the extra allocations for SafeHandles in
                            // arrays or collections that don't really represent their own native resource.
                            goalResponseHandle.DangerousAddRef(ref mustRelease);
                            goalResponse.__ReadFromHandle(goalResponseHandle.DangerousGetHandle());
                        }
                        finally
                        {
                            if (mustRelease)
                            {
                                goalResponseHandle.DangerousRelease();
                            }
                        }

                        return true;

                    case RCLRet.ClientTakeFailed:
                        return false;

                    default:
                        throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_take_goal_response)}() failed.");
                }
            }
        }

        private static bool TakeCancelResponse(ActionClient actionClient, SafeRequestIdHandle requestHeaderHandle, IRosMessage cancelResponse)
        {
            using (var cancelResponseHandle = CancelGoal_Response.__CreateMessageHandle())
            {
                RCLRet ret = RCLdotnetDelegates.native_rcl_action_take_cancel_response(actionClient.Handle, requestHeaderHandle, cancelResponseHandle);
                switch (ret)
                {
                    case RCLRet.Ok:
                        bool mustRelease = false;
                        try
                        {
                            // Using SafeHandles for __ReadFromHandle() is very tedious as this needs to be
                            // handled in generated code across multiple assemblies.
                            // Array and collection indexing would need to create SafeHandles everywhere.
                            // It's not worth it, especially considering the extra allocations for SafeHandles in
                            // arrays or collections that don't really represent their own native resource.
                            cancelResponseHandle.DangerousAddRef(ref mustRelease);
                            cancelResponse.__ReadFromHandle(cancelResponseHandle.DangerousGetHandle());
                        }
                        finally
                        {
                            if (mustRelease)
                            {
                                cancelResponseHandle.DangerousRelease();
                            }
                        }

                        return true;

                    case RCLRet.ClientTakeFailed:
                        return false;

                    default:
                        throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_take_cancel_response)}() failed.");
                }
            }
        }

        private static bool TakeResultResponse(ActionClient actionClient, SafeRequestIdHandle requestHeaderHandle, IRosMessage resultResponse)
        {
            using (var resultResponseHandle = actionClient.CreateGetResultResponseHandle())
            {
                RCLRet ret = RCLdotnetDelegates.native_rcl_action_take_result_response(actionClient.Handle, requestHeaderHandle, resultResponseHandle);
                switch (ret)
                {
                    case RCLRet.Ok:
                        bool mustRelease = false;
                        try
                        {
                            // Using SafeHandles for __ReadFromHandle() is very tedious as this needs to be
                            // handled in generated code across multiple assemblies.
                            // Array and collection indexing would need to create SafeHandles everywhere.
                            // It's not worth it, especially considering the extra allocations for SafeHandles in
                            // arrays or collections that don't really represent their own native resource.
                            resultResponseHandle.DangerousAddRef(ref mustRelease);
                            resultResponse.__ReadFromHandle(resultResponseHandle.DangerousGetHandle());
                        }
                        finally
                        {
                            if (mustRelease)
                            {
                                resultResponseHandle.DangerousRelease();
                            }
                        }

                        return true;

                    case RCLRet.ClientTakeFailed:
                        return false;

                    default:
                        throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_take_result_response)}() failed.");
                }
            }
        }

        private static void SendResponse(Service service, SafeRequestIdHandle requestHeaderHandle, IRosMessage response)
        {
            using (var responseHandle = service.CreateResponseHandle())
            {
                bool mustRelease = false;
                try
                {
                    // Using SafeHandles for __WriteToHandle() is very tedious as this needs to be
                    // handled in generated code across multiple assemblies.
                    // Array and collection indexing would need to create SafeHandles everywere.
                    // It's not worth it, especialy considering the extra allocations for SafeHandles in
                    // arrays or collections that don't realy represent their own native recource.
                    responseHandle.DangerousAddRef(ref mustRelease);
                    response.__WriteToHandle(responseHandle.DangerousGetHandle());
                }
                finally
                {
                    if (mustRelease)
                    {
                        responseHandle.DangerousRelease();
                    }
                }

                RCLRet ret = RCLdotnetDelegates.native_rcl_send_response(service.Handle, requestHeaderHandle, responseHandle);
                if (ret != RCLRet.Ok)
                {
                    throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_send_response)}() failed.");
                }
            }
        }

        public static void SpinOnce(Node node, long timeout)
        {
            int numberOfSubscriptions = node.Subscriptions.Count;
            int numberOfGuardConditions = node.GuardConditions.Count;
            int numberOfTimers = 0;
            int numberOfClients = node.Clients.Count;
            int numberOfServices = node.Services.Count;
            int numberOfEvents = 0;

            foreach (var actionClient in node.ActionClients)
            {
                RCLRet ret = RCLdotnetDelegates.native_rcl_action_client_wait_set_get_num_entries(
                    actionClient.Handle,
                    out int acNumberOfSubscriptions,
                    out int acNumberOfGuardConditions,
                    out int acNumberOfTimers,
                    out int acNumberOfClients,
                    out int acNumberOfServices);

                RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_client_wait_set_get_num_entries)}() failed.");

                numberOfSubscriptions += acNumberOfSubscriptions;
                numberOfGuardConditions += acNumberOfGuardConditions;
                numberOfTimers += acNumberOfTimers;
                numberOfClients += acNumberOfClients;
                numberOfServices += acNumberOfServices;
            }

            bool waitSetEmpty = numberOfSubscriptions == 0
                && numberOfGuardConditions == 0
                && numberOfTimers == 0
                && numberOfClients == 0
                && numberOfServices == 0
                && numberOfEvents == 0;

            if (waitSetEmpty)
            {
                // TODO: (sh) Should we sleep here for the timeout to avoid loops without sleep?
                // Avoid WaitSetEmpty return value from rcl_wait.
                return;
            }

            // TODO: (sh) Reuse wait set (see executer in rclc, rclcpp and rclpy)
            using (SafeWaitSetHandle waitSetHandle = CreateWaitSet(
                numberOfSubscriptions,
                numberOfGuardConditions,
                numberOfTimers,
                numberOfClients,
                numberOfServices,
                numberOfEvents))
            {
                WaitSetClear(waitSetHandle);

                foreach (Subscription subscription in node.Subscriptions)
                {
                    WaitSetAddSubscription(waitSetHandle, subscription.Handle);
                }

                foreach (var service in node.Services)
                {
                    WaitSetAddService(waitSetHandle, service.Handle);
                }

                foreach (var client in node.Clients)
                {
                    WaitSetAddClient(waitSetHandle, client.Handle);
                }

                foreach (var guardCondition in node.GuardConditions)
                {
                    WaitSetAddGuardCondition(waitSetHandle, guardCondition.Handle);
                }

                foreach (var actionClient in node.ActionClients)
                {
                    WaitSetAddActionClient(waitSetHandle, actionClient.Handle);
                }

                bool ready = Wait(waitSetHandle, timeout);
                if (!ready)
                {
                    return; // timeout
                }

                int subscriptionIndex = 0;
                foreach (Subscription subscription in node.Subscriptions)
                {
                    if (RCLdotnetDelegates.native_rcl_wait_set_subscription_ready(waitSetHandle, subscriptionIndex) != 0)
                    {
                        IRosMessage message = subscription.CreateMessage();
                        bool result = Take(subscription, message);
                        if (result)
                        {
                            subscription.TriggerCallback(message);
                        }
                    }

                    subscriptionIndex++;
                }

                // requestIdHandle gets reused for each element in the loop.
                using (SafeRequestIdHandle requestIdHandle = CreateRequestId())
                {
                    int serviceIndex = 0;
                    foreach (var service in node.Services)
                    {
                        if (RCLdotnetDelegates.native_rcl_wait_set_service_ready(waitSetHandle, serviceIndex) != 0)
                        {
                            var request = service.CreateRequest();
                            var response = service.CreateResponse();

                            var result = TakeRequest(service, requestIdHandle, request);
                            if (result)
                            {
                                service.TriggerCallback(request, response);

                                SendResponse(service, requestIdHandle, response);
                            }
                        }

                        serviceIndex++;
                    }

                    int clientIndex = 0;
                    foreach (var client in node.Clients)
                    {
                        if (RCLdotnetDelegates.native_rcl_wait_set_client_ready(waitSetHandle, clientIndex) != 0)
                        {
                            var response = client.CreateResponse();

                            var result = TakeResponse(client, requestIdHandle, response);
                            if (result)
                            {
                                var sequenceNumber = RCLdotnetDelegates.native_rcl_request_id_get_sequence_number(requestIdHandle);
                                client.HandleResponse(sequenceNumber, response);
                            }
                        }

                        clientIndex++;
                    }

                    foreach (var actionClient in node.ActionClients)
                    {
                        RCLRet ret = RCLdotnetDelegates.native_rcl_action_client_wait_set_get_entities_ready(
                            waitSetHandle,
                            actionClient.Handle,
                            out bool isFeedbackReady,
                            out bool isStatusReady,
                            out bool isGoalResponseReady,
                            out bool isCancelResponseReady,
                            out bool isResultResponseReady);

                        RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_client_wait_set_get_entities_ready)}() failed.");

                        if (isFeedbackReady)
                        {
                            var feedbackMessage = actionClient.CreateFeedbackMessage();

                            var result = TakeFeedbackMessage(actionClient, feedbackMessage);
                            if (result)
                            {
                                actionClient.HandleFeedbackMessage(feedbackMessage);
                            }
                        }

                        if (isStatusReady)
                        {
                            var statusMessage = new GoalStatusArray();

                            var result = TakeStatusMessage(actionClient, statusMessage);
                            if (result)
                            {
                                actionClient.HandleStatusMessage(statusMessage);
                            }
                        }

                        if (isGoalResponseReady)
                        {
                            var goalResponse = actionClient.CreateSendGoalResponse();

                            var result = TakeGoalResponse(actionClient, requestIdHandle, goalResponse);
                            if (result)
                            {
                                var sequenceNumber = RCLdotnetDelegates.native_rcl_request_id_get_sequence_number(requestIdHandle);
                                actionClient.HandleGoalResponse(sequenceNumber, goalResponse);
                            }
                        }

                        if (isCancelResponseReady)
                        {
                            var cancelResponse = new CancelGoal_Response();

                            var result = TakeCancelResponse(actionClient, requestIdHandle, cancelResponse);
                            if (result)
                            {
                                var sequenceNumber = RCLdotnetDelegates.native_rcl_request_id_get_sequence_number(requestIdHandle);
                                actionClient.HandleCancelResponse(sequenceNumber, cancelResponse);
                            }
                        }

                        if (isResultResponseReady)
                        {
                            var resultResponse = actionClient.CreateGetResultResponse();

                            var result = TakeResultResponse(actionClient, requestIdHandle, resultResponse);
                            if (result)
                            {
                                var sequenceNumber = RCLdotnetDelegates.native_rcl_request_id_get_sequence_number(requestIdHandle);
                                actionClient.HandleResultResponse(sequenceNumber, resultResponse);
                            }
                        }
                    }
                }

                int guardConditionIndex = 0;
                foreach (GuardCondition guardCondition in node.GuardConditions)
                {
                    if (RCLdotnetDelegates.native_rcl_wait_set_guard_condition_ready(waitSetHandle, guardConditionIndex) != 0)
                    {
                        guardCondition.TriggerCallback();
                    }

                    guardConditionIndex++;
                }
            }
        }

        public static void Init()
        {
            lock (syncLock)
            {
                if (!initialized)
                {
                    RCLRet ret = RCLdotnetDelegates.native_rcl_init();
                    RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_init)}() failed.");
                    initialized = true;
                }
            }
        }

        public static string GetRMWIdentifier()
        {
            IntPtr ptr = RCLdotnetDelegates.native_rcl_get_rmw_identifier();
            string rmw_identifier = Marshal.PtrToStringAnsi(ptr);
            return rmw_identifier;
        }
    }
}
