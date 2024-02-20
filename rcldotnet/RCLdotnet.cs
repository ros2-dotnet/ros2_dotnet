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
using sensor_msgs.msg;
using System.Collections.Generic;
using System.Diagnostics;

namespace ROS2
{
    internal static class RCLdotnetDelegates
    {
        internal static readonly DllLoadUtils _dllLoadUtils;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLInitType(
            int argc, [MarshalAs(UnmanagedType.LPArray, ArraySubType = UnmanagedType.LPStr)] string[] argv);

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
        internal delegate RCLRet NativeRCLWaitSetAddTimerType(SafeWaitSetHandle waitSetHandle, SafeTimerHandle timerHandle);

        internal static NativeRCLWaitSetAddTimerType native_rcl_wait_set_add_timer = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLWaitSetAddGuardConditionType(SafeWaitSetHandle waitSetHandle, SafeGuardConditionHandle guardConditionHandle);

        internal static NativeRCLWaitSetAddGuardConditionType native_rcl_wait_set_add_guard_condition = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLWaitType(SafeWaitSetHandle waitSetHandle, long timeout);

        internal static NativeRCLWaitType native_rcl_wait = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate int NativeRCLWaitSetReady(SafeWaitSetHandle waitSetHandle, int index);

        internal static NativeRCLWaitSetReady native_rcl_wait_set_subscription_ready = null;

        internal static NativeRCLWaitSetReady native_rcl_wait_set_client_ready = null;

        internal static NativeRCLWaitSetReady native_rcl_wait_set_timer_ready = null;

        internal static NativeRCLWaitSetReady native_rcl_wait_set_service_ready = null;

        internal static NativeRCLWaitSetReady native_rcl_wait_set_guard_condition_ready = null;

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
            SafeWaitSetHandle waitSetHandle, SafeActionClientHandle actionClientHandle, out int isFeedbackReady, out int isStatusReady, out int isGoalResponseReady, out int isCancelResponseReady, out int isResultResponseReady);

        internal static NativeRCLActionClientWaitSetGetEntitiesReadyType native_rcl_action_client_wait_set_get_entities_ready = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionServerWaitSetGetNumEntriesType(
                SafeActionServerHandle actionServerHandle, out int numSubscriptions, out int numGuardConditions, out int numTimers, out int numClients, out int numServices);

        internal static NativeRCLActionServerWaitSetGetNumEntriesType native_rcl_action_server_wait_set_get_num_entries = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionWaitSetAddActionServerType(
                SafeWaitSetHandle waitSetHandle, SafeActionServerHandle actionServerHandle);

        internal static NativeRCLActionWaitSetAddActionServerType native_rcl_action_wait_set_add_action_server = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionServerWaitSetGetEntitiesReadyType(
                SafeWaitSetHandle waitSetHandle, SafeActionServerHandle actionServerHandle, out int isGoalRequestReady, out int isCancelRequestReady, out int isResultRequestReady, out int isGoalExpired);

        internal static NativeRCLActionServerWaitSetGetEntitiesReadyType native_rcl_action_server_wait_set_get_entities_ready = null;

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
        internal delegate RCLRet NativeRCLActionTakeGoalRequestType(
            SafeActionServerHandle actionServerHandle, SafeRequestIdHandle requestHeaderHandle, SafeHandle goalRequestHandle);

        internal static NativeRCLActionTakeGoalRequestType native_rcl_action_take_goal_request = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionSendGoalResponseType(
            SafeActionServerHandle actionServerHandle, SafeRequestIdHandle requestHeaderHandle, SafeHandle gaolResponseHandle);

        internal static NativeRCLActionSendGoalResponseType native_rcl_action_send_goal_response = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionAcceptNewGoalType(
            ref SafeActionGoalHandle actionGoalHandleHandle, SafeActionServerHandle actionServerHandle, SafeHandle goalInfoHandle);

        internal static NativeRCLActionAcceptNewGoalType native_rcl_action_accept_new_goal = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionDestroyGoalHandleType(IntPtr actionGoalHandleHandle);

        internal static NativeRCLActionDestroyGoalHandleType native_rcl_action_destroy_goal_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionUpdateGoalStateType(
            SafeActionGoalHandle actionGoalHandleHandle, ActionGoalEvent goalEvent);

        internal static NativeRCLActionUpdateGoalStateType native_rcl_action_update_goal_state = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionPublishStatusType(SafeActionServerHandle actionServerHandle);
        internal static NativeRCLActionPublishStatusType native_rcl_action_publish_status = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionPublishFeedbackType(SafeActionServerHandle actionServerHandle, SafeHandle feedbackMessageHandle);
        internal static NativeRCLActionPublishFeedbackType native_rcl_action_publish_feedback = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionTakeCancelRequestType(
            SafeActionServerHandle actionServerHandle, SafeRequestIdHandle requestHeaderHandle, SafeHandle cancelRequestHandle);

        internal static NativeRCLActionTakeCancelRequestType native_rcl_action_take_cancel_request = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionProcessCancelRequestType(
            SafeActionServerHandle actionServerHandle, SafeHandle cancelRequestHandle, SafeHandle cancelResponseHandle);

        internal static NativeRCLActionProcessCancelRequestType native_rcl_action_process_cancel_request = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionSendCancelResponseType(
            SafeActionServerHandle actionServerHandle, SafeRequestIdHandle requestHeaderHandle, SafeHandle cancelResponseHandle);

        internal static NativeRCLActionSendCancelResponseType native_rcl_action_send_cancel_response = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionTakeResultRequestType(
            SafeActionServerHandle actionServerHandle, SafeRequestIdHandle requestHeaderHandle, SafeHandle resultRequestHandle);

        internal static NativeRCLActionTakeResultRequestType native_rcl_action_take_result_request = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionSendResultResponseType(
            SafeActionServerHandle actionServerHandle, SafeRequestIdHandle requestHeaderHandle, SafeHandle resultResponseHandle);

        internal static NativeRCLActionSendResultResponseType native_rcl_action_send_result_response = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionNotifyGoalDoneType(SafeActionServerHandle actionServerHandle);
        internal static NativeRCLActionNotifyGoalDoneType native_rcl_action_notify_goal_done = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionExpireGoalsType(
            SafeActionServerHandle actionServerHandle, SafeHandle goalInfoHandle, out int numExpired);

        internal static NativeRCLActionExpireGoalsType native_rcl_action_expire_goals = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate int NativeRCLActionGoalHandleIsActiveType(SafeActionGoalHandle actionGoalHandleHandle);
        internal static NativeRCLActionGoalHandleIsActiveType native_rcl_action_goal_handle_is_active = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionGoalHandleGetStatusType(SafeActionGoalHandle actionGoalHandleHandle, out byte status);
        internal static NativeRCLActionGoalHandleGetStatusType native_rcl_action_goal_handle_get_status = null;

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

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate IntPtr NativeRCLGetStringType(SafeHandle handle);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLCreateClockHandleType(ref SafeClockHandle clockHandle, int clockType);

        internal static NativeRCLCreateClockHandleType native_rcl_create_clock_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLDestroyClockHandleType(IntPtr clockHandle);

        internal static NativeRCLDestroyClockHandleType native_rcl_destroy_clock_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLCreateTimerHandleType(
            ref SafeTimerHandle timerHandle, SafeClockHandle clockHandle, Duration period, TimerCallback callback);

        internal static NativeRCLCreateTimerHandleType native_rcl_create_timer_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLDestroyTimerHandleType(IntPtr timerHandle);

        internal static NativeRCLDestroyTimerHandleType native_rcl_destroy_timer_handle = null;

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

            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_wait_set_add_timer), out native_rcl_wait_set_add_timer);

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
                (NativeRCLWaitSetReady)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_wait_set_subscription_ready_ptr, typeof(NativeRCLWaitSetReady));

            IntPtr native_rcl_wait_set_client_ready_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_wait_set_client_ready");
            RCLdotnetDelegates.native_rcl_wait_set_client_ready =
                (NativeRCLWaitSetReady)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_wait_set_client_ready_ptr, typeof(NativeRCLWaitSetReady));

            IntPtr native_rcl_wait_set_service_ready_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_wait_set_service_ready");
            RCLdotnetDelegates.native_rcl_wait_set_service_ready =
                (NativeRCLWaitSetReady)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_wait_set_service_ready_ptr, typeof(NativeRCLWaitSetReady));

            IntPtr native_rcl_wait_set_guard_condition_ready_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_wait_set_guard_condition_ready");
            RCLdotnetDelegates.native_rcl_wait_set_guard_condition_ready =
                (NativeRCLWaitSetReady)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_wait_set_guard_condition_ready_ptr, typeof(NativeRCLWaitSetReady));

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

            IntPtr native_rcl_action_server_wait_set_get_num_entries_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_server_wait_set_get_num_entries");
            RCLdotnetDelegates.native_rcl_action_server_wait_set_get_num_entries =
                (NativeRCLActionServerWaitSetGetNumEntriesType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_server_wait_set_get_num_entries_ptr, typeof(NativeRCLActionServerWaitSetGetNumEntriesType));

            IntPtr native_rcl_action_wait_set_add_action_server_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_wait_set_add_action_server");
            RCLdotnetDelegates.native_rcl_action_wait_set_add_action_server =
                (NativeRCLActionWaitSetAddActionServerType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_wait_set_add_action_server_ptr, typeof(NativeRCLActionWaitSetAddActionServerType));

            IntPtr native_rcl_action_server_wait_set_get_entities_ready_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_server_wait_set_get_entities_ready");
            RCLdotnetDelegates.native_rcl_action_server_wait_set_get_entities_ready =
                (NativeRCLActionServerWaitSetGetEntitiesReadyType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_server_wait_set_get_entities_ready_ptr, typeof(NativeRCLActionServerWaitSetGetEntitiesReadyType));

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

            IntPtr native_rcl_action_take_goal_request_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_take_goal_request");
            RCLdotnetDelegates.native_rcl_action_take_goal_request =
                (NativeRCLActionTakeGoalRequestType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_take_goal_request_ptr, typeof(NativeRCLActionTakeGoalRequestType));

            IntPtr native_rcl_action_send_goal_response_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_send_goal_response");
            RCLdotnetDelegates.native_rcl_action_send_goal_response =
                (NativeRCLActionSendGoalResponseType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_send_goal_response_ptr, typeof(NativeRCLActionSendGoalResponseType));

            IntPtr native_rcl_action_accept_new_goal_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_accept_new_goal");
            RCLdotnetDelegates.native_rcl_action_accept_new_goal =
                (NativeRCLActionAcceptNewGoalType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_accept_new_goal_ptr, typeof(NativeRCLActionAcceptNewGoalType));

            IntPtr native_rcl_action_destroy_goal_handle_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_destroy_goal_handle");
            RCLdotnetDelegates.native_rcl_action_destroy_goal_handle =
                (NativeRCLActionDestroyGoalHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_destroy_goal_handle_ptr, typeof(NativeRCLActionDestroyGoalHandleType));

            IntPtr native_rcl_action_update_goal_state_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_update_goal_state");
            RCLdotnetDelegates.native_rcl_action_update_goal_state =
                (NativeRCLActionUpdateGoalStateType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_update_goal_state_ptr, typeof(NativeRCLActionUpdateGoalStateType));

            IntPtr native_rcl_action_publish_status_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_publish_status");
            RCLdotnetDelegates.native_rcl_action_publish_status =
                (NativeRCLActionPublishStatusType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_publish_status_ptr, typeof(NativeRCLActionPublishStatusType));

            IntPtr native_rcl_action_publish_feedback_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_publish_feedback");
            RCLdotnetDelegates.native_rcl_action_publish_feedback =
                (NativeRCLActionPublishFeedbackType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_publish_feedback_ptr, typeof(NativeRCLActionPublishFeedbackType));

            IntPtr native_rcl_action_take_cancel_request_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_take_cancel_request");
            RCLdotnetDelegates.native_rcl_action_take_cancel_request =
                (NativeRCLActionTakeCancelRequestType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_take_cancel_request_ptr, typeof(NativeRCLActionTakeCancelRequestType));

            IntPtr native_rcl_action_process_cancel_request_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_process_cancel_request");
            RCLdotnetDelegates.native_rcl_action_process_cancel_request =
                (NativeRCLActionProcessCancelRequestType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_process_cancel_request_ptr, typeof(NativeRCLActionProcessCancelRequestType));

            IntPtr native_rcl_action_send_cancel_response_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_send_cancel_response");
            RCLdotnetDelegates.native_rcl_action_send_cancel_response =
                (NativeRCLActionSendCancelResponseType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_send_cancel_response_ptr, typeof(NativeRCLActionSendCancelResponseType));

            IntPtr native_rcl_action_take_result_request_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_take_result_request");
            RCLdotnetDelegates.native_rcl_action_take_result_request =
                (NativeRCLActionTakeResultRequestType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_take_result_request_ptr, typeof(NativeRCLActionTakeResultRequestType));

            IntPtr native_rcl_action_send_result_response_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_send_result_response");
            RCLdotnetDelegates.native_rcl_action_send_result_response =
                (NativeRCLActionSendResultResponseType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_send_result_response_ptr, typeof(NativeRCLActionSendResultResponseType));

            IntPtr native_rcl_action_notify_goal_done_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_notify_goal_done");
            RCLdotnetDelegates.native_rcl_action_notify_goal_done =
                (NativeRCLActionNotifyGoalDoneType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_notify_goal_done_ptr, typeof(NativeRCLActionNotifyGoalDoneType));

            IntPtr native_rcl_action_expire_goals_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_expire_goals");
            RCLdotnetDelegates.native_rcl_action_expire_goals =
                (NativeRCLActionExpireGoalsType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_expire_goals_ptr, typeof(NativeRCLActionExpireGoalsType));

            IntPtr native_rcl_action_goal_handle_is_active_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_goal_handle_is_active");
            RCLdotnetDelegates.native_rcl_action_goal_handle_is_active =
                (NativeRCLActionGoalHandleIsActiveType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_goal_handle_is_active_ptr, typeof(NativeRCLActionGoalHandleIsActiveType));

            IntPtr native_rcl_action_goal_handle_get_status_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_goal_handle_get_status");
            RCLdotnetDelegates.native_rcl_action_goal_handle_get_status =
                (NativeRCLActionGoalHandleGetStatusType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_goal_handle_get_status_ptr, typeof(NativeRCLActionGoalHandleGetStatusType));

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

            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_create_clock_handle), out native_rcl_create_clock_handle);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_destroy_clock_handle), out native_rcl_destroy_clock_handle);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_create_timer_handle), out native_rcl_create_timer_handle);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_destroy_timer_handle), out native_rcl_destroy_timer_handle);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_wait_set_timer_ready), out native_rcl_wait_set_timer_ready);
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

        public static Clock CreateClock(ClockType type = ClockType.RosTime)
        {
            var clockHandle = new SafeClockHandle();
            RCLRet ret = RCLdotnetDelegates.native_rcl_create_clock_handle(ref clockHandle, (int)type);
            if (ret != RCLRet.Ok)
            {
                clockHandle.Dispose();
                throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_create_clock_handle)}() failed.");
            }

            Clock clock = new Clock(clockHandle);
            return clock;
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

        private static void WaitSetAddTimer(SafeWaitSetHandle waitSetHandle, SafeTimerHandle timerHandle)
        {
            RCLRet ret = RCLdotnetDelegates.native_rcl_wait_set_add_timer(waitSetHandle, timerHandle);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_wait_set_add_timer)}() failed.");
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

        private static void WaitSetAddActionServer(SafeWaitSetHandle waitSetHandle, SafeActionServerHandle actionServerHandle)
        {
            RCLRet ret = RCLdotnetDelegates.native_rcl_action_wait_set_add_action_server(waitSetHandle, actionServerHandle);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_wait_set_add_action_server)}() failed.");
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
                        if (!(message is sensor_msgs.msg.Image))
                        {
                            ReadFromMessageHandle(message, messageHandle);
                        }
                        else
                        {
                            HackedReadFromMessageHandle(subscription, message, messageHandle);
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
                        ReadFromMessageHandle(request, requestHandle);
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
                        ReadFromMessageHandle(response, responseHandle);
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
                        ReadFromMessageHandle(feedbackMessage, feedbackMessageHandle);
                        return true;

                    case RCLRet.SubscriptionTakeFailed:
                    case RCLRet.ActionClientTakeFailed:
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
                        ReadFromMessageHandle(statusMessage, statusMessageHandle);
                        return true;

                    case RCLRet.SubscriptionTakeFailed:
                    case RCLRet.ActionClientTakeFailed:
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
                        ReadFromMessageHandle(goalResponse, goalResponseHandle);
                        return true;

                    case RCLRet.ClientTakeFailed:
                    case RCLRet.ActionClientTakeFailed:
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
                        ReadFromMessageHandle(cancelResponse, cancelResponseHandle);
                        return true;

                    case RCLRet.ClientTakeFailed:
                    case RCLRet.ActionClientTakeFailed:
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
                        ReadFromMessageHandle(resultResponse, resultResponseHandle);
                        return true;

                    case RCLRet.ClientTakeFailed:
                    case RCLRet.ActionClientTakeFailed:
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
                WriteToMessageHandle(response, responseHandle);

                RCLRet ret = RCLdotnetDelegates.native_rcl_send_response(service.Handle, requestHeaderHandle, responseHandle);
                RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_send_response)}() failed.");
            }
        }

        private static bool TakeGoalRequest(ActionServer actionServer, SafeRequestIdHandle requestHeaderHandle, IRosMessage goalRequest)
        {
            using (var goalRequestHandle = actionServer.CreateSendGoalRequestHandle())
            {
                RCLRet ret = RCLdotnetDelegates.native_rcl_action_take_goal_request(actionServer.Handle, requestHeaderHandle, goalRequestHandle);
                switch (ret)
                {
                    case RCLRet.Ok:
                        ReadFromMessageHandle(goalRequest, goalRequestHandle);
                        return true;

                    case RCLRet.ServiceTakeFailed:
                    case RCLRet.ActionServerTakeFailed:
                        return false;

                    default:
                        throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_take_goal_request)}() failed.");
                }
            }
        }

        private static bool TakeCancelRequest(ActionServer actionServer, SafeRequestIdHandle requestHeaderHandle, IRosMessage cancelRequest)
        {
            using (var cancelRequestHandle = CancelGoal_Request.__CreateMessageHandle())
            {
                RCLRet ret = RCLdotnetDelegates.native_rcl_action_take_cancel_request(actionServer.Handle, requestHeaderHandle, cancelRequestHandle);
                switch (ret)
                {
                    case RCLRet.Ok:
                        ReadFromMessageHandle(cancelRequest, cancelRequestHandle);
                        return true;

                    case RCLRet.ServiceTakeFailed:
                    case RCLRet.ActionServerTakeFailed:
                        return false;

                    default:
                        throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_take_cancel_request)}() failed.");
                }
            }
        }

        private static bool TakeResultRequest(ActionServer actionServer, SafeRequestIdHandle requestHeaderHandle, IRosMessage resultRequest)
        {
            using (var resultRequestHandle = actionServer.CreateGetResultRequestHandle())
            {
                RCLRet ret = RCLdotnetDelegates.native_rcl_action_take_result_request(actionServer.Handle, requestHeaderHandle, resultRequestHandle);
                switch (ret)
                {
                    case RCLRet.Ok:
                        ReadFromMessageHandle(resultRequest, resultRequestHandle);
                        return true;

                    case RCLRet.ServiceTakeFailed:
                    case RCLRet.ActionServerTakeFailed:
                        return false;

                    default:
                        throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_take_result_request)}() failed.");
                }
            }
        }

        public static void SpinOnce(Node node, long timeout)
        {
            int numberOfSubscriptions = node.Subscriptions.Count;
            int numberOfGuardConditions = node.GuardConditions.Count;
            int numberOfTimers = node.Timers.Count;
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

            foreach (var actionServer in node.ActionServers)
            {
                RCLRet ret = RCLdotnetDelegates.native_rcl_action_server_wait_set_get_num_entries(
                    actionServer.Handle,
                    out int asNumberOfSubscriptions,
                    out int asNumberOfGuardConditions,
                    out int asNumberOfTimers,
                    out int asNumberOfClients,
                    out int asNumberOfServices);

                RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_server_wait_set_get_num_entries)}() failed.");

                numberOfSubscriptions += asNumberOfSubscriptions;
                numberOfGuardConditions += asNumberOfGuardConditions;
                numberOfTimers += asNumberOfTimers;
                numberOfClients += asNumberOfClients;
                numberOfServices += asNumberOfServices;
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

                foreach (var actionServer in node.ActionServers)
                {
                    WaitSetAddActionServer(waitSetHandle, actionServer.Handle);
                }

                foreach (var timer in node.Timers)
                {
                    WaitSetAddTimer(waitSetHandle, timer.Handle);
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

                    int timerIndex = 0;
                    foreach (var timer in node.Timers)
                    {
                        if (RCLdotnetDelegates.native_rcl_wait_set_timer_ready(waitSetHandle, timerIndex) != 0)
                        {
                            try
                            {
                                timer.Call();
                            }
                            catch
                            {
                                // ignored
                            }
                        }

                        timerIndex++;
                    }

                    foreach (var actionClient in node.ActionClients)
                    {
                        RCLRet ret = RCLdotnetDelegates.native_rcl_action_client_wait_set_get_entities_ready(
                            waitSetHandle,
                            actionClient.Handle,
                            out int isFeedbackReady,
                            out int isStatusReady,
                            out int isGoalResponseReady,
                            out int isCancelResponseReady,
                            out int isResultResponseReady);

                        RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_client_wait_set_get_entities_ready)}() failed.");

                        // Check isGoalResponseReady so that a new goalHandle is
                        // already created if the status or feedback for the new
                        // goal is received.
                        if (isGoalResponseReady != 0)
                        {
                            var goalResponse = actionClient.CreateSendGoalResponse();

                            var result = TakeGoalResponse(actionClient, requestIdHandle, goalResponse);
                            if (result)
                            {
                                var sequenceNumber = RCLdotnetDelegates.native_rcl_request_id_get_sequence_number(requestIdHandle);
                                actionClient.HandleGoalResponse(sequenceNumber, goalResponse);
                            }
                        }

                        // Check isStatusReady before isFeedbackReady so that
                        // the feedback callback already has the newest status
                        // information if updates are received at the same time.
                        if (isStatusReady != 0)
                        {
                            var statusMessage = new GoalStatusArray();

                            var result = TakeStatusMessage(actionClient, statusMessage);
                            if (result)
                            {
                                actionClient.HandleStatusMessage(statusMessage);
                            }
                        }

                        if (isFeedbackReady != 0)
                        {
                            var feedbackMessage = actionClient.CreateFeedbackMessage();

                            var result = TakeFeedbackMessage(actionClient, feedbackMessage);
                            if (result)
                            {
                                actionClient.HandleFeedbackMessage(feedbackMessage);
                            }
                        }

                        if (isCancelResponseReady != 0)
                        {
                            var cancelResponse = new CancelGoal_Response();

                            var result = TakeCancelResponse(actionClient, requestIdHandle, cancelResponse);
                            if (result)
                            {
                                var sequenceNumber = RCLdotnetDelegates.native_rcl_request_id_get_sequence_number(requestIdHandle);
                                actionClient.HandleCancelResponse(sequenceNumber, cancelResponse);
                            }
                        }

                        if (isResultResponseReady != 0)
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

                    foreach (var actionServer in node.ActionServers)
                    {
                        RCLRet ret = RCLdotnetDelegates.native_rcl_action_server_wait_set_get_entities_ready(
                            waitSetHandle,
                            actionServer.Handle,
                            out int isGoalRequestReady,
                            out int isCancelRequestReady,
                            out int isResultRequestReady,
                            out int isGoalExpired);

                        RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_server_wait_set_get_entities_ready)}() failed.");

                        if (isGoalRequestReady != 0)
                        {
                            var goalRequest = actionServer.CreateSendGoalRequest();

                            var result = TakeGoalRequest(actionServer, requestIdHandle, goalRequest);
                            if (result)
                            {
                                actionServer.HandleGoalRequest(requestIdHandle, goalRequest);
                            }
                        }

                        if (isCancelRequestReady != 0)
                        {
                            var cancelRequest = new CancelGoal_Request();

                            var result = TakeCancelRequest(actionServer, requestIdHandle, cancelRequest);
                            if (result)
                            {
                                actionServer.HandleCancelRequest(requestIdHandle, cancelRequest);
                            }
                        }

                        if (isResultRequestReady != 0)
                        {
                            var resultRequest = actionServer.CreateGetResultRequest();

                            // We can't reuse requestIdHandle here as it is needed later when the result is ready.
                            // Create a new one for each GetResultRequest.
                            SafeRequestIdHandle resultRequestIdHandle = CreateRequestId();

                            var result = TakeResultRequest(actionServer, resultRequestIdHandle, resultRequest);
                            if (result)
                            {
                                actionServer.HandleResultRequest(resultRequestIdHandle, resultRequest);
                            }
                        }

                        if (isGoalExpired != 0)
                        {
                            actionServer.HandleGoalExpired();
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
                    string[] args = System.Environment.GetCommandLineArgs();
                    RCLRet ret = RCLdotnetDelegates.native_rcl_init(args.Length, args);
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


        // WORKAROUND IMAGES
        // public static Dictionary<Subscription, sensor_msgs.msg.Image> lastImages =new Dictionary<Subscription, sensor_msgs.msg.Image>();

        public static Dictionary<Subscription, sensor_msgs.msg.Image> imageSubscription = new Dictionary<Subscription, sensor_msgs.msg.Image>();

        public static Dictionary<sensor_msgs.msg.Image, byte[]> lastImagesBytes = new Dictionary<Image, byte[]>();

        public static object IMG_DATA_LOCK = new object();

        public static void HackedImage__ReadFromHandle(Subscription subscription, global::System.IntPtr messageHandle, sensor_msgs.msg.Image image)  
        {
            Debug.WriteLine("HackedImage__ReadFromHandle");
            var sw = System.Diagnostics.Stopwatch.StartNew();

            lock(IMG_DATA_LOCK)
            {
            image.Header.__ReadFromHandle(sensor_msgs.msg.Image.native_get_field_header_HANDLE(messageHandle));
            image.Height = sensor_msgs.msg.Image.native_read_field_height(messageHandle);
            image.Width = sensor_msgs.msg.Image.native_read_field_width(messageHandle);
            IntPtr pStr_Encoding = sensor_msgs.msg.Image.native_read_field_encoding(messageHandle);
            image.Encoding = Marshal.PtrToStringAnsi(pStr_Encoding);
            
            image.IsBigendian = sensor_msgs.msg.Image.native_read_field_is_bigendian(messageHandle);
            image.Step = sensor_msgs.msg.Image.native_read_field_step(messageHandle);
                {
                    int size__local_variable = sensor_msgs.msg.Image.native_getsize_field_data_message(messageHandle);

                    HackedMemoryCopy(subscription, messageHandle, image, size__local_variable);

                    // sw.Stop();

                    // var sw2 = System.Diagnostics.Stopwatch.StartNew();

                    // image.Data = new System.Collections.Generic.List<byte>(size__local_variable);
                    // for (int i__local_variable = 0; i__local_variable < size__local_variable; i__local_variable++)
                    // {
                    //     image.Data.Add(sensor_msgs.msg.Image.native_read_field_data(sensor_msgs.msg.Image.native_get_field_data_message(messageHandle, i__local_variable)));
                    // }

                    // sw2.Stop();
                    // Console.WriteLine($"copying image took: {sw.ElapsedTicks} ticks, {sw2.ElapsedTicks} ticks: {(float)(sw.ElapsedTicks - sw2.ElapsedTicks)/(float)sw.ElapsedTicks}");
                
                }
            }

            sw.Stop();
            Debug.WriteLine($"HackedImage__ReadFromHandle END: {sw.ElapsedMilliseconds} ms");
        }

           private static void HackedMemoryCopy(Subscription subscription, IntPtr messageHandle, sensor_msgs.msg.Image image, int size__local_variable)
        {
            IntPtr first = sensor_msgs.msg.Image.native_get_field_data_message(messageHandle, 0);

            // lastImagesHandles[image] = new Tuple<IntPtr, uint>(first, (uint)size__local_variable);

            byte[] bytes = null;
            if (imageSubscription.ContainsKey(subscription))
            {
                var lastImage = imageSubscription[subscription];
                bytes = lastImagesBytes[lastImage];
                lastImagesBytes.Remove(lastImage);
            }

            if (bytes == null || bytes.Length != size__local_variable)
            {
                Debug.WriteLine("HackedImage__ReadFromHandle: new byte buffer created");

                bytes = new byte[size__local_variable];
                // Console.WriteLine($"makmustReleaseing new size for image subscription: {size__local_variable} subscription: {subscription.GetHashCode()}");
            }


            Marshal.Copy(first,bytes, 0, (int)(size__local_variable));

            lastImagesBytes[image] = bytes;
            imageSubscription[subscription] = image;
        }

        internal static void HackedReadFromMessageHandle(Subscription subscription, IRosMessage message, SafeHandle messageHandle)
        {
            bool mustRelease = false;
            try
            {
                // Using SafeHandles for __ReadFromHandle() is very tedious as
                // this needs to be handled in generated code across multiple
                // assemblies. Array and collection indexing would need to
                // create SafeHandles everywhere. It's not worth it, especially
                // considering the extra allocations for SafeHandles in arrays
                // or collections that don't really represent their own native
                // resource.
                messageHandle.DangerousAddRef(ref mustRelease);
                HackedImage__ReadFromHandle(subscription, messageHandle.DangerousGetHandle(),message as sensor_msgs.msg.Image);
                //message.__ReadFromHandle(messageHandle.DangerousGetHandle());
            }
            catch(Exception e)
            {
                Debug.WriteLine("HackedImage__ReadFromHandle: exception");
            }
            finally
            {
                if (mustRelease)
                {
                    messageHandle.DangerousRelease();
                }
            }
        }

        internal static void ReadFromMessageHandle(IRosMessage message, SafeHandle messageHandle)
        {
            bool mustRelease = false;
            try
            {
                // Using SafeHandles for __ReadFromHandle() is very tedious as
                // this needs to be handled in generated code across multiple
                // assemblies. Array and collection indexing would need to
                // create SafeHandles everywhere. It's not worth it, especially
                // considering the extra allocations for SafeHandles in arrays
                // or collections that don't really represent their own native
                // resource.
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
        }

        internal static void WriteToMessageHandle(IRosMessage message, SafeHandle messageHandle)
        {
            bool mustRelease = false;
            try
            {
                // Using SafeHandles for __WriteToHandle() is very tedious as
                // this needs to be handled in generated code across multiple
                // assemblies. Array and collection indexing would need to
                // create SafeHandles everywhere. It's not worth it, especially
                // considering the extra allocations for SafeHandles in arrays
                // or collections that don't really represent their own native
                // resource.
                messageHandle.DangerousAddRef(ref mustRelease);
                message.__WriteToHandle(messageHandle.DangerousGetHandle());
            }
            finally
            {
                if (mustRelease)
                {
                    messageHandle.DangerousRelease();
                }
            }
        }

        internal static string GetStringFromNativeDelegate(RCLdotnetDelegates.NativeRCLGetStringType nativeDelegate, SafeHandle safeHandle)
        {
            bool mustRelease = false;
            try
            {
                // This avoids accessing a invalid/freed pointer if some other thread disposes the SafeNodeHandle.
                safeHandle.DangerousAddRef(ref mustRelease);
                IntPtr namePtr = nativeDelegate(safeHandle);
                return Marshal.PtrToStringAnsi(namePtr);
            }
            finally
            {
                if (mustRelease)
                {
                    safeHandle.DangerousRelease();
                }
            }
        }
    }
}
