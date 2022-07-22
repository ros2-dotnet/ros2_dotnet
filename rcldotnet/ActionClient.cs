/* Copyright 2022 Stefan Hoffmann <stefan.hoffmann@schiller.de>
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
using System.Collections.Concurrent;
using System.Runtime.InteropServices;
using System.Threading.Tasks;
using action_msgs.msg;
using action_msgs.srv;
using builtin_interfaces.msg;
using ROS2.Utils;
using unique_identifier_msgs.msg;

namespace ROS2
{
    internal static class ActionClientDelegates
    {
        internal static readonly DllLoadUtils _dllLoadUtils;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionSendGoalRequestType(
            SafeActionClientHandle clientHandle, SafeHandle goalRequestHandle, out long sequenceNumber);

        internal static NativeRCLActionSendGoalRequestType native_rcl_action_send_goal_request = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionSendResultRequestType(
            SafeActionClientHandle clientHandle, SafeHandle resultRequestHandle, out long sequenceNumber);

        internal static NativeRCLActionSendResultRequestType native_rcl_action_send_result_request = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionSendCancelRequestType(
            SafeActionClientHandle clientHandle, SafeHandle cancelRequestHandle, out long sequenceNumber);

        internal static NativeRCLActionSendCancelRequestType native_rcl_action_send_cancel_request = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLActionServerIsAvailableType(
            SafeNodeHandle nodeHandle, SafeActionClientHandle clientHandle, out bool isAvailable);

        internal static NativeRCLActionServerIsAvailableType native_rcl_action_server_is_available = null;

        static ActionClientDelegates()
        {
            _dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
            IntPtr nativeLibrary = _dllLoadUtils.LoadLibrary("rcldotnet");

            IntPtr native_rcl_action_send_goal_request_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_send_goal_request");
            ActionClientDelegates.native_rcl_action_send_goal_request =
                (NativeRCLActionSendGoalRequestType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_send_goal_request_ptr, typeof(NativeRCLActionSendGoalRequestType));

            IntPtr native_rcl_action_send_result_request_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_send_result_request");
            ActionClientDelegates.native_rcl_action_send_result_request =
                (NativeRCLActionSendResultRequestType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_send_result_request_ptr, typeof(NativeRCLActionSendResultRequestType));

            IntPtr native_rcl_action_send_cancel_request_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_send_cancel_request");
            ActionClientDelegates.native_rcl_action_send_cancel_request =
                (NativeRCLActionSendCancelRequestType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_send_cancel_request_ptr, typeof(NativeRCLActionSendCancelRequestType));

            IntPtr native_rcl_action_server_is_available_ptr =
                _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_action_server_is_available");
            ActionClientDelegates.native_rcl_action_server_is_available =
                (NativeRCLActionServerIsAvailableType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_action_server_is_available_ptr, typeof(NativeRCLActionServerIsAvailableType));
        }
    }

    public abstract class ActionClient
    {
        // Only allow internal subclasses.
        internal ActionClient()
        {
        }

        // ActionClient does intentionally (for now) not implement IDisposable
        // as this needs some extra consideration how the type works after its
        // internal handle is disposed. By relying on the GC/Finalizer of
        // SafeHandle the handle only gets Disposed if the action client is not
        // live anymore.
        internal abstract SafeActionClientHandle Handle { get; }

        public abstract bool ServerIsReady();

        internal abstract IRosMessage CreateSendGoalResponse();

        internal abstract SafeHandle CreateSendGoalResponseHandle();

        internal abstract IRosMessage CreateGetResultResponse();

        internal abstract SafeHandle CreateGetResultResponseHandle();

        internal abstract IRosMessage CreateFeedbackMessage();

        internal abstract SafeHandle CreateFeedbackMessageHandle();

        internal abstract void HandleFeedbackMessage(IRosMessage feedbackMessage);

        internal abstract void HandleStatusMessage(GoalStatusArray statusMessage);

        internal abstract void HandleGoalResponse(long sequenceNumber, IRosMessage goalResponse);

        internal abstract void HandleCancelResponse(long sequenceNumber, CancelGoal_Response cancelResponse);

        internal abstract void HandleResultResponse(long sequenceNumber, IRosMessage resultResponse);
    }

    public sealed class ActionClient<TAction, TGoal, TResult, TFeedback> : ActionClient
        where TAction : IRosActionDefinition<TGoal, TResult, TFeedback>
        where TGoal : IRosMessage, new()
        where TResult : IRosMessage, new()
        where TFeedback : IRosMessage, new()
    {
        // ros2_java uses a WeakReference here. Not sure if its needed or not.
        private readonly Node _node;

        // TODO: (sh) rclpy uses week references to the goal handles here? Is this needed?
        private readonly ConcurrentDictionary<Guid, ActionClientGoalHandle<TAction, TGoal, TResult, TFeedback>> _goalHandles
            = new ConcurrentDictionary<Guid, ActionClientGoalHandle<TAction, TGoal, TResult, TFeedback>>();

        private readonly ConcurrentDictionary<long, PendingGoalRequest> _pendingGoalRequests
            = new ConcurrentDictionary<long, PendingGoalRequest>();

        private readonly ConcurrentDictionary<long, PendingCancelRequest> _pendingCancelRequests
            = new ConcurrentDictionary<long, PendingCancelRequest>();

        private readonly ConcurrentDictionary<long, PendingResultRequest> _pendingResultRequests
            = new ConcurrentDictionary<long, PendingResultRequest>();

        // No public constructor.
        internal ActionClient(SafeActionClientHandle handle, Node node)
        {
            Handle = handle;
            _node = node;
        }

        internal override SafeActionClientHandle Handle { get; }

        public override bool ServerIsReady()
        {
            RCLRet ret = ActionClientDelegates.native_rcl_action_server_is_available(_node.Handle, Handle, out var serverIsReady);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(ActionClientDelegates.native_rcl_action_server_is_available)}() failed.");

            return serverIsReady;
        }

        public Task<ActionClientGoalHandle<TAction, TGoal, TResult, TFeedback>> SendGoalAsync(TGoal goal)
        {
            return SendGoalAsyncInternal(goal, null);
        }

        public Task<ActionClientGoalHandle<TAction, TGoal, TResult, TFeedback>> SendGoalAsync(TGoal goal, Action<TFeedback> feedbackCallback)
        {
            return SendGoalAsyncInternal(goal, feedbackCallback);
        }

        private Task<ActionClientGoalHandle<TAction, TGoal, TResult, TFeedback>> SendGoalAsyncInternal(TGoal goal, Action<TFeedback> feedbackCallback)
        {
            long sequenceNumber;

            var goalId = Guid.NewGuid();
            var goalRequest = ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateSendGoalRequest();
            goalRequest.GoalIdAsRosMessage = goalId.ToUuidMsg();
            goalRequest.Goal = goal;

            using (var goalRequestHandle = ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateSendGoalRequestHandle())
            {
                RCLdotnet.WriteToMessageHandle(goalRequest, goalRequestHandle);

                RCLRet ret = ActionClientDelegates.native_rcl_action_send_goal_request(Handle, goalRequestHandle, out sequenceNumber);
                RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(ActionClientDelegates.native_rcl_action_send_goal_request)}() failed.");
            }

            var taskCompletionSource = new TaskCompletionSource<ActionClientGoalHandle<TAction, TGoal, TResult, TFeedback>>();
            var pendingGoalRequest = new PendingGoalRequest(goalId, taskCompletionSource, feedbackCallback);
            if (!_pendingGoalRequests.TryAdd(sequenceNumber, pendingGoalRequest))
            {
                // TODO: (sh) Add error handling/logging. SequenceNumber already taken.
            }

            return taskCompletionSource.Task;
        }

        // TODO: (sh) Add CancelAllGoalsAsync(), like rclcpp Client<ActionT>.async_cancel_all_goals()
        // TODO: (sh) Add CancelGoalsBeforeAsync(), like rclcpp Client<ActionT>.async_cancel_goals_before()

        // TODO: (sh) Maybe make this public.
        // (but then check if goalHandle belongs to this actionClient is needed)
        internal Task<CancelGoal_Response> CancelGoalAsync(ActionClientGoalHandle goalHandle)
        {
            long sequenceNumber;

            var cancelRequest = new CancelGoal_Request();
            cancelRequest.GoalInfo.GoalId.Uuid = goalHandle.GoalId.ToUuidByteArray();

            using (var cancelRequestHandle = CancelGoal_Request.__CreateMessageHandle())
            {
                RCLdotnet.WriteToMessageHandle(cancelRequest, cancelRequestHandle);

                RCLRet ret = ActionClientDelegates.native_rcl_action_send_cancel_request(Handle, cancelRequestHandle, out sequenceNumber);
                RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(ActionClientDelegates.native_rcl_action_send_cancel_request)}() failed.");
            }

            var taskCompletionSource = new TaskCompletionSource<CancelGoal_Response>();
            var pendingCancelRequest = new PendingCancelRequest(taskCompletionSource);
            if (!_pendingCancelRequests.TryAdd(sequenceNumber, pendingCancelRequest))
            {
                // TODO: (sh) Add error handling/logging. SequenceNumber already taken.
            }

            return taskCompletionSource.Task;
        }

        internal Task<TResult> GetResultAsync(
            ActionClientGoalHandle goalHandle,
            TaskCompletionSource<TResult> resultTaskCompletionSource)
        {
            try
            {
                long sequenceNumber;

                var resultRequest = ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateGetResultRequest();
                resultRequest.GoalIdAsRosMessage = goalHandle.GoalId.ToUuidMsg();

                using (var resultRequestHandle = ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateGetResultRequestHandle())
                {
                    RCLdotnet.WriteToMessageHandle(resultRequest, resultRequestHandle);

                    RCLRet ret = ActionClientDelegates.native_rcl_action_send_result_request(Handle, resultRequestHandle, out sequenceNumber);
                    RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(ActionClientDelegates.native_rcl_action_send_result_request)}() failed.");
                }

                var pendingResultRequest = new PendingResultRequest(resultTaskCompletionSource);
                if (!_pendingResultRequests.TryAdd(sequenceNumber, pendingResultRequest))
                {
                    // TODO: (sh) Add error handling/logging. SequenceNumber already taken.
                }

            }
            catch (Exception exception)
            {
                // Don't directly return Task.FromException() so that the tasks
                // returned from other threads get the exception as well.
                resultTaskCompletionSource.TrySetException(exception);
            }

            return resultTaskCompletionSource.Task;
        }

        internal override IRosMessage CreateSendGoalResponse()
        {
            return ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateSendGoalResponse();
        }

        internal override SafeHandle CreateSendGoalResponseHandle()
        {
            return ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateSendGoalResponseHandle();
        }

        internal override IRosMessage CreateGetResultResponse()
        {
            return ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateGetResultResponse();
        }

        internal override SafeHandle CreateGetResultResponseHandle()
        {
            return ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateGetResultResponseHandle();
        }

        internal override IRosMessage CreateFeedbackMessage()
        {
            return ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateFeedbackMessage();
        }

        internal override SafeHandle CreateFeedbackMessageHandle()
        {
            return ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateFeedbackMessageHandle();
        }

        internal override void HandleFeedbackMessage(IRosMessage feedbackMessage)
        {
            var feedbackMessageCasted = (IRosActionFeedbackMessage<TFeedback>)feedbackMessage;

            var goalIdUuid = (UUID)feedbackMessageCasted.GoalIdAsRosMessage;
            var goalId = goalIdUuid.ToGuid();

            if (!_goalHandles.TryGetValue(goalId, out var goalHandle))
            {
                // TODO: (sh) Add error handling/logging. GoalId not found.
                return;
            }

            goalHandle.FeedbackCallback?.Invoke(feedbackMessageCasted.Feedback);
        }

        internal override void HandleStatusMessage(GoalStatusArray statusMessage)
        {
            foreach (var statusMessageEntry in statusMessage.StatusList)
            {
                var goalId = statusMessageEntry.GoalInfo.GoalId.ToGuid();
                if (_goalHandles.TryGetValue(goalId, out var goalHandle))
                {
                    var status = statusMessageEntry.Status;
                    goalHandle.Status = ConvertGoalStatus(status);

                    // rclpy does this, rclcpp does not.
                    if (status == GoalStatus.STATUS_SUCCEEDED
                        || status == GoalStatus.STATUS_CANCELED
                        || status == GoalStatus.STATUS_ABORTED)
                    {
                        // Remove "done" goals from the dictionary.
                        // TODO: (sh) Should we remove goalHandles that are not in the status list as well? After a timeout?
                        _goalHandles.TryRemove(goalId, out _);
                    }
                }
            }
        }

        private static ActionGoalStatus ConvertGoalStatus(sbyte status)
        {
            switch (status)
            {
                case GoalStatus.STATUS_UNKNOWN:
                    return ActionGoalStatus.Unknown;

                case GoalStatus.STATUS_ACCEPTED:
                    return ActionGoalStatus.Accepted;

                case GoalStatus.STATUS_EXECUTING:
                    return ActionGoalStatus.Executing;

                case GoalStatus.STATUS_CANCELING:
                    return ActionGoalStatus.Canceling;

                case GoalStatus.STATUS_SUCCEEDED:
                    return ActionGoalStatus.Succeeded;

                case GoalStatus.STATUS_CANCELED:
                    return ActionGoalStatus.Canceled;

                case GoalStatus.STATUS_ABORTED:
                    return ActionGoalStatus.Aborted;

                default:
                    throw new ArgumentException("Unknown status.", nameof(status));
            }
        }

        internal override void HandleGoalResponse(long sequenceNumber, IRosMessage goalResponse)
        {
            if (!_pendingGoalRequests.TryRemove(sequenceNumber, out var pendingGoalRequest))
            {
                // TODO: (sh) Add error handling/logging. SequenceNumber not found.
                return;
            }

            var goalId = pendingGoalRequest.GoalId;
            var goalResponseCasted = (IRosActionSendGoalResponse)goalResponse;
            var accepted = goalResponseCasted.Accepted;
            var stamp = (Time)goalResponseCasted.StampAsRosMessage;

            var goalHandle = new ActionClientGoalHandle<TAction, TGoal, TResult, TFeedback>(this, goalId, accepted, stamp, pendingGoalRequest.FeedbackCallback);

            if (accepted)
            {
                if (!_goalHandles.TryAdd(goalId, goalHandle))
                {
                    var exception = new Exception($"Two goals were accepted with the same Id '{goalId}'.");
                    pendingGoalRequest.TaskCompletionSource.TrySetException(exception);
                    return;
                };
            }

            pendingGoalRequest.TaskCompletionSource.TrySetResult(goalHandle);
        }

        internal override void HandleCancelResponse(long sequenceNumber, CancelGoal_Response cancelResponse)
        {
            if (!_pendingCancelRequests.TryRemove(sequenceNumber, out var pendingCancelRequest))
            {
                // TODO: (sh) Add error handling/logging. SequenceNumber not found.
                return;
            }

            pendingCancelRequest.TaskCompletionSource.TrySetResult(cancelResponse);
        }

        internal override void HandleResultResponse(long sequenceNumber, IRosMessage resultResponse)
        {
            if (!_pendingResultRequests.TryRemove(sequenceNumber, out var pendingResultRequest))
            {
                // TODO: (sh) Add error handling/logging. SequenceNumber not found.
                return;
            }

            var resultResponseCasted = (IRosActionGetResultResponse<TResult>)resultResponse;
            var result = resultResponseCasted.Result;

            pendingResultRequest.TaskCompletionSource.TrySetResult(result);
        }

        private sealed class PendingGoalRequest
        {
            public PendingGoalRequest(
                Guid goalId,
                TaskCompletionSource<ActionClientGoalHandle<TAction, TGoal, TResult, TFeedback>> taskCompletionSource,
                Action<TFeedback> feedbackCallback)
            {
                GoalId = goalId;
                TaskCompletionSource = taskCompletionSource;
                FeedbackCallback = feedbackCallback;
            }

            public Guid GoalId { get; }
            public TaskCompletionSource<ActionClientGoalHandle<TAction, TGoal, TResult, TFeedback>> TaskCompletionSource { get; }
            public Action<TFeedback> FeedbackCallback { get; }
        }

        private sealed class PendingCancelRequest
        {
            public PendingCancelRequest(TaskCompletionSource<CancelGoal_Response> taskCompletionSource)
            {
                TaskCompletionSource = taskCompletionSource;
            }

            public TaskCompletionSource<CancelGoal_Response> TaskCompletionSource { get; }
        }

        private sealed class PendingResultRequest
        {
            public PendingResultRequest(TaskCompletionSource<TResult> taskCompletionSource)
            {
                TaskCompletionSource = taskCompletionSource;
            }

            public TaskCompletionSource<TResult> TaskCompletionSource { get; }
        }
    }
}
