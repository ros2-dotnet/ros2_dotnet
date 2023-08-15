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
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Threading.Tasks;
using action_msgs.msg;
using action_msgs.srv;
using unique_identifier_msgs.msg;

namespace ROS2
{
    /// <summary>
    /// A response returned by an action server callback when a goal is requested.
    /// </summary>
    public enum GoalResponse
    {
        /// <summary>
        /// Invalid default value.
        /// </summary>
        Default = 0,

        /// <summary>
        /// The goal is rejected and will not be executed.
        /// </summary>
        Reject = 1,

        // `rclpy` doesn't provide the option to defer the the execution (unless you override the `acceptCallback`)
        // Accept = 2,
        // Alternative from `rclcpp`:

        /// <summary>
        /// The server accepts the goal, and is going to begin execution immediately.
        /// </summary>
        AcceptAndExecute = 2,

        /// <summary>
        /// The server accepts the goal, and is going to execute it later.
        /// </summary>
        AcceptAndDefer = 3,
    }

    /// <summary>
    /// A response returned by an action server callback when a goal has been asked to be canceled.
    /// </summary>
    public enum CancelResponse
    {
        /// <summary>
        /// Invalid default value.
        /// </summary>
        Default = 0,

        /// <summary>
        /// The server will not try to cancel the goal.
        /// </summary>
        Reject = 1,

        /// <summary>
        /// The server has agreed to try to cancel the goal.
        /// </summary>
        Accept = 2,
    }

    internal enum ActionGoalEvent
    {
        // These values need to mirror rcl_action_goal_event_t.

        Execute = 0,
        CancelGoal,
        Succeed,
        Abort,
        Canceled,
    }

    public abstract class ActionServer
    {
        // Only allow internal subclasses.
        internal ActionServer()
        {
        }

        // ActionServer does intentionally (for now) not implement IDisposable
        // as this needs some extra consideration how the type works after its
        // internal handle is disposed. By relying on the GC/Finalizer of
        // SafeHandle the handle only gets Disposed if the action client is not
        // live anymore.
        internal abstract SafeActionServerHandle Handle { get; }

        internal abstract IRosMessage CreateSendGoalRequest();
        internal abstract SafeHandle CreateSendGoalRequestHandle();

        internal abstract IRosActionGetResultRequest CreateGetResultRequest();
        internal abstract SafeHandle CreateGetResultRequestHandle();

        internal abstract void HandleGoalRequest(SafeRequestIdHandle requestIdHandle, IRosMessage goalRequest);

        internal abstract void HandleCancelRequest(SafeRequestIdHandle requestIdHandle, CancelGoal_Request cancelRequest);

        internal abstract void HandleResultRequest(SafeRequestIdHandle requestIdHandle, IRosActionGetResultRequest resultRequest);

        internal abstract void HandleGoalExpired();
    }

    public sealed class ActionServer<TAction, TGoal, TResult, TFeedback> : ActionServer
        where TAction : IRosActionDefinition<TGoal, TResult, TFeedback>
        where TGoal : IRosMessage, new()
        where TResult : IRosMessage, new()
        where TFeedback : IRosMessage, new()
    {
        private readonly Action<ActionServerGoalHandle<TAction, TGoal, TResult, TFeedback>> _acceptedCallback;

        private readonly Func<Guid, TGoal, GoalResponse> _goalCallback;

        private readonly Func<ActionServerGoalHandle<TAction, TGoal, TResult, TFeedback>, CancelResponse> _cancelCallback;

        private readonly ConcurrentDictionary<Guid, ActionServerGoalHandle<TAction, TGoal, TResult, TFeedback>> _goalHandles
            = new ConcurrentDictionary<Guid, ActionServerGoalHandle<TAction, TGoal, TResult, TFeedback>>();

        // No public constructor.
        internal ActionServer(
            SafeActionServerHandle handle,
            Action<ActionServerGoalHandle<TAction, TGoal, TResult, TFeedback>> acceptedCallback,
            Func<Guid, TGoal, GoalResponse> goalCallback,
            Func<ActionServerGoalHandle<TAction, TGoal, TResult, TFeedback>, CancelResponse> cancelCallback)
        {
            Handle = handle;
            _acceptedCallback = acceptedCallback;
            _goalCallback = goalCallback;
            _cancelCallback = cancelCallback;
        }

        internal override SafeActionServerHandle Handle { get; }

        internal override IRosMessage CreateSendGoalRequest()
        {
            return ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateSendGoalRequest();
        }

        internal override SafeHandle CreateSendGoalRequestHandle()
        {
            return ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateSendGoalRequestHandle();
        }

        internal override IRosActionGetResultRequest CreateGetResultRequest()
        {
            return ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateGetResultRequest();
        }

        internal override SafeHandle CreateGetResultRequestHandle()
        {
            return ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateGetResultRequestHandle();
        }

        internal IRosActionGetResultResponse<TResult> CreateGetResultResponse()
        {
            return ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateGetResultResponse();
        }

        private SafeHandle CreateGetResultResponseHandle()
        {
            return ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateGetResultResponseHandle();
        }

        private IRosActionSendGoalResponse CreateSendGoalResponse()
        {
            return ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateSendGoalResponse();
        }

        private SafeHandle CreateSendGoalResponseHandle()
        {
            return ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateSendGoalResponseHandle();
        }

        private SafeHandle CreateFeedbackMessageHandle()
        {
            return ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateFeedbackMessageHandle();
        }

        internal override void HandleGoalRequest(SafeRequestIdHandle requestIdHandle, IRosMessage goalRequest)
        {
            var goalRequestCasted = (IRosActionSendGoalRequest<TGoal>)goalRequest;

            var goal = goalRequestCasted.Goal;
            var uuid = (UUID)goalRequestCasted.GoalIdAsRosMessage;
            var goalId = uuid.ToGuid();

            // TODO: check if a goalHandle for the goalId already exists.
            //
            // rclpy does this before calling the goalCallback, rclcpp doesn't
            // do this and relies on rcl_action_accept_new_goal to return NULL.
            // In the rclpy case goalCallback doesn't get called, in rclcpp it
            // does.
            // rclpy logs this, sends a not accepted response.
            // rclcpp throws an error without logging and sending an response.
            // Currently this behaves like rclcpp.

            var goalResponse = _goalCallback(goalId, goal);

            var accepted =
                goalResponse == GoalResponse.AcceptAndExecute ||
                goalResponse == GoalResponse.AcceptAndDefer;

            ActionServerGoalHandle<TAction, TGoal, TResult, TFeedback> goalHandle = null;

            // goalInfo.Stamp will be filled in by native_rcl_action_accept_new_goal().
            GoalInfo goalInfo = null;

            if (accepted)
            {
                goalInfo = new GoalInfo();

                goalInfo.GoalId.Uuid = goalId.ToUuidByteArray();

                using (var goalInfoHandle = GoalInfo.__CreateMessageHandle())
                {
                    RCLdotnet.WriteToMessageHandle(goalInfo, goalInfoHandle);

                    var actionGoalHandle = new SafeActionGoalHandle();
                    RCLRet ret = RCLdotnetDelegates.native_rcl_action_accept_new_goal(ref actionGoalHandle, Handle, goalInfoHandle);
                    if (ret != RCLRet.Ok)
                    {
                        actionGoalHandle.Dispose();
                        throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_accept_new_goal)}() failed.");
                    }

                    // Read back goalInfo.Stamp.
                    RCLdotnet.ReadFromMessageHandle(goalInfo, goalInfoHandle);

                    goalHandle = new ActionServerGoalHandle<TAction, TGoal, TResult, TFeedback>(actionGoalHandle, this, goalId, goal);

                    if (!_goalHandles.TryAdd(goalId, goalHandle))
                    {
                        throw new Exception($"Two goals were accepted with the same Id '{goalId}'.");
                    };

                    if (goalResponse == GoalResponse.AcceptAndExecute)
                    {
                        ret = RCLdotnetDelegates.native_rcl_action_update_goal_state(goalHandle.Handle, ActionGoalEvent.Execute);
                        RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_update_goal_state)}() failed.");
                    }
                }
            }

            var response = CreateSendGoalResponse();
            response.Accepted = accepted;

            if (accepted)
            {
                Debug.Assert(goalInfo != null);

                // TODO: (sh) File Issues for different handling of this in rclcpp or rclpy?
                // rclpy fills in the response.Stamp but with a different timestamp than rcl_action_accept_new_goal() restamps.
                // rclcpp doesn't fill in the stamp at all.

                // Return the stamp that native_rcl_action_accept_new_goal filled in.
                response.StampAsRosMessage = goalInfo.Stamp;
            }

            SendGoalResponse(requestIdHandle, response);

            if (accepted)
            {
                Debug.Assert(goalHandle != null);

                PublishStatus();

                _acceptedCallback(goalHandle);
            }
        }

        private void SendGoalResponse(SafeRequestIdHandle requestHeaderHandle, IRosMessage goalResponse)
        {
            using (var goalResponseHandle = CreateSendGoalResponseHandle())
            {
                RCLdotnet.WriteToMessageHandle(goalResponse, goalResponseHandle);

                RCLRet ret = RCLdotnetDelegates.native_rcl_action_send_goal_response(Handle, requestHeaderHandle, goalResponseHandle);
                RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_send_goal_response)}() failed.");
            }
        }

        internal override void HandleCancelRequest(SafeRequestIdHandle requestIdHandle, CancelGoal_Request cancelRequest)
        {
            var cancelResponse = new CancelGoal_Response();

            using (var cancelRequestHandle = CancelGoal_Request.__CreateMessageHandle())
            using (var cancelResponseHandle = CancelGoal_Response.__CreateMessageHandle())
            {
                RCLdotnet.WriteToMessageHandle(cancelRequest, cancelRequestHandle);

                RCLRet ret = RCLdotnetDelegates.native_rcl_action_process_cancel_request(Handle, cancelRequestHandle, cancelResponseHandle);
                RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_process_cancel_request)}() failed.");

                RCLdotnet.ReadFromMessageHandle(cancelResponse, cancelResponseHandle);

                var goalsCancelingFromRcl = cancelResponse.GoalsCanceling;

                // Create and fill in new list for the goals the callback accepts to cancel.
                cancelResponse.GoalsCanceling = new List<GoalInfo>();

                foreach (var goalInfo in goalsCancelingFromRcl)
                {
                    var goalId = goalInfo.GoalId.ToGuid();

                    if (_goalHandles.TryGetValue(goalId, out var goalHandle))
                    {
                        var response = _cancelCallback(goalHandle);

                        if (response == CancelResponse.Accept)
                        {
                            ret = RCLdotnetDelegates.native_rcl_action_update_goal_state(goalHandle.Handle, ActionGoalEvent.CancelGoal);
                            if (ret == RCLRet.Ok)
                            {
                                cancelResponse.GoalsCanceling.Add(goalInfo);
                            }
                            else
                            {
                                // If the goal's just succeeded after user cancel callback
                                // that will generate an exception from invalid transition.
                                // -> Remove from response since goal has been succeeded.

                                // TODO: (sh) Logging.

                                // Don't add to goalsForResponse.
                            }
                        }
                    }
                    else
                    {
                        // Possibly the user doesn't care to track the goal handle
                        // Remove from response.
                        // or:
                        // We can't cancel a goal for a goalHandle we don't have.

                        // Don't add to goalsForResponse.
                    }
                }

                // TODO: (sh) rclpy doesn't set this? -> Create issue there?
                // If the user rejects all individual requests to cancel goals,
                // then we consider the top-level cancel request as rejected.
                if (goalsCancelingFromRcl.Count > 0 && cancelResponse.GoalsCanceling.Count == 0)
                {
                    cancelResponse.ReturnCode = CancelGoal_Response.ERROR_REJECTED;
                }

                if (cancelResponse.GoalsCanceling.Count > 0)
                {
                    // At least one goal state changed, publish a new status message.
                    PublishStatus();
                }

                RCLdotnet.WriteToMessageHandle(cancelResponse, cancelResponseHandle);

                ret = RCLdotnetDelegates.native_rcl_action_send_cancel_response(Handle, requestIdHandle, cancelResponseHandle);
                RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_send_cancel_response)}() failed.");
            }
        }

        internal override void HandleResultRequest(SafeRequestIdHandle requestIdHandle, IRosActionGetResultRequest resultRequest)
        {
            var uuid = (UUID)resultRequest.GoalIdAsRosMessage;
            var goalId = uuid.ToGuid();

            if (_goalHandles.TryGetValue(goalId, out var goalHandle))
            {
                // Using Task.ContinueWith here should deal with all the multi-threaded race conditions.
                goalHandle.ResultTask.ContinueWith(ResultTaskContinuationAction, requestIdHandle, TaskContinuationOptions.ExecuteSynchronously);
            }
            else
            {
                try
                {
                    var resultResponse = CreateGetResultResponse();
                    resultResponse.Status = GoalStatus.STATUS_UNKNOWN;
                    SendResultResponse(requestIdHandle, resultResponse);
                }
                finally
                {
                    requestIdHandle.Dispose();
                }
            }
        }

        private void ResultTaskContinuationAction(Task<IRosActionGetResultResponse<TResult>> resultTask, object state)
        {
            var requestIdHandle = (SafeRequestIdHandle)state;

            try
            {
                SendResultResponse(requestIdHandle, resultTask.Result);
            }
            finally
            {
                requestIdHandle.Dispose();
            }
        }

        private void SendResultResponse(SafeRequestIdHandle requestHeaderHandle, IRosActionGetResultResponse<TResult> resultResponse)
        {
            using (var resultResponseHandle = CreateGetResultResponseHandle())
            {
                RCLdotnet.WriteToMessageHandle(resultResponse, resultResponseHandle);

                RCLRet ret = RCLdotnetDelegates.native_rcl_action_send_result_response(Handle, requestHeaderHandle, resultResponseHandle);
                RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_send_result_response)}() failed.");
            }
        }

        internal override void HandleGoalExpired()
        {
            // This expects only one goal to expire at a time (rclcpp does the same).
            // Loop until no new goals are expired.
            var goalInfo = new GoalInfo();
            int numExpired;

            using (var goalInfoHandle = GoalInfo.__CreateMessageHandle())
            {
                do
                {
                    RCLRet ret = RCLdotnetDelegates.native_rcl_action_expire_goals(Handle, goalInfoHandle, out numExpired);
                    RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_expire_goals)}() failed.");

                    if (numExpired > 0)
                    {
                        RCLdotnet.ReadFromMessageHandle(goalInfo, goalInfoHandle);
                        Guid goalId = goalInfo.GoalId.ToGuid();

                        _goalHandles.TryRemove(goalId, out _);
                    }
                } while (numExpired > 0);
            }
        }

        internal void HandleTerminalState(
            ActionServerGoalHandle<TAction, TGoal, TResult, TFeedback> actionServerGoalHandle,
            IRosActionGetResultResponse<TResult> response)
        {
            PublishStatus();
            actionServerGoalHandle.ResultTaskCompletionSource.SetResult(response);

            RCLRet ret = RCLdotnetDelegates.native_rcl_action_notify_goal_done(Handle);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_notify_goal_done)}() failed.");
        }

        internal void PublishStatus()
        {
            RCLRet ret = RCLdotnetDelegates.native_rcl_action_publish_status(Handle);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_publish_status)}() failed.");
        }

        internal void PublishFeedbackMessage(IRosMessage feedbackMessage)
        {
            using (var feedbackMessageHandle = CreateFeedbackMessageHandle())
            {
                RCLdotnet.WriteToMessageHandle(feedbackMessage, feedbackMessageHandle);

                RCLRet ret = RCLdotnetDelegates.native_rcl_action_publish_feedback(Handle, feedbackMessageHandle);
                RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_publish_feedback)}() failed.");
            }
        }
    }
}
