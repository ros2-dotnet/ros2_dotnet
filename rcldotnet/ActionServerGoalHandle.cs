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
using System.Threading.Tasks;
using action_msgs.msg;
using unique_identifier_msgs.msg;

namespace ROS2
{
    public abstract class ActionServerGoalHandle
    {
        // Only allow internal subclasses.
        internal ActionServerGoalHandle()
        {
        }

        public abstract Guid GoalId { get; }

        /// <summary>
        /// Indicate if goal is pending or executing.
        /// </summary>
        /// <value>False if goal has reached a terminal state.</value>
        public abstract bool IsActive { get; }

        /// <summary>
        /// Indicate if client has requested this goal be cancelled.
        /// </summary>
        /// <value>True if a cancellation request has been accepted for this goal.</value>
        public abstract bool IsCanceling { get; }

        /// <summary>
        /// Indicate if goal is executing.
        /// </summary>
        /// <value>True only if the goal is in an executing state.</value>
        public abstract bool IsExecuting { get; }

        public abstract ActionGoalStatus Status { get; }

        internal abstract SafeActionGoalHandle Handle { get; }

        // In `rclpy` this calls the `executeCallback` after setting the state to executing.
        // In `rclcpp` this does not call the `executeCallback` (as there is none) but sets the state for the explicit `AcceptAndDefer` `GoalResponse`.
        public abstract void Execute();
    }

    public sealed class ActionServerGoalHandle<TAction, TGoal, TResult, TFeedback> : ActionServerGoalHandle
        where TAction : IRosActionDefinition<TGoal, TResult, TFeedback>
        where TGoal : IRosMessage, new()
        where TResult : IRosMessage, new()
        where TFeedback : IRosMessage, new()
    {
        private readonly ActionServer<TAction, TGoal, TResult, TFeedback> _actionServer;

        // No public constructor.
        internal ActionServerGoalHandle(
            SafeActionGoalHandle handle,
            ActionServer<TAction, TGoal, TResult, TFeedback> actionServer,
            Guid goalId,
            TGoal goal)
        {
            Handle = handle;
            _actionServer = actionServer;
            Goal = goal;
            GoalId = goalId;
            ResultTaskCompletionSource = new TaskCompletionSource<IRosActionGetResultResponse<TResult>>();
        }

        // `rclpy` uses the name `Request`, but the name from `rclcpp` `Goal` fits better.
        public TGoal Goal { get; }

        public override Guid GoalId { get; }

        public override bool IsActive
        {
            get
            {
                int isActive = RCLdotnetDelegates.native_rcl_action_goal_handle_is_active(Handle);
                return isActive != 0;
            }
        }

        public override bool IsCanceling => Status == ActionGoalStatus.Canceling;

        public override bool IsExecuting => Status == ActionGoalStatus.Executing;

        public override ActionGoalStatus Status
        {
            get
            {
                RCLRet ret = RCLdotnetDelegates.native_rcl_action_goal_handle_get_status(Handle, out byte status);

                // In .NET properties should not throw exceptions. Should this
                // be converted to a method for this reason? -> No as the rcl
                // methods only do argument null checks for values which
                // rcldotnet should ensure that they are not null.
                RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_goal_handle_get_status)}() failed.");

                return (ActionGoalStatus)status;
            }
        }

        internal override SafeActionGoalHandle Handle { get; }

        internal TaskCompletionSource<IRosActionGetResultResponse<TResult>> ResultTaskCompletionSource { get; }

        internal Task<IRosActionGetResultResponse<TResult>> ResultTask => ResultTaskCompletionSource.Task;

        public override void Execute()
        {
            UpdateGoalState(ActionGoalEvent.Execute);

            _actionServer.PublishStatus();
        }

        public void PublishFeedback(TFeedback feedback)
        {
            IRosActionFeedbackMessage<TFeedback> feedbackMessage =
                ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>.CreateFeedbackMessage();

            var goalId = (UUID)feedbackMessage.GoalIdAsRosMessage;
            goalId.Uuid = GoalId.ToUuidByteArray();
            feedbackMessage.Feedback = feedback;
            _actionServer.PublishFeedbackMessage(feedbackMessage);
        }

        // TODO: (sh) Decide which (or both?) of these methods should be exposed.
        // // "rclpy style"
        // public void Succeed() => throw new NotImplementedException();
        // public void Abort() => throw new NotImplementedException();
        // public void Canceled() => throw new NotImplementedException();

        // "rclcpp style"
        public void Succeed(TResult result)
        {
            UpdateGoalState(ActionGoalEvent.Succeed);

            var response = _actionServer.CreateGetResultResponse();
            response.Status = GoalStatus.STATUS_SUCCEEDED;
            response.Result = result;
            _actionServer.HandleTerminalState(this, response);
        }

        public void Abort(TResult result)
        {
            UpdateGoalState(ActionGoalEvent.Abort);

            var response = _actionServer.CreateGetResultResponse();
            response.Status = GoalStatus.STATUS_ABORTED;
            response.Result = result;
            _actionServer.HandleTerminalState(this, response);
        }

        public void Canceled(TResult result)
        {
            UpdateGoalState(ActionGoalEvent.Canceled);

            var response = _actionServer.CreateGetResultResponse();
            response.Status = GoalStatus.STATUS_CANCELED;
            response.Result = result;
            _actionServer.HandleTerminalState(this, response);
        }

        private void UpdateGoalState(ActionGoalEvent actionGoalEvent)
        {
            RCLRet ret = RCLdotnetDelegates.native_rcl_action_update_goal_state(Handle, actionGoalEvent);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_action_update_goal_state)}() failed.");
        }
    }
}
