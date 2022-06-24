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
using System.Threading;
using System.Threading.Tasks;
using builtin_interfaces.msg;

namespace ROS2
{
    public abstract class ActionClientGoalHandle
    {
        // Only allow internal subclasses.
        internal ActionClientGoalHandle()
        {
        }

        public abstract Guid GoalId { get; }

        public abstract bool Accepted { get; }

        public abstract Time Stamp { get; }

        public abstract ActionGoalStatus Status { get; internal set; }
    }

    public sealed class ActionClientGoalHandle<TAction, TGoal, TResult, TFeedback> : ActionClientGoalHandle
        where TAction : IRosActionDefinition<TGoal, TResult, TFeedback>
        where TGoal : IRosMessage, new()
        where TResult : IRosMessage, new()
        where TFeedback : IRosMessage, new()
    {
        private readonly ActionClient<TAction, TGoal, TResult, TFeedback> _actionClient;

        private TaskCompletionSource<TResult> _resultTaskCompletionSource;

        // No public constructor.
        internal ActionClientGoalHandle(
            ActionClient<TAction, TGoal, TResult, TFeedback> actionClient,
            Guid goalId,
            bool accepted,
            Time stamp,
            Action<TFeedback> feedbackCallback)
        {
            GoalId = goalId;
            Accepted = accepted;
            Stamp = stamp;
            FeedbackCallback = feedbackCallback;
            _actionClient = actionClient;
        }

        public override Guid GoalId { get; }

        public override bool Accepted { get; }

        public override Time Stamp { get; }

        public override ActionGoalStatus Status { get; internal set; }

        internal Action<TFeedback> FeedbackCallback { get; }

        // TODO: (sh) should we return the CancelGoal_Response? Wrap it in type with dotnet enum/Guid?
        public Task CancelGoalAsync()
        {
            return _actionClient.CancelGoalAsync(this);
        }

        public Task<TResult> GetResultAsync()
        {
            // Fast case to avoid allocation and calling Interlocked.CompareExchange.
            if (_resultTaskCompletionSource != null)
            {
                return _resultTaskCompletionSource.Task;
            }

            var resultTaskCompletionSource = new TaskCompletionSource<TResult>();

            if (Interlocked.CompareExchange(ref _resultTaskCompletionSource, resultTaskCompletionSource, null) != null)
            {
                // Some other thread was first.
                return _resultTaskCompletionSource.Task;
            }

            return _actionClient.GetResultAsync(this, resultTaskCompletionSource);
        }
    }
}
