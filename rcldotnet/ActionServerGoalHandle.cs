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

namespace ROS2
{
    public abstract class ActionServerGoalHandle
    {
        // Only allow internal subclasses.
        internal ActionServerGoalHandle()
        {
        }

        public abstract Guid GoalId { get; }

        public abstract bool IsActive { get; }

        public abstract bool IsCancelRequested { get; }

        public abstract ActionGoalStatus Status { get; }

        // In `rclpy` this calls the `executeCallback` after setting the state to executing.
        // In `rclcpp` this does not call the `executeCallback` (as there is none) but sets the state for the explicit `AcceptAndDefer` `GoalResponse`.
        public void Execute()
        {
            throw new NotImplementedException();
        }
    }

    public sealed class ActionServerGoalHandle<TAction, TGoal, TResult, TFeedback> : ActionServerGoalHandle
        where TAction : IRosActionDefinition<TGoal, TResult, TFeedback>
        where TGoal : IRosMessage, new()
        where TResult : IRosMessage, new()
        where TFeedback : IRosMessage, new()
    {
        // No public constructor.
        internal ActionServerGoalHandle()
        {
        }

        // `rclpy` uses the name `Request`, but the name from `rclcpp` `Goal` fits better.
        public TGoal Goal { get; }

        public override Guid GoalId => throw new NotImplementedException();

        public override bool IsActive => throw new NotImplementedException();

        public override bool IsCancelRequested => throw new NotImplementedException();

        public override ActionGoalStatus Status => throw new NotImplementedException();

        public void PublishFeedback(TFeedback feedback) => throw new NotImplementedException();


        // TODO: (sh) Decide which (or both?) of these methods should be exposed.
        // // "rclpy style"
        // public void Succeed() => throw new NotImplementedException();
        // public void Abort() => throw new NotImplementedException();
        // public void Canceled() => throw new NotImplementedException();

        // "rclcpp style"
        public void Succeed(TResult result)
        {
            throw new NotImplementedException();
        }

        public void Abort(TResult result)
        {
            throw new NotImplementedException();
        }

        public void Canceled(TResult result)
        {
            throw new NotImplementedException();
        }
    }
}
