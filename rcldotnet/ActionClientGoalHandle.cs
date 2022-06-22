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

        public abstract ActionGoalStatus Status { get; }
    }

    public sealed class ActionClientGoalHandle<TAction, TGoal, TResult, TFeedback> : ActionClientGoalHandle
        where TAction : IRosActionDefinition<TGoal, TResult, TFeedback>
        where TGoal : IRosMessage, new()
        where TResult : IRosMessage, new()
        where TFeedback : IRosMessage, new()
    {
        // No public constructor.
        internal ActionClientGoalHandle()
        {
        }

        public override Guid GoalId { get; }

        public override bool Accepted { get; }

        public override Time Stamp { get; }

        public override ActionGoalStatus Status { get; }

        public Task CancelGoalAsync()
        {
            throw new NotImplementedException();
        }

        public Task<TResult> GetResultAsync()
        {
            throw new NotImplementedException();
        }
    }
}
