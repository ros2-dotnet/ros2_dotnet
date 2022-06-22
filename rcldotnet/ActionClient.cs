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

namespace ROS2
{
    public abstract class ActionClient
    {
        // Only allow internal subclasses.
        internal ActionClient()
        {
        }
    }

    public sealed class ActionClient<TAction, TGoal, TResult, TFeedback> : ActionClient
        where TAction : IRosActionDefinition<TGoal, TResult, TFeedback>
        where TGoal : IRosMessage, new()
        where TResult : IRosMessage, new()
        where TFeedback : IRosMessage, new()
    {
        // No public constructor.
        internal ActionClient()
        {
        }

        public bool ServiceIsReady()
        {
            throw new NotImplementedException();
        }

        public Task<ActionClientGoalHandle<TAction, TGoal, TResult, TFeedback>> SendGoalAsync(TGoal goal)
        {
            throw new NotImplementedException();
        }

        public Task<ActionClientGoalHandle<TAction, TGoal, TResult, TFeedback>> SendGoalAsync(TGoal goal, Action<TFeedback> feedbackCallback)
        {
            throw new NotImplementedException();
        }
    }
}
