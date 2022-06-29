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

    public abstract class ActionServer
    {
        // Only allow internal subclasses.
        internal ActionServer()
        {
        }
    }

    public sealed class ActionServer<TAction, TGoal, TResult, TFeedback> : ActionServer
        where TAction : IRosActionDefinition<TGoal, TResult, TFeedback>
        where TGoal : IRosMessage, new()
        where TResult : IRosMessage, new()
        where TFeedback : IRosMessage, new()
    {
        // No public constructor.
        internal ActionServer()
        {
        }
    }
}
