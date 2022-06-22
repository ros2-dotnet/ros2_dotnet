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
    public enum ActionGoalStatus
    {
        // see definition here: https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/msg/GoalStatus.msg

        // <summary>
        // Indicates status has not been properly set.
        // </summary>
        Unknown = 0,

        // <summary>
        // The goal has been accepted and is awaiting execution.
        // </summary>
        Accepted = 1,

        // <summary>
        // The goal is currently being executed by the action server.
        // </summary>
        Executing = 2,

        // <summary>
        // The client has requested that the goal be canceled and the action
        // server has accepted the cancel request.
        // </summary>
        Canceling = 3,

        // <summary>
        // The goal was achieved successfully by the action server.
        // </summary>
        Succeeded = 4,

        // <summary>
        // The goal was canceled after an external request from an
        // action client.
        // </summary>
        Canceled = 5,

        // <summary>
        // The goal was terminated by the action server without an
        // external request.
        // </summary>
        Aborted = 6,
    }
}
