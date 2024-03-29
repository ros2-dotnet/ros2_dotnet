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
    public interface IRosActionFeedbackMessage<TFeedback> : IRosMessage
        where TFeedback : IRosMessage, new()
    {
        // NOTICE: cyclic reference, see `IRosActionSendGoalRequest<TGoal>`
        // unique_identifier_msgs.msg.UUID GoalId { get; set; }

        // This will be implemented explicitly so it doesn't collide with fields
        // of the same name.
        IRosMessage GoalIdAsRosMessage { get; set; }

        TFeedback Feedback { get; set; }
    }
}
