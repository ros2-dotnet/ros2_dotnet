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
    public interface IRosActionSendGoalRequest<TGoal> : IRosMessage
        where TGoal : IRosMessage, new()
    {
        // NOTICE: This would cause a cyclic reference:
        //
        // - `unique_identifier_msgs.msg.UUID` in the `unique_identifier_msgs`
        //   assembly references `IRosMessage` in the `rcldotnet_common`
        //   assembly.
        // - `IRosActionSendGoalRequest<TGoal>` in the `rcldotnet_common`
        //   assembly references `unique_identifier_msgs.msg.UUID` in the
        //   `unique_identifier_msgs` assembly.
        //
        // So we need a workaround:
        // - Use reflection later on to get to this.
        // - Or use types like `byte[]` (or ValueTuple<int, int> for
        //   `builtin_interfaces.msg.Time`) and generate accessor methods that
        //   convert and use those types.
        // - Or provide property `IRosMessage GoalIdRosMessage { get; set; }`
        //   and cast to the concrete type on usage.
        //
        // unique_identifier_msgs.msg.UUID GoalId { get; set; }

        TGoal Goal { get; set; }
    }
}
