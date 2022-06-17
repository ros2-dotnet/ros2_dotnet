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

using System.Threading.Tasks;
using ROS2;
using Xunit;

namespace RCLdotnetTests
{
    public sealed class TestGuardConditions
    {
        [Fact]
        public void TestGuardCondition()
        {
            RCLdotnet.Init();
            var node = RCLdotnet.CreateNode("guard_condition");

            int guardConditionTriggeredCount = 0;

            var guardCondition = node.CreateGuardCondition(HandleGuardCondition);

            // spin once without trigger
            RCLdotnet.SpinOnce(node, 50);
            Assert.Equal(0, guardConditionTriggeredCount);

            // spin once with trigger
            guardCondition.Trigger();
            RCLdotnet.SpinOnce(node, 50);
            Assert.Equal(1, guardConditionTriggeredCount);

            // spin once without trigger
            RCLdotnet.SpinOnce(node, 50);
            Assert.Equal(1, guardConditionTriggeredCount);

            void HandleGuardCondition()
            {
                guardConditionTriggeredCount++;
            }
        }

        [Fact]
        public void TestGuardConditionFromBackgroundWorkerThread()
        {
            RCLdotnet.Init();
            var node = RCLdotnet.CreateNode("guard_condition");

            int guardConditionTriggeredCount = 0;

            var guardCondition = node.CreateGuardCondition(HandleGuardCondition);

            TriggerAfterDelay();

            // spin once and wait for trigger
            RCLdotnet.SpinOnce(node, 1000);
            Assert.Equal(1, guardConditionTriggeredCount);

            // spin once without trigger
            RCLdotnet.SpinOnce(node, 50);
            Assert.Equal(1, guardConditionTriggeredCount);

            void HandleGuardCondition()
            {
                guardConditionTriggeredCount++;
            }

            async void TriggerAfterDelay()
            {
                await Task.Delay(10).ConfigureAwait(false);
                guardCondition.Trigger();
            }
        }
    }
}
