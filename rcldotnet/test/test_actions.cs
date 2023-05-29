/* Copyright 2023 Stefan Hoffmann <stefan.hoffmann@schiller.de>
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
using System.Collections.Generic;
using System.Threading.Tasks;
using ROS2;
using test_msgs.action;
using Xunit;

namespace RCLdotnetTests
{
    // TODO: (sh) Notes for extended tests:
    // - Add tests for:
    //   - ActionClientGoalHandle.Stamp
    //   - ActionServerGoalHandle.IsActive, .IsExecuting, .Status, .Execute()
    //   - Node.CreateActionServer() that overrides goalCallback
    //   - Node.CreateActionServer() that overrides cancelCallback that doesn't
    //     allow cancel.
    // - Extend test coverage for:
    //   - ActionClientGoalHandle.Status
    // - Test that invalid state-transitions throw exceptions.
    //   (e.g. ActionServerGoalHandle.Canceled() when the cancelCallback didn't
    //   allow cancellation)
    // - Test like rclpy (look into rclcpp as well) with mock server/client to
    //   test the cases with different callback orderings.
    //   - https://github.com/ros2/rclpy/blob/rolling/rclpy/test/test_action_client.py
    //   - https://github.com/ros2/rclpy/blob/rolling/rclpy/test/test_action_server.py

    public sealed class TestActions
    {
        private readonly Node _serverNode;
        private readonly Node _clientNode;

        // Each test method get's its own instance of this class, so no need to
        // reset the fields here.
        private Guid _serverGoalId;
        private int _serverGoalOrder;
        private List<int> _clientFeedbackSequence = null;

        public TestActions()
        {
            RCLdotnet.Init();
            _serverNode = RCLdotnet.CreateNode("action_server_node");
            _clientNode = RCLdotnet.CreateNode("action_client_node");
        }

        [Fact]
        public void TestActionClientServerIsReady()
        {
            var actionClient = _clientNode.CreateActionClient<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback>("unittest_dotnet_fibonacci");

            SpinEachNodeOnce();
            Assert.False(actionClient.ServerIsReady());

            var actionServer = _serverNode.CreateActionServer<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback>("unittest_dotnet_fibonacci", HandleAccepted);

            SpinEachNodeOnce();
            Assert.True(actionClient.ServerIsReady());
        }

        [Fact]
        public void TestActionSucceed()
        {
            var actionServer = _serverNode.CreateActionServer<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback>("unittest_dotnet_fibonacci", HandleAccepted);
            var actionClient = _clientNode.CreateActionClient<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback>("unittest_dotnet_fibonacci");

            var goal = new Fibonacci_Goal();
            goal.Order = 3;
            var goalHandleTask = actionClient.SendGoalAsync(goal, HandleFeedback);

            SpinEachNodeUntilTaskCompleted(goalHandleTask);
            Assert.Equal(3, _serverGoalOrder);
            Assert.NotNull(_clientFeedbackSequence);
            Assert.Equal(new List<int> { 0, 1 }, _clientFeedbackSequence);

            var goalHandle = goalHandleTask.GetAwaiter().GetResult();
            Assert.True(goalHandle.Accepted);
            Assert.NotEqual(Guid.Empty, goalHandle.GoalId);
            Assert.Equal(_serverGoalId, goalHandle.GoalId);

            var resultTask = goalHandle.GetResultAsync();
            SpinEachNodeUntilTaskCompleted(resultTask);
            var result = resultTask.GetAwaiter().GetResult();
            Assert.Equal(new List<int> { 0, 1 }, result.Sequence);
            Assert.Equal(ActionGoalStatus.Succeeded, goalHandle.Status);
        }

        [Fact]
        public void TestActionAbort()
        {
            var actionServer = _serverNode.CreateActionServer<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback>("unittest_dotnet_fibonacci", HandleAcceptedWithAbort);
            var actionClient = _clientNode.CreateActionClient<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback>("unittest_dotnet_fibonacci");

            var goal = new Fibonacci_Goal();
            goal.Order = 23;
            var goalHandleTask = actionClient.SendGoalAsync(goal);

            SpinEachNodeUntilTaskCompleted(goalHandleTask);
            Assert.Equal(23, _serverGoalOrder);

            var goalHandle = goalHandleTask.GetAwaiter().GetResult();
            Assert.True(goalHandle.Accepted);
            Assert.NotEqual(Guid.Empty, goalHandle.GoalId);
            Assert.Equal(_serverGoalId, goalHandle.GoalId);

            var resultTask = goalHandle.GetResultAsync();
            SpinEachNodeUntilTaskCompleted(resultTask);
            var result = resultTask.GetAwaiter().GetResult();
            Assert.Equal(new List<int> { -1 }, result.Sequence);
            Assert.Equal(ActionGoalStatus.Aborted, goalHandle.Status);
        }

        [Fact]
        public void TestActionCanceled()
        {
            var actionServer = _serverNode.CreateActionServer<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback>("unittest_dotnet_fibonacci", HandleAcceptedWithCanceled, cancelCallback: HandleCancel);
            var actionClient = _clientNode.CreateActionClient<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback>("unittest_dotnet_fibonacci");

            var goal = new Fibonacci_Goal();
            goal.Order = int.MaxValue;
            var goalHandleTask = actionClient.SendGoalAsync(goal);

            SpinEachNodeUntilTaskCompleted(goalHandleTask);
            Assert.Equal(int.MaxValue, _serverGoalOrder);

            var goalHandle = goalHandleTask.GetAwaiter().GetResult();
            Assert.True(goalHandle.Accepted);
            Assert.NotEqual(Guid.Empty, goalHandle.GoalId);
            Assert.Equal(_serverGoalId, goalHandle.GoalId);

            var cancelGoalTask = goalHandle.CancelGoalAsync();
            SpinEachNodeUntilTaskCompleted(cancelGoalTask);
            cancelGoalTask.GetAwaiter().GetResult();

            var resultTask = goalHandle.GetResultAsync();
            SpinEachNodeUntilTaskCompleted(resultTask);
            var result = resultTask.GetAwaiter().GetResult();
            Assert.Equal(new List<int> { 42 }, result.Sequence);
            Assert.Equal(ActionGoalStatus.Canceled, goalHandle.Status);
        }

        private void HandleAccepted(ActionServerGoalHandle<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback> goalHandle)
        {
            _serverGoalId = goalHandle.GoalId;
            _serverGoalOrder = goalHandle.Goal.Order;

            var feedback = new Fibonacci_Feedback();
            feedback.Sequence = new List<int> { 0, 1 };
            goalHandle.PublishFeedback(feedback);

            SpinEachNodeOnce();

            var result = new Fibonacci_Result();
            result.Sequence = feedback.Sequence;
            goalHandle.Succeed(result);
        }

        private void HandleAcceptedWithAbort(ActionServerGoalHandle<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback> goalHandle)
        {
            _serverGoalId = goalHandle.GoalId;
            _serverGoalOrder = goalHandle.Goal.Order;

            var result = new Fibonacci_Result();
            result.Sequence = new List<int> { -1 };
            goalHandle.Abort(result);
        }

        private void HandleAcceptedWithCanceled(ActionServerGoalHandle<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback> goalHandle)
        {
            _ = HandleAcceptedWithCanceledAsync(goalHandle);
        }

        private async Task HandleAcceptedWithCanceledAsync(ActionServerGoalHandle<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback> goalHandle)
        {
            try
            {
                _serverGoalId = goalHandle.GoalId;
                _serverGoalOrder = goalHandle.Goal.Order;

                var result = new Fibonacci_Result();
                result.Sequence = new List<int> { 42 };

                for (int i = 0; i < 20; i++)
                {
                    if (goalHandle.IsCanceling)
                    {
                        goalHandle.Canceled(result);
                        return;
                    }

                    await Task.Delay(50);
                }

                // didn't get to the `IsCanceling` state -> abort that the final assertion fails
                goalHandle.Abort(result);
            }
            catch (Exception exception)
            {
                Console.WriteLine(exception.Message);
            }
        }

        private CancelResponse HandleCancel(ActionServerGoalHandle<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback> goalHandle)
        {
            return CancelResponse.Accept;
        }

        private void HandleFeedback(Fibonacci_Feedback feedback)
        {
            _clientFeedbackSequence = feedback.Sequence;
        }

        private void SpinEachNodeUntilTaskCompleted(Task task)
        {
            while (!task.IsCompleted)
            {
                SpinEachNodeOnce();
            }
        }

        private void SpinEachNodeOnce()
        {
            RCLdotnet.SpinOnce(_serverNode, 50);
            RCLdotnet.SpinOnce(_clientNode, 50);
        }
    }
}
