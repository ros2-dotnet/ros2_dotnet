using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using ROS2;
using test_msgs.action;

namespace ConsoleApplication
{
    public static class RCLDotnetActionServer
    {
        public static void Main(string[] args)
        {
            RCLdotnet.Init();
            var node = RCLdotnet.CreateNode("action_server");

            var actionServer = node.CreateActionServer<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback>("fibonacci", HandleAccepted, cancelCallback: HandleCancel);

            RCLdotnet.Spin(node);
        }

        private static void HandleAccepted(ActionServerGoalHandle<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback> goalHandle)
        {
            // Don't block in the callback.
            // -> Don't wait for the returned Task.
            _ = DoWorkWithGoal(goalHandle);
        }

        private static CancelResponse HandleCancel(ActionServerGoalHandle<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback> goalHandle)
        {
            return CancelResponse.Accept;
        }

        private static async Task DoWorkWithGoal(ActionServerGoalHandle<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback> goalHandle)
        {
            Console.WriteLine("Executing goal...");
            var feedback = new Fibonacci_Feedback();

            feedback.Sequence = new List<int> { 0, 1 };

            for (int i = 1; i < goalHandle.Goal.Order; i++)
            {
                if (goalHandle.IsCanceling)
                {
                    var cancelResult = new Fibonacci_Result();
                    cancelResult.Sequence = feedback.Sequence;

                    Console.WriteLine($"Canceled Result: {string.Join(", ", cancelResult.Sequence)}");
                    goalHandle.Canceled(cancelResult);
                    return;
                }

                feedback.Sequence.Add(feedback.Sequence[i] + feedback.Sequence[i - 1]);

                Console.WriteLine($"Feedback: {string.Join(", ", feedback.Sequence)}");
                goalHandle.PublishFeedback(feedback);

                // NOTE: This causes the code to resume in an background worker Thread.
                // Consider this when copying code from the example if additional synchronization is needed.
                await Task.Delay(1000);
            }

            var result = new Fibonacci_Result();
            result.Sequence = feedback.Sequence;

            Console.WriteLine($"Result: {string.Join(", ", result.Sequence)}");
            goalHandle.Succeed(result);
        }
    }
}
