using System;
using System.Threading;
using System.Threading.Tasks;
using ROS2;
using test_msgs.action;

namespace ConsoleApplication
{
    public static class RCLDotnetActionClient
    {
        public static void Main(string[] args)
        {
            RCLdotnet.Init();
            var node = RCLdotnet.CreateNode("action_client");

            var actionClient = node.CreateActionClient<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback>("fibonacci");

            var cts = new CancellationTokenSource();
            var task = DoWorkAsync(actionClient, cts.Token);

            while (RCLdotnet.Ok())
            {
                RCLdotnet.SpinOnce(node, 500);

                if (task.IsCompletedSuccessfully)
                {
                    break;
                }
                else if (task.IsFaulted)
                {
                    Console.WriteLine($"Task faulted. Exception {task.Exception}");
                    break;
                }
                else if (task.IsCanceled)
                {
                    Console.WriteLine("Task canceled.");
                    break;
                }
            }

            cts.Cancel();
            task.GetAwaiter().GetResult();
        }

        private static async Task DoWorkAsync(
            ActionClient<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback> actionClient,
            CancellationToken cancellationToken)
        {
            while (!actionClient.ServerIsReady())
            {
                cancellationToken.ThrowIfCancellationRequested();

                // NOTE: This causes the code to resume in an background worker Thread.
                // Consider this when copying code from the example if additional synchronization is needed.
                await Task.Delay(1000, cancellationToken);
            }

            var goal = new Fibonacci_Goal();
            goal.Order = 10;

            Console.WriteLine("SendGoal");

            ActionClientGoalHandle<Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback> goalHandleForCallback = null;

            var goalHandle = await actionClient.SendGoalAsync(goal, (Fibonacci_Feedback feedback) =>
            {
                Console.WriteLine($"Feedback: {string.Join(", ", feedback.Sequence)}");
                Console.WriteLine($"Status after Feedback: {goalHandleForCallback.Status}");
            });

            goalHandleForCallback = goalHandle;

            if (goalHandle.Accepted)
            {
                Console.WriteLine("Goal accepted.");

                var result = await goalHandle.GetResultAsync();
                Console.WriteLine($"Result: {string.Join(", ", result.Sequence)}");
                Console.WriteLine($"Status: {goalHandle.Status}");
            }
            else
            {
                Console.WriteLine("Goal not accepted.");
            }
        }
    }
}
