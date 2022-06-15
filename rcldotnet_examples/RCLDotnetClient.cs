using System;
using System.Threading;
using ROS2;

namespace ConsoleApplication
{
    public static class RCLDotnetClient
    {
        public static void Main(string[] args)
        {
            RCLdotnet.Init();
            var node = RCLdotnet.CreateNode("client");

            var client = node.CreateClient<std_srvs.srv.SetBool, std_srvs.srv.SetBool_Request, std_srvs.srv.SetBool_Response>("test_dotnet_service_name");

            // TODO: (sh) Add WaitForService(timeout) method that observes the node graph.
            Console.WriteLine("Waiting for service...");
            while (RCLdotnet.Ok() && !client.ServiceIsReady())
            {
                Thread.Sleep(500);
            }

            var request = new std_srvs.srv.SetBool_Request();
            request.Data = true;

            var task = client.SendRequestAsync(request);

            Console.WriteLine("Sent request.");

            while (RCLdotnet.Ok())
            {
                RCLdotnet.SpinOnce(node, 500);

                if (task.IsCompletedSuccessfully)
                {
                    var response = task.Result;
                    Console.WriteLine($"Service responded with: Success '{response.Success}' Message '{response.Message}'");
                    break;
                }
                else if (task.IsFaulted)
                {
                    Console.WriteLine($"Task faulted. Exception {task.Exception}");
                    break;
                }
                else if (task.IsCanceled)
                {
                    Console.WriteLine("Task canceld.");
                    break;
                }
            }
        }
    }
}
