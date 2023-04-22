using System;
using System.Reflection;
using System.Runtime;
using System.Runtime.InteropServices;
using System.Threading;
using ROS2;

namespace ConsoleApplication
{
    public static class RCLDotnetService
    {
        public static void Main(string[] args)
        {
            RCLdotnet.Init();
            var node = RCLdotnet.CreateNode("service");

            var service = node.CreateService<std_srvs.srv.SetBool, std_srvs.srv.SetBool_Request, std_srvs.srv.SetBool_Response>("test_dotnet_service_name", HandleRequest);

            RCLdotnet.Spin(node);
        }

        private static void HandleRequest(std_srvs.srv.SetBool_Request request, std_srvs.srv.SetBool_Response response)
        {
            Console.WriteLine($"Service got called with data {request.Data}");
            response.Success = true;
            response.Message = $"Got data {request.Data}";
        }
    }
}
