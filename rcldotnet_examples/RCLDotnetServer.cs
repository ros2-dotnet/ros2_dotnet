using System;
using System.Reflection;
using System.Runtime;
using System.Runtime.InteropServices;
using System.Threading;

using ROS2;
using ROS2.Common;
using ROS2.Utils;

namespace ConsoleApplication {
  public class RCLDotnetServer {
    public static void Main (string[] args) {
      if (RCLdotnet.Init () != RCLRet.Ok)
      {
        Console.WriteLine("Unable to initialize RCL");
        return;
      }

      INode node = RCLdotnet.CreateNode ("server");

      IService<std_srvs.srv.SetBool_Request, std_srvs.srv.SetBool_Response> boolServer = 
        node.CreateService<std_srvs.srv.SetBool, std_srvs.srv.SetBool_Request, std_srvs.srv.SetBool_Response> (
          "bool_server", SetBool, QosProfile.Profile.Default);

      RCLdotnet.Spin (node);

      RCLdotnet.Shutdown ();
    }

    public static std_srvs.srv.SetBool_Response SetBool(std_srvs.srv.SetBool_Request request)
    {
      Console.WriteLine($"Setting value {request.Data}");
      //int sum = request.a + request.b;
      return new std_srvs.srv.SetBool_Response() { Success = request.Data, Message = "read you loud and clear" };
    }
  }
}
