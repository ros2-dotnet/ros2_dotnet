using System;
using System.Reflection;
using System.Runtime;
using System.Runtime.InteropServices;
using System.Threading.Tasks;

using ROS2;
using ROS2.Common;
using ROS2.Utils;

namespace ConsoleApplication {
  public class RCLDotnetClient {
    public static async Task Main (string[] args) {
      while (!System.Diagnostics.Debugger.IsAttached) {
        System.Threading.Thread.Sleep(100);
      }

      if (RCLdotnet.Init () != RCLRet.Ok)
      {
        Console.WriteLine("Unable to initialize RCL");
        return;
      }

      INode node = RCLdotnet.CreateNode ("client");

      IClient<std_srvs.srv.SetBool_Request, std_srvs.srv.SetBool_Response> client = 
        node.CreateClient<std_srvs.srv.SetBool, std_srvs.srv.SetBool_Request, std_srvs.srv.SetBool_Response> (
          "bool_server", QosProfile.Profile.Default);

      var request = new std_srvs.srv.SetBool_Request() { Data = true };

      while (!await client.WaitForServiceAsync(1))
      {
        if (!ROS2.RCLdotnet.Ok())
        {
          Console.WriteLine("Interrupted while waiting for the service. Exiting.");
          return;
        }

        Console.WriteLine("Service not available, waiting again...");
      }

      try 
      {
        var response = await client.SendRequestAsync(request);
        Console.WriteLine($"Server ==> {response.Message}");
      }
      catch(RCLException exc)
      {
        Console.WriteLine($"Request failed: {exc.Message}");
      }

      RCLdotnet.Shutdown ();
    }
  }
}
