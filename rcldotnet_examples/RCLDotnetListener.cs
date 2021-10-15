using System;
using System.Reflection;
using System.Runtime;
using System.Runtime.InteropServices;
using System.Threading;

using ROS2;
using ROS2.Common;
using ROS2.Utils;

namespace ConsoleApplication {
  public class RCLDotnetListener {
    public static void Main (string[] args) {
      if (RCLdotnet.Init () != RCLRet.Ok)
      {
        Console.WriteLine("Unable to initialize RCL");
        return;
      }

      INode node = RCLdotnet.CreateNode ("listener");

      ISubscription<std_msgs.msg.String> chatter_sub = node.CreateSubscription<std_msgs.msg.String> (
        "chatter", msg => Console.WriteLine ("I heard: [" + msg.Data + "]"), QosProfile.Profile.Default);

      RCLdotnet.Spin (node);

      RCLdotnet.Shutdown ();
    }
  }
}
