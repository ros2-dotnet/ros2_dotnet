using System;
using System.Reflection;
using System.Runtime;
using System.Runtime.InteropServices;
using System.Threading;

using ROS2;

namespace ConsoleApplication {
  public class RCLDotnetListener {
    public static void Main (string[] args) {
      RCLdotnet.Init ();

      INode node = RCLdotnet.CreateNode ("listener");

      ISubscription<std_msgs.msg.String> chatter_sub = node.CreateSubscription<std_msgs.msg.String> (
        "chatter", msg => Console.WriteLine ("I heard: [" + msg.Data + "]"));

      RCLdotnet.Spin (node);
    }
  }
}
