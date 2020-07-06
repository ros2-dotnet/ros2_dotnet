using System;
using System.Reflection;
using System.Runtime;
using System.Runtime.InteropServices;
using System.Threading;

using ROS2;
using ROS2.Utils;

namespace ConsoleApplication {
  public class RCLDotnetTalker {
    public static void Main (string[] args) {
      RCLdotnet.Init ();

      INode node = RCLdotnet.CreateNode ("talker");

      IPublisher<std_msgs.msg.String> chatter_pub = node.CreatePublisher<std_msgs.msg.String> ("chatter", QosProfile.Profile.Default);

      std_msgs.msg.String msg = new std_msgs.msg.String ();

      int i = 1;

      while (RCLdotnet.Ok ()) {
        msg.Data = "Hello World: " + i;
        i++;
        Console.WriteLine ("Publishing: \"" + msg.Data + "\"");
        chatter_pub.Publish (msg);

        // Sleep a little bit between each message
        Thread.Sleep (1000);
      }
    }
  }
}
