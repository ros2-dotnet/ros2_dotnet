using System;

using ROS2;

namespace ConsoleApplication
{
    public class RCLDotnetListener
    {
        public static void Main(string[] args)
        {
            RCLdotnet.Init();

            Node node = RCLdotnet.CreateNode("listener");

            Subscription<std_msgs.msg.String> chatter_sub = node.CreateSubscription<std_msgs.msg.String>(
                "chatter", msg => Console.WriteLine("I heard: [" + msg.Data + "]"));

            RCLdotnet.Spin(node);
        }
    }
}
