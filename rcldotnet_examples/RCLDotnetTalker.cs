using System;
using System.Threading;

using rclcs;
using ROS2.Utils;

namespace ConsoleApplication
{
    public class RCLDotnetTalker
    {
        public static void Main(string[] args)
        {
            Context ctx = new Context();
            Rclcs.Init(ctx);
            INode node = Rclcs.CreateNode("talker", ctx);
            IPublisher<std_msgs.msg.String> chatter_pub = node.CreatePublisher<std_msgs.msg.String>("chatter");

            std_msgs.msg.String msg = new std_msgs.msg.String();

            int i = 1;

            while (Rclcs.Ok(ctx))
            {
                msg.data = "Hello World: " + i;
                i++;
                Console.WriteLine("Publishing: \"" + msg.data + "\"");
                chatter_pub.Publish(msg);

                // Sleep a little bit between each message
                Thread.Sleep(1000);
            }
            Rclcs.Shutdown(ctx);
        }
    }
}
