using System;
using rclcs;

namespace ConsoleApplication
{
    public class RCLDotnetListener
    {
        public static void Main(string[] args)
        {
            Context ctx = new Context();
            Rclcs.Init(ctx);
            INode node = Rclcs.CreateNode("listener", ctx);

            ISubscription<std_msgs.msg.String> chatter_sub = node.CreateSubscription<std_msgs.msg.String>(
              "chatter", msg => Console.WriteLine("I heard: [" + msg.data + "]"));
                
            Rclcs.Spin(node, ctx);
            Rclcs.Shutdown(ctx);
        }
    }
}
