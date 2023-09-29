using System;
using ROS2;

namespace ConsoleApplication
{
    public class RCLDotnetTalker
    {
        private readonly Node node;
        private readonly Publisher<std_msgs.msg.String> chatterPub;

        private int i = 0;
        std_msgs.msg.String msg = new();

        private RCLDotnetTalker()
        {
            RCLdotnet.Init();
            node = RCLdotnet.CreateNode("talker");

            chatterPub = node.CreatePublisher<std_msgs.msg.String>("chatter");

            ROS2.Timer timer = node.CreateTimer(new Duration(1.0), PublishChatter);
        }

        private void PublishChatter()
        {
            msg.Data = $"Hello World: {i}";
            Console.WriteLine($"Publishing: \"{msg.Data}\"");
            chatterPub.Publish(msg);

            i++;
        }

        private void Spin() => RCLdotnet.Spin(node);

        public static void Main(string[] args)
        {
            RCLDotnetTalker talker = new RCLDotnetTalker();
            talker.Spin();
        }
    }
}
