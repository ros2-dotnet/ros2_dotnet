using System;
using System.Threading;

using ROS2;

namespace ConsoleApplication
{
    public class RCLDotnetTalker
    {
        private readonly Node _node;
        private readonly Publisher<std_msgs.msg.String> _chatterPub;

        private int _i = 0;
        private readonly std_msgs.msg.String _msg = new();

        private RCLDotnetTalker()
        {
            RCLdotnet.Init();
            _node = RCLdotnet.CreateNode("talker");

            _chatterPub = _node.CreatePublisher<std_msgs.msg.String>("chatter");

            _node.CreateTimer(TimeSpan.FromSeconds(1.0), PublishChatter);
        }

        private void PublishChatter(TimeSpan elapsed)
        {
            _msg.Data = $"Hello World: {_i}";
            Console.WriteLine($"Publishing: \"{_msg.Data}\"");
            _chatterPub.Publish(_msg);

            _i++;
        }

        private void Spin() => RCLdotnet.Spin(_node);

        public static void Main(string[] args)
        {
            RCLDotnetTalker talker = new RCLDotnetTalker();
            talker.Spin();
        }
    }
}
