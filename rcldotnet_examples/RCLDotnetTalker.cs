using System;
using ROS2;

namespace ConsoleApplication
{
    public class RCLDotnetTalker
    {
        private readonly Node _node;
        private readonly Publisher<std_msgs.msg.String> _chatterPub;
        private readonly Timer _timer;

        private int _i = 0;
        std_msgs.msg.String _msg = new();

        private RCLDotnetTalker()
        {
            RCLdotnet.Init();
            _node = RCLdotnet.CreateNode("talker");

            _chatterPub = _node.CreatePublisher<std_msgs.msg.String>("chatter");

            _timer = _node.CreateTimer(new Duration(1.0), PublishChatter);
        }

        private void PublishChatter(Duration elapsed)
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
