using System;
using System.Threading;

using ROS2;

namespace ConsoleApplication
{
    public class RCLDotnetTalker
    {
        public static void Main(string[] args)
        {
            RCLdotnet.Init();

            Node node = RCLdotnet.CreateNode("talker");

            Publisher<std_msgs.msg.String> chatterPub = node.CreatePublisher<std_msgs.msg.String>("chatter");

            std_msgs.msg.String msg = new std_msgs.msg.String();

            int i = 1;

            while (RCLdotnet.Ok())
            {
                msg.Data = "Hello World: " + i;
                i++;
                Console.WriteLine("Publishing: \"" + msg.Data + "\"");
                chatterPub.Publish(msg);

                // Sleep a little bit between each message
                Thread.Sleep(1000);
            }
        }
    }
}
