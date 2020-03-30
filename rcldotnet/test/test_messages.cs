using System;
using System.Reflection;
using System.Runtime;
using System.Runtime.InteropServices;
using System.Threading;

using ROS2;
using ROS2.Utils;

using Xunit;

namespace RCLdotnetTests
{
  public class TestMessages
  {
    [Fact]
    public void TestPublishString()
    {
      RCLdotnet.Init ();
      INode node_string_1 = RCLdotnet.CreateNode ("test_string_1");
      INode node_string_2 = RCLdotnet.CreateNode ("test_string_2");
      IPublisher<std_msgs.msg.String> chatter_pub = node_string_1.CreatePublisher<std_msgs.msg.String> ("topic_string");

      std_msgs.msg.String msg = new std_msgs.msg.String ();
      std_msgs.msg.String msg2 = new std_msgs.msg.String ();
      msg.Data = "Hello";

      bool received=false;
      ISubscription<std_msgs.msg.String> chatter_sub = node_string_1.CreateSubscription<std_msgs.msg.String> (
        "topic_string", rcv_msg =>
        {
          received=true;
          msg2 = rcv_msg;
        }
        );

      while(!received)
      {
        chatter_pub.Publish (msg);

        RCLdotnet.SpinOnce(node_string_1, 500);
        RCLdotnet.SpinOnce(node_string_1, 500);
      }
      Assert.Equal("Hello", msg2.Data);
    }

    [Fact]
    public void TestPublishBuiltins()
    {
      RCLdotnet.Init ();
      INode node_builtins_1 = RCLdotnet.CreateNode ("test_builtins_1");
      INode node_builtins_2 = RCLdotnet.CreateNode ("test_builtins_2");
      IPublisher<test_msgs.msg.Builtins> chatter_pub = node_builtins_1.CreatePublisher<test_msgs.msg.Builtins> ("topic_builtins");

      test_msgs.msg.Builtins msg = new test_msgs.msg.Builtins ();
      test_msgs.msg.Builtins msg2 = new test_msgs.msg.Builtins ();
      msg.Duration_value.Sec = 1;
      msg.Duration_value.Nanosec = 2u;
      msg.Time_value.Sec = 3;
      msg.Time_value.Nanosec = 4u;

      bool received=false;
      ISubscription<test_msgs.msg.Builtins> chatter_sub = node_builtins_1.CreateSubscription<test_msgs.msg.Builtins> (
        "topic_builtins", rcv_msg =>
        {
          received=true;
          msg2 = rcv_msg;
        }
        );

      while(!received)
      {
        chatter_pub.Publish (msg);

        RCLdotnet.SpinOnce(node_builtins_1, 500);
        RCLdotnet.SpinOnce(node_builtins_1, 500);
      }

      Assert.Equal(1, msg2.Duration_value.Sec);
      Assert.Equal(2u, msg2.Duration_value.Nanosec);
      Assert.Equal(3, msg2.Time_value.Sec);
      Assert.Equal(4u, msg2.Time_value.Nanosec);
    }
  }
}