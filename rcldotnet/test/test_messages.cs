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
        public void TestPublishNested()
        {
            RCLdotnet.Init ();
            INode node_nested_1 = RCLdotnet.CreateNode ("test_nested_1");
            INode node_nested_2 = RCLdotnet.CreateNode ("test_nested_2");
            IPublisher<test_msgs.msg.Nested> chatter_pub = node_nested_1.CreatePublisher<test_msgs.msg.Nested> ("topic_nested");

            test_msgs.msg.Nested msg = new test_msgs.msg.Nested ();
            test_msgs.msg.Nested msg2 = new test_msgs.msg.Nested ();
            msg.Primitive_values.Bool_value = true;
            msg.Primitive_values.Byte_value = 1;
            msg.Primitive_values.Char_value = 'b';
            msg.Primitive_values.Float32_value = 3.0F;
            msg.Primitive_values.Float64_value = 4.0;
            msg.Primitive_values.Int8_value  = 5;
            msg.Primitive_values.Uint8_value = 6;
            msg.Primitive_values.Int16_value  = 7;
            msg.Primitive_values.Uint16_value = 8;
            msg.Primitive_values.Int32_value = 9;
            msg.Primitive_values.Uint32_value  = 10;
            msg.Primitive_values.Int64_value  = 11;
            msg.Primitive_values.Uint64_value = 12;
            msg.Primitive_values.String_value = "thirteen";


            bool received=false;
            ISubscription<test_msgs.msg.Nested> chatter_sub = node_nested_1.CreateSubscription<test_msgs.msg.Nested> (
              "topic_nested", rcv_msg =>
                {
                  received=true;
                  msg2 = rcv_msg;
                }
              );

            while(!received)
            {
              chatter_pub.Publish (msg);

              RCLdotnet.SpinOnce(node_nested_1, 500);
              RCLdotnet.SpinOnce(node_nested_1, 500);
            }

            Assert.Equal(true, msg2.Primitive_values.Bool_value);
            Assert.Equal(1, msg2.Primitive_values.Byte_value);
            Assert.Equal('b', msg2.Primitive_values.Char_value);
            Assert.Equal(3.0F, msg2.Primitive_values.Float32_value);
            Assert.Equal(4.0, msg2.Primitive_values.Float64_value);
            Assert.Equal(5, msg2.Primitive_values.Int8_value);
            Assert.Equal(6, msg2.Primitive_values.Uint8_value);
            Assert.Equal(7, msg2.Primitive_values.Int16_value);
            Assert.Equal(8, msg2.Primitive_values.Uint16_value);
            Assert.Equal(9, msg2.Primitive_values.Int32_value);
            Assert.Equal(10u, msg2.Primitive_values.Uint32_value);
            Assert.Equal(11, msg2.Primitive_values.Int64_value);
            Assert.Equal(12u, msg2.Primitive_values.Uint64_value);
            Assert.Equal("thirteen", msg2.Primitive_values.String_value);
        }

    }
}
