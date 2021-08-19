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
      ISubscription<std_msgs.msg.String> chatter_sub = node_string_2.CreateSubscription<std_msgs.msg.String> (
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
        RCLdotnet.SpinOnce(node_string_2, 500);
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
      ISubscription<test_msgs.msg.Builtins> chatter_sub = node_builtins_2.CreateSubscription<test_msgs.msg.Builtins> (
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
        RCLdotnet.SpinOnce(node_builtins_2, 500);
      }

      Assert.Equal(1, msg2.Duration_value.Sec);
      Assert.Equal(2u, msg2.Duration_value.Nanosec);
      Assert.Equal(3, msg2.Time_value.Sec);
      Assert.Equal(4u, msg2.Time_value.Nanosec);
    }

    [Fact]
    public void TestPublishArrays()
    {
      RCLdotnet.Init ();
      INode node_array_1 = RCLdotnet.CreateNode ("test_string_1");
      INode node_array_2 = RCLdotnet.CreateNode ("test_string_2");
      IPublisher<test_msgs.msg.Arrays> chatter_pub = node_array_1.CreatePublisher<test_msgs.msg.Arrays> ("topic_array");

      test_msgs.msg.Arrays msg = new test_msgs.msg.Arrays ();
      test_msgs.msg.Arrays msg2 = new test_msgs.msg.Arrays ();

      // bool_values
      msg.Bool_values.Add(true);
      msg.Bool_values.Add(false);
      msg.Bool_values.Add(true);

      // byte_values
      msg.Byte_values.Add(0);
      msg.Byte_values.Add(1);
      msg.Byte_values.Add(2);

      // char_values
      msg.Char_values.Add(3);
      msg.Char_values.Add(4);
      msg.Char_values.Add(5);

      // float32_values
      msg.Float32_values.Add(6.1f);
      msg.Float32_values.Add(7.1f);
      msg.Float32_values.Add(8.1f);

      // float64_values
      msg.Float64_values.Add(9.1);
      msg.Float64_values.Add(10.1);
      msg.Float64_values.Add(11.1);

      // int8_values
      msg.Int8_values.Add(12);
      msg.Int8_values.Add(13);
      msg.Int8_values.Add(14);

      // uint8_values
      msg.Uint8_values.Add(15);
      msg.Uint8_values.Add(16);
      msg.Uint8_values.Add(17);

      // int16_values
      msg.Int16_values.Add(18);
      msg.Int16_values.Add(19);
      msg.Int16_values.Add(20);

      // uint16_values
      msg.Uint16_values.Add(21);
      msg.Uint16_values.Add(22);
      msg.Uint16_values.Add(23);

      // int32_values
      msg.Int32_values.Add(24);
      msg.Int32_values.Add(25);
      msg.Int32_values.Add(26);

      // uint32_values
      msg.Uint32_values.Add(27);
      msg.Uint32_values.Add(28);
      msg.Uint32_values.Add(29);

      // int64_values
      msg.Int64_values.Add(30);
      msg.Int64_values.Add(31);
      msg.Int64_values.Add(32);

      // uint64_values
      msg.Uint64_values.Add(33);
      msg.Uint64_values.Add(34);
      msg.Uint64_values.Add(35);

      // string_values
      msg.String_values.Add("one");
      msg.String_values.Add("two");
      msg.String_values.Add("three");

      test_msgs.msg.BasicTypes basic_type_1 = new test_msgs.msg.BasicTypes();
      basic_type_1.Bool_value = true;
      basic_type_1.Byte_value = 36;
      basic_type_1.Char_value = 37;
      basic_type_1.Float32_value = 38.1f;
      basic_type_1.Float64_value = 39.1;
      basic_type_1.Int8_value = 40;
      basic_type_1.Uint8_value = 41;
      basic_type_1.Int16_value = 42;
      basic_type_1.Uint16_value = 43;
      basic_type_1.Int32_value = 44;
      basic_type_1.Uint32_value = 45;
      basic_type_1.Int64_value = 46;
      basic_type_1.Uint64_value = 47;

      test_msgs.msg.BasicTypes basic_type_2 = new test_msgs.msg.BasicTypes();
      basic_type_2.Bool_value = false;
      basic_type_2.Byte_value = 48;
      basic_type_2.Char_value = 49;
      basic_type_2.Float32_value = 50.1f;
      basic_type_2.Float64_value = 51.1;
      basic_type_2.Int8_value = 52;
      basic_type_2.Uint8_value = 53;
      basic_type_2.Int16_value = 54;
      basic_type_2.Uint16_value = 55;
      basic_type_2.Int32_value = 56;
      basic_type_2.Uint32_value = 57;
      basic_type_2.Int64_value = 58;
      basic_type_2.Uint64_value = 59;

      test_msgs.msg.BasicTypes basic_type_3 = new test_msgs.msg.BasicTypes();
      basic_type_3.Bool_value = true;
      basic_type_3.Byte_value = 60;
      basic_type_3.Char_value = 61;
      basic_type_3.Float32_value = 62.1f;
      basic_type_3.Float64_value = 63.1;
      basic_type_3.Int8_value = 64;
      basic_type_3.Uint8_value = 65;
      basic_type_3.Int16_value = 66;
      basic_type_3.Uint16_value = 67;
      basic_type_3.Int32_value = 68;
      basic_type_3.Uint32_value = 69;
      basic_type_3.Int64_value = 70;
      basic_type_3.Uint64_value = 71;

      msg.Basic_types_values.Add(basic_type_1);
      msg.Basic_types_values.Add(basic_type_2);
      msg.Basic_types_values.Add(basic_type_3);

      bool received=false;
      ISubscription<test_msgs.msg.Arrays> chatter_sub = node_array_2.CreateSubscription<test_msgs.msg.Arrays> (
        "topic_array", rcv_msg =>
        {
          received=true;
          msg2 = rcv_msg;
        }
      );

      while(!received)
      {
        chatter_pub.Publish (msg);

        RCLdotnet.SpinOnce(node_array_1, 500);
        RCLdotnet.SpinOnce(node_array_2, 500);
      }

      // bool_values
      Assert.True(msg2.Bool_values[0]);
      Assert.False(msg2.Bool_values[1]);
      Assert.True(msg2.Bool_values[2]);

      // byte_values
      Assert.Equal(0, msg2.Byte_values[0]);
      Assert.Equal(1, msg2.Byte_values[1]);
      Assert.Equal(2, msg2.Byte_values[2]);

      // char_values
      Assert.Equal(3, msg2.Char_values[0]);
      Assert.Equal(4, msg2.Char_values[1]);
      Assert.Equal(5, msg2.Char_values[2]);

      // float32_values
      Assert.Equal(6.1f, msg2.Float32_values[0]);
      Assert.Equal(7.1f, msg2.Float32_values[1]);
      Assert.Equal(8.1f, msg2.Float32_values[2]);

      // float64_values
      Assert.Equal(9.1, msg2.Float64_values[0]);
      Assert.Equal(10.1, msg2.Float64_values[1]);
      Assert.Equal(11.1, msg2.Float64_values[2]);

      // int8_values
      Assert.Equal(12, msg2.Int8_values[0]);
      Assert.Equal(13, msg2.Int8_values[1]);
      Assert.Equal(14, msg2.Int8_values[2]);

      // uint8_values
      Assert.Equal(15, msg2.Uint8_values[0]);
      Assert.Equal(16, msg2.Uint8_values[1]);
      Assert.Equal(17, msg2.Uint8_values[2]);

      // int16_values
      Assert.Equal(18, msg2.Int16_values[0]);
      Assert.Equal(19, msg2.Int16_values[1]);
      Assert.Equal(20, msg2.Int16_values[2]);

      // uint16_values
      Assert.Equal(21, msg2.Uint16_values[0]);
      Assert.Equal(22, msg2.Uint16_values[1]);
      Assert.Equal(23, msg2.Uint16_values[2]);

      // int32_values
      Assert.Equal(24, msg2.Int32_values[0]);
      Assert.Equal(25, msg2.Int32_values[1]);
      Assert.Equal(26, msg2.Int32_values[2]);
      
      // uint32_values
      Assert.Equal((uint)27, msg2.Uint32_values[0]);
      Assert.Equal((uint)28, msg2.Uint32_values[1]);
      Assert.Equal((uint)29, msg2.Uint32_values[2]);

      // int64_values
      Assert.Equal(30, msg2.Int64_values[0]);
      Assert.Equal(31, msg2.Int64_values[1]);
      Assert.Equal(32, msg2.Int64_values[2]);
      
      // uint64_values
      Assert.Equal((ulong)33, msg2.Uint64_values[0]);
      Assert.Equal((ulong)34, msg2.Uint64_values[1]);
      Assert.Equal((ulong)35, msg2.Uint64_values[2]);

      // string_values
      Assert.Equal("one", msg2.String_values[0]);
      Assert.Equal("two", msg2.String_values[1]);
      Assert.Equal("three", msg2.String_values[2]);
      
      Assert.Equal(3, msg2.Basic_types_values.Count);
      Assert.True(msg2.Basic_types_values[0].Bool_value);
      Assert.Equal(36, msg2.Basic_types_values[0].Byte_value);
      Assert.Equal(37, msg2.Basic_types_values[0].Char_value);
      Assert.Equal(38.1f, msg2.Basic_types_values[0].Float32_value);
      Assert.Equal(39.1, msg2.Basic_types_values[0].Float64_value);
      Assert.Equal(40, msg2.Basic_types_values[0].Int8_value);
      Assert.Equal(41, msg2.Basic_types_values[0].Uint8_value);
      Assert.Equal(42, msg2.Basic_types_values[0].Int16_value);
      Assert.Equal(43, msg2.Basic_types_values[0].Uint16_value);
      Assert.Equal(44, msg2.Basic_types_values[0].Int32_value);
      Assert.Equal((uint)45, msg2.Basic_types_values[0].Uint32_value);
      Assert.Equal(46, msg2.Basic_types_values[0].Int64_value);
      Assert.Equal((ulong)47, msg2.Basic_types_values[0].Uint64_value);

      Assert.False(msg2.Basic_types_values[1].Bool_value);
      Assert.Equal(48, msg2.Basic_types_values[1].Byte_value);
      Assert.Equal(49, msg2.Basic_types_values[1].Char_value);
      Assert.Equal(50.1f, msg2.Basic_types_values[1].Float32_value);
      Assert.Equal(51.1, msg2.Basic_types_values[1].Float64_value);
      Assert.Equal(52, msg2.Basic_types_values[1].Int8_value);
      Assert.Equal(53, msg2.Basic_types_values[1].Uint8_value);
      Assert.Equal(54, msg2.Basic_types_values[1].Int16_value);
      Assert.Equal(55, msg2.Basic_types_values[1].Uint16_value);
      Assert.Equal(56, msg2.Basic_types_values[1].Int32_value);
      Assert.Equal((uint)57, msg2.Basic_types_values[1].Uint32_value);
      Assert.Equal(58, msg2.Basic_types_values[1].Int64_value);
      Assert.Equal((ulong)59, msg2.Basic_types_values[1].Uint64_value);
 
      Assert.True(msg2.Basic_types_values[2].Bool_value);
      Assert.Equal(60, msg2.Basic_types_values[2].Byte_value);
      Assert.Equal(61, msg2.Basic_types_values[2].Char_value);
      Assert.Equal(62.1f, msg2.Basic_types_values[2].Float32_value);
      Assert.Equal(63.1, msg2.Basic_types_values[2].Float64_value);
      Assert.Equal(64, msg2.Basic_types_values[2].Int8_value);
      Assert.Equal(65, msg2.Basic_types_values[2].Uint8_value);
      Assert.Equal(66, msg2.Basic_types_values[2].Int16_value);
      Assert.Equal(67, msg2.Basic_types_values[2].Uint16_value);
      Assert.Equal(68, msg2.Basic_types_values[2].Int32_value);
      Assert.Equal((uint)69, msg2.Basic_types_values[2].Uint32_value);
      Assert.Equal(70, msg2.Basic_types_values[2].Int64_value);
      Assert.Equal((ulong)71, msg2.Basic_types_values[2].Uint64_value);
    }

  }
}