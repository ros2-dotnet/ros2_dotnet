using System;
using System.Collections.Generic;
using ROS2;
using Xunit;

namespace RCLdotnetTests
{
  public class TestMessages
  {
    [Fact]
    public void TestPublishString()
    {
      RCLdotnet.Init ();
      Node node_string_1 = RCLdotnet.CreateNode ("test_string_1");
      Node node_string_2 = RCLdotnet.CreateNode ("test_string_2");
      Publisher<std_msgs.msg.String> chatter_pub = node_string_1.CreatePublisher<std_msgs.msg.String> ("topic_string");

      std_msgs.msg.String msg = new std_msgs.msg.String ();
      std_msgs.msg.String msg2 = new std_msgs.msg.String ();
      msg.Data = "Hello";

      bool received=false;
      Subscription<std_msgs.msg.String> chatter_sub = node_string_2.CreateSubscription<std_msgs.msg.String> (
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
      Node node_builtins_1 = RCLdotnet.CreateNode ("test_builtins_1");
      Node node_builtins_2 = RCLdotnet.CreateNode ("test_builtins_2");
      Publisher<test_msgs.msg.Builtins> chatter_pub = node_builtins_1.CreatePublisher<test_msgs.msg.Builtins> ("topic_builtins");

      test_msgs.msg.Builtins msg = new test_msgs.msg.Builtins ();
      test_msgs.msg.Builtins msg2 = new test_msgs.msg.Builtins ();
      msg.DurationValue.Sec = 1;
      msg.DurationValue.Nanosec = 2u;
      msg.TimeValue.Sec = 3;
      msg.TimeValue.Nanosec = 4u;

      bool received=false;
      Subscription<test_msgs.msg.Builtins> chatter_sub = node_builtins_2.CreateSubscription<test_msgs.msg.Builtins> (
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

      Assert.Equal(1, msg2.DurationValue.Sec);
      Assert.Equal(2u, msg2.DurationValue.Nanosec);
      Assert.Equal(3, msg2.TimeValue.Sec);
      Assert.Equal(4u, msg2.TimeValue.Nanosec);
    }

    [Fact]
    public void TestPublishArrays()
    {
      RCLdotnet.Init ();
      Node node_array_1 = RCLdotnet.CreateNode ("test_arrays_1");
      Node node_array_2 = RCLdotnet.CreateNode ("test_arrays_2");
      Publisher<test_msgs.msg.Arrays> chatter_pub = node_array_1.CreatePublisher<test_msgs.msg.Arrays> ("topic_array");

      test_msgs.msg.Arrays msg = new test_msgs.msg.Arrays ();
      test_msgs.msg.Arrays msg2 = new test_msgs.msg.Arrays ();

      // boolValues
      msg.BoolValues[0] = true;
      msg.BoolValues[1] = false;
      msg.BoolValues[2] = true;

      // byteValues
      msg.ByteValues[0] = 0;
      msg.ByteValues[1] = 1;
      msg.ByteValues[2] = 2;

      // charValues
      msg.CharValues[0] = 3;
      msg.CharValues[1] = 4;
      msg.CharValues[2] = 5;

      // float32Values
      msg.Float32Values[0] = 6.1f;
      msg.Float32Values[1] = 7.1f;
      msg.Float32Values[2] = 8.1f;

      // float64Values
      msg.Float64Values[0] = 9.1;
      msg.Float64Values[1] = 10.1;
      msg.Float64Values[2] = 11.1;

      // int8Values
      msg.Int8Values[0] = 12;
      msg.Int8Values[1] = 13;
      msg.Int8Values[2] = 14;

      // uint8Values
      msg.Uint8Values[0] = 15;
      msg.Uint8Values[1] = 16;
      msg.Uint8Values[2] = 17;

      // int16Values
      msg.Int16Values[0] = 18;
      msg.Int16Values[1] = 19;
      msg.Int16Values[2] = 20;

      // uint16Values
      msg.Uint16Values[0] = 21;
      msg.Uint16Values[1] = 22;
      msg.Uint16Values[2] = 23;

      // int32Values
      msg.Int32Values[0] = 24;
      msg.Int32Values[1] = 25;
      msg.Int32Values[2] = 26;

      // uint32Values
      msg.Uint32Values[0] = 27;
      msg.Uint32Values[1] = 28;
      msg.Uint32Values[2] = 29;

      // int64Values
      msg.Int64Values[0] = 30;
      msg.Int64Values[1] = 31;
      msg.Int64Values[2] = 32;

      // uint64Values
      msg.Uint64Values[0] = 33;
      msg.Uint64Values[1] = 34;
      msg.Uint64Values[2] = 35;

      // stringValues
      msg.StringValues[0] = "one";
      msg.StringValues[1] = "two";
      msg.StringValues[2] = "three";

      // basicTypesValues
      test_msgs.msg.BasicTypes basic_type_1 = new test_msgs.msg.BasicTypes();
      basic_type_1.BoolValue = true;
      basic_type_1.ByteValue = 36;
      basic_type_1.CharValue = 37;
      basic_type_1.Float32Value = 38.1f;
      basic_type_1.Float64Value = 39.1;
      basic_type_1.Int8Value = 40;
      basic_type_1.Uint8Value = 41;
      basic_type_1.Int16Value = 42;
      basic_type_1.Uint16Value = 43;
      basic_type_1.Int32Value = 44;
      basic_type_1.Uint32Value = 45;
      basic_type_1.Int64Value = 46;
      basic_type_1.Uint64Value = 47;

      test_msgs.msg.BasicTypes basic_type_2 = new test_msgs.msg.BasicTypes();
      basic_type_2.BoolValue = false;
      basic_type_2.ByteValue = 48;
      basic_type_2.CharValue = 49;
      basic_type_2.Float32Value = 50.1f;
      basic_type_2.Float64Value = 51.1;
      basic_type_2.Int8Value = 52;
      basic_type_2.Uint8Value = 53;
      basic_type_2.Int16Value = 54;
      basic_type_2.Uint16Value = 55;
      basic_type_2.Int32Value = 56;
      basic_type_2.Uint32Value = 57;
      basic_type_2.Int64Value = 58;
      basic_type_2.Uint64Value = 59;

      test_msgs.msg.BasicTypes basic_type_3 = new test_msgs.msg.BasicTypes();
      basic_type_3.BoolValue = true;
      basic_type_3.ByteValue = 60;
      basic_type_3.CharValue = 61;
      basic_type_3.Float32Value = 62.1f;
      basic_type_3.Float64Value = 63.1;
      basic_type_3.Int8Value = 64;
      basic_type_3.Uint8Value = 65;
      basic_type_3.Int16Value = 66;
      basic_type_3.Uint16Value = 67;
      basic_type_3.Int32Value = 68;
      basic_type_3.Uint32Value = 69;
      basic_type_3.Int64Value = 70;
      basic_type_3.Uint64Value = 71;

      msg.BasicTypesValues[0] = basic_type_1;
      msg.BasicTypesValues[1] = basic_type_2;
      msg.BasicTypesValues[2] = basic_type_3;

      bool received=false;
      Subscription<test_msgs.msg.Arrays> chatter_sub = node_array_2.CreateSubscription<test_msgs.msg.Arrays> (
        "topic_array", rcv_msg =>
        {
            received = true;
            msg2 = rcv_msg;
        }
      );

      while (!received)
      {
          chatter_pub.Publish(msg);

          RCLdotnet.SpinOnce(node_array_1, 500);
          RCLdotnet.SpinOnce(node_array_2, 500);
      }

      // boolValues
      Assert.Equal(3, test_msgs.msg.Arrays.BoolValuesLength);
      Assert.Equal(3, msg2.BoolValues.Length);
      Assert.True(msg2.BoolValues[0]);
      Assert.False(msg2.BoolValues[1]);
      Assert.True(msg2.BoolValues[2]);

      // byteValues
      Assert.Equal(3, test_msgs.msg.Arrays.ByteValuesLength);
      Assert.Equal(3, msg2.ByteValues.Length);
      Assert.Equal(0, msg2.ByteValues[0]);
      Assert.Equal(1, msg2.ByteValues[1]);
      Assert.Equal(2, msg2.ByteValues[2]);

      // charValues
      Assert.Equal(3, test_msgs.msg.Arrays.CharValuesLength);
      Assert.Equal(3, msg2.CharValues.Length);
      Assert.Equal(3, msg2.CharValues[0]);
      Assert.Equal(4, msg2.CharValues[1]);
      Assert.Equal(5, msg2.CharValues[2]);

      // float32Values
      Assert.Equal(3, test_msgs.msg.Arrays.Float32ValuesLength);
      Assert.Equal(3, msg2.Float32Values.Length);
      Assert.Equal(6.1f, msg2.Float32Values[0]);
      Assert.Equal(7.1f, msg2.Float32Values[1]);
      Assert.Equal(8.1f, msg2.Float32Values[2]);

      // float64Values
      Assert.Equal(3, test_msgs.msg.Arrays.Float64ValuesLength);
      Assert.Equal(3, msg2.Float64Values.Length);
      Assert.Equal(9.1, msg2.Float64Values[0]);
      Assert.Equal(10.1, msg2.Float64Values[1]);
      Assert.Equal(11.1, msg2.Float64Values[2]);

      // int8Values
      Assert.Equal(3, test_msgs.msg.Arrays.Int8ValuesLength);
      Assert.Equal(3, msg2.Int8Values.Length);
      Assert.Equal(12, msg2.Int8Values[0]);
      Assert.Equal(13, msg2.Int8Values[1]);
      Assert.Equal(14, msg2.Int8Values[2]);

      // uint8Values
      Assert.Equal(3, test_msgs.msg.Arrays.Uint8ValuesLength);
      Assert.Equal(3, msg2.Uint8Values.Length);
      Assert.Equal(15, msg2.Uint8Values[0]);
      Assert.Equal(16, msg2.Uint8Values[1]);
      Assert.Equal(17, msg2.Uint8Values[2]);

      // int16Values
      Assert.Equal(3, test_msgs.msg.Arrays.Int16ValuesLength);
      Assert.Equal(3, msg2.Int16Values.Length);
      Assert.Equal(18, msg2.Int16Values[0]);
      Assert.Equal(19, msg2.Int16Values[1]);
      Assert.Equal(20, msg2.Int16Values[2]);

      // uint16Values
      Assert.Equal(3, test_msgs.msg.Arrays.Uint16ValuesLength);
      Assert.Equal(3, msg2.Uint16Values.Length);
      Assert.Equal(21, msg2.Uint16Values[0]);
      Assert.Equal(22, msg2.Uint16Values[1]);
      Assert.Equal(23, msg2.Uint16Values[2]);

      // int32Values
      Assert.Equal(3, test_msgs.msg.Arrays.Int32ValuesLength);
      Assert.Equal(3, msg2.Int32Values.Length);
      Assert.Equal(24, msg2.Int32Values[0]);
      Assert.Equal(25, msg2.Int32Values[1]);
      Assert.Equal(26, msg2.Int32Values[2]);

      // uint32Values
      Assert.Equal(3, test_msgs.msg.Arrays.Uint32ValuesLength);
      Assert.Equal(3, msg2.Uint32Values.Length);
      Assert.Equal((uint)27, msg2.Uint32Values[0]);
      Assert.Equal((uint)28, msg2.Uint32Values[1]);
      Assert.Equal((uint)29, msg2.Uint32Values[2]);

      // int64Values
      Assert.Equal(3, test_msgs.msg.Arrays.Int64ValuesLength);
      Assert.Equal(3, msg2.Int64Values.Length);
      Assert.Equal(30, msg2.Int64Values[0]);
      Assert.Equal(31, msg2.Int64Values[1]);
      Assert.Equal(32, msg2.Int64Values[2]);

      // uint64Values
      Assert.Equal(3, test_msgs.msg.Arrays.Uint64ValuesLength);
      Assert.Equal(3, msg2.Uint64Values.Length);
      Assert.Equal((ulong)33, msg2.Uint64Values[0]);
      Assert.Equal((ulong)34, msg2.Uint64Values[1]);
      Assert.Equal((ulong)35, msg2.Uint64Values[2]);

      // stringValues
      Assert.Equal(3, test_msgs.msg.Arrays.StringValuesLength);
      Assert.Equal(3, msg2.StringValues.Length);
      Assert.Equal("one", msg2.StringValues[0]);
      Assert.Equal("two", msg2.StringValues[1]);
      Assert.Equal("three", msg2.StringValues[2]);

      // basicTypesValues
      Assert.Equal(3, test_msgs.msg.Arrays.BasicTypesValuesLength);
      Assert.Equal(3, msg2.BasicTypesValues.Length);

      Assert.True(msg2.BasicTypesValues[0].BoolValue);
      Assert.Equal(36, msg2.BasicTypesValues[0].ByteValue);
      Assert.Equal(37, msg2.BasicTypesValues[0].CharValue);
      Assert.Equal(38.1f, msg2.BasicTypesValues[0].Float32Value);
      Assert.Equal(39.1, msg2.BasicTypesValues[0].Float64Value);
      Assert.Equal(40, msg2.BasicTypesValues[0].Int8Value);
      Assert.Equal(41, msg2.BasicTypesValues[0].Uint8Value);
      Assert.Equal(42, msg2.BasicTypesValues[0].Int16Value);
      Assert.Equal(43, msg2.BasicTypesValues[0].Uint16Value);
      Assert.Equal(44, msg2.BasicTypesValues[0].Int32Value);
      Assert.Equal((uint)45, msg2.BasicTypesValues[0].Uint32Value);
      Assert.Equal(46, msg2.BasicTypesValues[0].Int64Value);
      Assert.Equal((ulong)47, msg2.BasicTypesValues[0].Uint64Value);

      Assert.False(msg2.BasicTypesValues[1].BoolValue);
      Assert.Equal(48, msg2.BasicTypesValues[1].ByteValue);
      Assert.Equal(49, msg2.BasicTypesValues[1].CharValue);
      Assert.Equal(50.1f, msg2.BasicTypesValues[1].Float32Value);
      Assert.Equal(51.1, msg2.BasicTypesValues[1].Float64Value);
      Assert.Equal(52, msg2.BasicTypesValues[1].Int8Value);
      Assert.Equal(53, msg2.BasicTypesValues[1].Uint8Value);
      Assert.Equal(54, msg2.BasicTypesValues[1].Int16Value);
      Assert.Equal(55, msg2.BasicTypesValues[1].Uint16Value);
      Assert.Equal(56, msg2.BasicTypesValues[1].Int32Value);
      Assert.Equal((uint)57, msg2.BasicTypesValues[1].Uint32Value);
      Assert.Equal(58, msg2.BasicTypesValues[1].Int64Value);
      Assert.Equal((ulong)59, msg2.BasicTypesValues[1].Uint64Value);

      Assert.True(msg2.BasicTypesValues[2].BoolValue);
      Assert.Equal(60, msg2.BasicTypesValues[2].ByteValue);
      Assert.Equal(61, msg2.BasicTypesValues[2].CharValue);
      Assert.Equal(62.1f, msg2.BasicTypesValues[2].Float32Value);
      Assert.Equal(63.1, msg2.BasicTypesValues[2].Float64Value);
      Assert.Equal(64, msg2.BasicTypesValues[2].Int8Value);
      Assert.Equal(65, msg2.BasicTypesValues[2].Uint8Value);
      Assert.Equal(66, msg2.BasicTypesValues[2].Int16Value);
      Assert.Equal(67, msg2.BasicTypesValues[2].Uint16Value);
      Assert.Equal(68, msg2.BasicTypesValues[2].Int32Value);
      Assert.Equal((uint)69, msg2.BasicTypesValues[2].Uint32Value);
      Assert.Equal(70, msg2.BasicTypesValues[2].Int64Value);
      Assert.Equal((ulong)71, msg2.BasicTypesValues[2].Uint64Value);
    }

    [Fact]
    public void TestPublishArraysSizeCheckToLittle()
    {
      RCLdotnet.Init();
      var nodeArraysSizeCheck = RCLdotnet.CreateNode("test_arrays_size_check_to_little");
      var chatterPub = nodeArraysSizeCheck.CreatePublisher<test_msgs.msg.Arrays>("topic_array_size_check_to_little");
      
      var msg = new test_msgs.msg.Arrays();
      msg.BoolValues = new bool[]
      {
        false,
        true,
      };
      
      var exception = Assert.Throws<Exception>(() => chatterPub.Publish(msg));
      Assert.Equal("Invalid size of array 'BoolValues'.", exception.Message);
    }

    [Fact]
    public void TestPublishArraysSizeCheckToMuch()
    {
      RCLdotnet.Init();
      var nodeArraysSizeCheck = RCLdotnet.CreateNode("test_arrays_size_check_to_much");
      var chatterPub = nodeArraysSizeCheck.CreatePublisher<test_msgs.msg.Arrays>("topic_array_size_check_to_much");

      var msg = new test_msgs.msg.Arrays();
      msg.StringValues = new string[]
      {
        "0",
        "1",
        "2",
        "3",
      };
      
      var exception = Assert.Throws<Exception>(() => chatterPub.Publish(msg));
      Assert.Equal("Invalid size of array 'StringValues'.", exception.Message);
    }

    [Fact]
    public void TestPublishUnboundedSequences()
    {
      RCLdotnet.Init();
      Node node_array_1 = RCLdotnet.CreateNode("test_unbounded_sequences_1");
      Node node_array_2 = RCLdotnet.CreateNode("test_unbounded_sequences_2");
      Publisher<test_msgs.msg.UnboundedSequences> chatter_pub = node_array_1.CreatePublisher<test_msgs.msg.UnboundedSequences>("topic_unbounded_sequences");

      test_msgs.msg.UnboundedSequences msg = new test_msgs.msg.UnboundedSequences();
      test_msgs.msg.UnboundedSequences msg2 = new test_msgs.msg.UnboundedSequences();

      // boolValues
      msg.BoolValues.Add(true);
      msg.BoolValues.Add(false);
      msg.BoolValues.Add(true);

      // byteValues
      msg.ByteValues.Add(0);
      msg.ByteValues.Add(1);
      msg.ByteValues.Add(2);

      // charValues
      msg.CharValues.Add(3);
      msg.CharValues.Add(4);
      msg.CharValues.Add(5);

      // float32Values
      msg.Float32Values.Add(6.1f);
      msg.Float32Values.Add(7.1f);
      msg.Float32Values.Add(8.1f);

      // float64Values
      msg.Float64Values.Add(9.1);
      msg.Float64Values.Add(10.1);
      msg.Float64Values.Add(11.1);

      // int8Values
      msg.Int8Values.Add(12);
      msg.Int8Values.Add(13);
      msg.Int8Values.Add(14);

      // uint8Values
      msg.Uint8Values.Add(15);
      msg.Uint8Values.Add(16);
      msg.Uint8Values.Add(17);

      // int16Values
      msg.Int16Values.Add(18);
      msg.Int16Values.Add(19);
      msg.Int16Values.Add(20);

      // uint16Values
      msg.Uint16Values.Add(21);
      msg.Uint16Values.Add(22);
      msg.Uint16Values.Add(23);

      // int32Values
      msg.Int32Values.Add(24);
      msg.Int32Values.Add(25);
      msg.Int32Values.Add(26);

      // uint32Values
      msg.Uint32Values.Add(27);
      msg.Uint32Values.Add(28);
      msg.Uint32Values.Add(29);

      // int64Values
      msg.Int64Values.Add(30);
      msg.Int64Values.Add(31);
      msg.Int64Values.Add(32);

      // uint64Values
      msg.Uint64Values.Add(33);
      msg.Uint64Values.Add(34);
      msg.Uint64Values.Add(35);

      // stringValues
      msg.StringValues.Add("one");
      msg.StringValues.Add("two");
      msg.StringValues.Add("three");

      test_msgs.msg.BasicTypes basic_type_1 = new test_msgs.msg.BasicTypes();
      basic_type_1.BoolValue = true;
      basic_type_1.ByteValue = 36;
      basic_type_1.CharValue = 37;
      basic_type_1.Float32Value = 38.1f;
      basic_type_1.Float64Value = 39.1;
      basic_type_1.Int8Value = 40;
      basic_type_1.Uint8Value = 41;
      basic_type_1.Int16Value = 42;
      basic_type_1.Uint16Value = 43;
      basic_type_1.Int32Value = 44;
      basic_type_1.Uint32Value = 45;
      basic_type_1.Int64Value = 46;
      basic_type_1.Uint64Value = 47;

      test_msgs.msg.BasicTypes basic_type_2 = new test_msgs.msg.BasicTypes();
      basic_type_2.BoolValue = false;
      basic_type_2.ByteValue = 48;
      basic_type_2.CharValue = 49;
      basic_type_2.Float32Value = 50.1f;
      basic_type_2.Float64Value = 51.1;
      basic_type_2.Int8Value = 52;
      basic_type_2.Uint8Value = 53;
      basic_type_2.Int16Value = 54;
      basic_type_2.Uint16Value = 55;
      basic_type_2.Int32Value = 56;
      basic_type_2.Uint32Value = 57;
      basic_type_2.Int64Value = 58;
      basic_type_2.Uint64Value = 59;

      test_msgs.msg.BasicTypes basic_type_3 = new test_msgs.msg.BasicTypes();
      basic_type_3.BoolValue = true;
      basic_type_3.ByteValue = 60;
      basic_type_3.CharValue = 61;
      basic_type_3.Float32Value = 62.1f;
      basic_type_3.Float64Value = 63.1;
      basic_type_3.Int8Value = 64;
      basic_type_3.Uint8Value = 65;
      basic_type_3.Int16Value = 66;
      basic_type_3.Uint16Value = 67;
      basic_type_3.Int32Value = 68;
      basic_type_3.Uint32Value = 69;
      basic_type_3.Int64Value = 70;
      basic_type_3.Uint64Value = 71;

      msg.BasicTypesValues.Add(basic_type_1);
      msg.BasicTypesValues.Add(basic_type_2);
      msg.BasicTypesValues.Add(basic_type_3);

      bool received=false;
      Subscription<test_msgs.msg.UnboundedSequences> chatter_sub = node_array_2.CreateSubscription<test_msgs.msg.UnboundedSequences>(
        "topic_unbounded_sequences", rcv_msg =>
        {
          received=true;
          msg2 = rcv_msg;
        }
      );

      while(!received)
      {
        chatter_pub.Publish(msg);

        RCLdotnet.SpinOnce(node_array_1, 500);
        RCLdotnet.SpinOnce(node_array_2, 500);
      }

      // boolValues
      Assert.Equal(3, msg2.BoolValues.Count);
      Assert.True(msg2.BoolValues[0]);
      Assert.False(msg2.BoolValues[1]);
      Assert.True(msg2.BoolValues[2]);

      // byteValues
      Assert.Equal(3, msg2.ByteValues.Count);
      Assert.Equal(0, msg2.ByteValues[0]);
      Assert.Equal(1, msg2.ByteValues[1]);
      Assert.Equal(2, msg2.ByteValues[2]);

      // charValues
      Assert.Equal(3, msg2.CharValues.Count);
      Assert.Equal(3, msg2.CharValues[0]);
      Assert.Equal(4, msg2.CharValues[1]);
      Assert.Equal(5, msg2.CharValues[2]);

      // float32Values
      Assert.Equal(3, msg2.Float32Values.Count);
      Assert.Equal(6.1f, msg2.Float32Values[0]);
      Assert.Equal(7.1f, msg2.Float32Values[1]);
      Assert.Equal(8.1f, msg2.Float32Values[2]);

      // float64Values
      Assert.Equal(3, msg2.Float64Values.Count);
      Assert.Equal(9.1, msg2.Float64Values[0]);
      Assert.Equal(10.1, msg2.Float64Values[1]);
      Assert.Equal(11.1, msg2.Float64Values[2]);

      // int8Values
      Assert.Equal(3, msg2.Int8Values.Count);
      Assert.Equal(12, msg2.Int8Values[0]);
      Assert.Equal(13, msg2.Int8Values[1]);
      Assert.Equal(14, msg2.Int8Values[2]);

      // uint8Values
      Assert.Equal(3, msg2.Uint8Values.Count);
      Assert.Equal(15, msg2.Uint8Values[0]);
      Assert.Equal(16, msg2.Uint8Values[1]);
      Assert.Equal(17, msg2.Uint8Values[2]);

      // int16Values
      Assert.Equal(3, msg2.Int16Values.Count);
      Assert.Equal(18, msg2.Int16Values[0]);
      Assert.Equal(19, msg2.Int16Values[1]);
      Assert.Equal(20, msg2.Int16Values[2]);

      // uint16Values
      Assert.Equal(3, msg2.Uint16Values.Count);
      Assert.Equal(21, msg2.Uint16Values[0]);
      Assert.Equal(22, msg2.Uint16Values[1]);
      Assert.Equal(23, msg2.Uint16Values[2]);

      // int32Values
      Assert.Equal(3, msg2.Int32Values.Count);
      Assert.Equal(24, msg2.Int32Values[0]);
      Assert.Equal(25, msg2.Int32Values[1]);
      Assert.Equal(26, msg2.Int32Values[2]);
      
      // uint32Values
      Assert.Equal(3, msg2.Uint32Values.Count);
      Assert.Equal((uint)27, msg2.Uint32Values[0]);
      Assert.Equal((uint)28, msg2.Uint32Values[1]);
      Assert.Equal((uint)29, msg2.Uint32Values[2]);

      // int64Values
      Assert.Equal(3, msg2.Int64Values.Count);
      Assert.Equal(30, msg2.Int64Values[0]);
      Assert.Equal(31, msg2.Int64Values[1]);
      Assert.Equal(32, msg2.Int64Values[2]);
      
      // uint64Values
      Assert.Equal(3, msg2.Uint64Values.Count);
      Assert.Equal((ulong)33, msg2.Uint64Values[0]);
      Assert.Equal((ulong)34, msg2.Uint64Values[1]);
      Assert.Equal((ulong)35, msg2.Uint64Values[2]);

      // stringValues
      Assert.Equal(3, msg2.StringValues.Count);
      Assert.Equal("one", msg2.StringValues[0]);
      Assert.Equal("two", msg2.StringValues[1]);
      Assert.Equal("three", msg2.StringValues[2]);
      
      Assert.Equal(3, msg2.BasicTypesValues.Count);
      Assert.True(msg2.BasicTypesValues[0].BoolValue);
      Assert.Equal(36, msg2.BasicTypesValues[0].ByteValue);
      Assert.Equal(37, msg2.BasicTypesValues[0].CharValue);
      Assert.Equal(38.1f, msg2.BasicTypesValues[0].Float32Value);
      Assert.Equal(39.1, msg2.BasicTypesValues[0].Float64Value);
      Assert.Equal(40, msg2.BasicTypesValues[0].Int8Value);
      Assert.Equal(41, msg2.BasicTypesValues[0].Uint8Value);
      Assert.Equal(42, msg2.BasicTypesValues[0].Int16Value);
      Assert.Equal(43, msg2.BasicTypesValues[0].Uint16Value);
      Assert.Equal(44, msg2.BasicTypesValues[0].Int32Value);
      Assert.Equal((uint)45, msg2.BasicTypesValues[0].Uint32Value);
      Assert.Equal(46, msg2.BasicTypesValues[0].Int64Value);
      Assert.Equal((ulong)47, msg2.BasicTypesValues[0].Uint64Value);

      Assert.False(msg2.BasicTypesValues[1].BoolValue);
      Assert.Equal(48, msg2.BasicTypesValues[1].ByteValue);
      Assert.Equal(49, msg2.BasicTypesValues[1].CharValue);
      Assert.Equal(50.1f, msg2.BasicTypesValues[1].Float32Value);
      Assert.Equal(51.1, msg2.BasicTypesValues[1].Float64Value);
      Assert.Equal(52, msg2.BasicTypesValues[1].Int8Value);
      Assert.Equal(53, msg2.BasicTypesValues[1].Uint8Value);
      Assert.Equal(54, msg2.BasicTypesValues[1].Int16Value);
      Assert.Equal(55, msg2.BasicTypesValues[1].Uint16Value);
      Assert.Equal(56, msg2.BasicTypesValues[1].Int32Value);
      Assert.Equal((uint)57, msg2.BasicTypesValues[1].Uint32Value);
      Assert.Equal(58, msg2.BasicTypesValues[1].Int64Value);
      Assert.Equal((ulong)59, msg2.BasicTypesValues[1].Uint64Value);
 
      Assert.True(msg2.BasicTypesValues[2].BoolValue);
      Assert.Equal(60, msg2.BasicTypesValues[2].ByteValue);
      Assert.Equal(61, msg2.BasicTypesValues[2].CharValue);
      Assert.Equal(62.1f, msg2.BasicTypesValues[2].Float32Value);
      Assert.Equal(63.1, msg2.BasicTypesValues[2].Float64Value);
      Assert.Equal(64, msg2.BasicTypesValues[2].Int8Value);
      Assert.Equal(65, msg2.BasicTypesValues[2].Uint8Value);
      Assert.Equal(66, msg2.BasicTypesValues[2].Int16Value);
      Assert.Equal(67, msg2.BasicTypesValues[2].Uint16Value);
      Assert.Equal(68, msg2.BasicTypesValues[2].Int32Value);
      Assert.Equal((uint)69, msg2.BasicTypesValues[2].Uint32Value);
      Assert.Equal(70, msg2.BasicTypesValues[2].Int64Value);
      Assert.Equal((ulong)71, msg2.BasicTypesValues[2].Uint64Value);
    }

    [Fact]
    public void TestPublishBoundedSequences()
    {
      RCLdotnet.Init();
      Node node_array_1 = RCLdotnet.CreateNode("test_bounded_sequences_1");
      Node node_array_2 = RCLdotnet.CreateNode("test_bounded_sequences_2");
      Publisher<test_msgs.msg.BoundedSequences> chatter_pub = node_array_1.CreatePublisher<test_msgs.msg.BoundedSequences>("topic_bounded_sequences");

      test_msgs.msg.BoundedSequences msg = new test_msgs.msg.BoundedSequences();
      test_msgs.msg.BoundedSequences msg2 = new test_msgs.msg.BoundedSequences();

      // boolValues
      msg.BoolValues.Add(true);
      msg.BoolValues.Add(false);
      msg.BoolValues.Add(true);

      // byteValues
      msg.ByteValues.Add(0);
      msg.ByteValues.Add(1);
      msg.ByteValues.Add(2);

      // charValues
      msg.CharValues.Add(3);
      msg.CharValues.Add(4);
      msg.CharValues.Add(5);

      // float32Values
      msg.Float32Values.Add(6.1f);
      msg.Float32Values.Add(7.1f);
      msg.Float32Values.Add(8.1f);

      // float64Values
      msg.Float64Values.Add(9.1);
      msg.Float64Values.Add(10.1);
      msg.Float64Values.Add(11.1);

      // int8Values
      msg.Int8Values.Add(12);
      msg.Int8Values.Add(13);
      msg.Int8Values.Add(14);

      // uint8Values
      msg.Uint8Values.Add(15);
      msg.Uint8Values.Add(16);
      msg.Uint8Values.Add(17);

      // int16Values
      msg.Int16Values.Add(18);
      msg.Int16Values.Add(19);
      msg.Int16Values.Add(20);

      // uint16Values
      msg.Uint16Values.Add(21);
      msg.Uint16Values.Add(22);
      msg.Uint16Values.Add(23);

      // int32Values
      msg.Int32Values.Add(24);
      msg.Int32Values.Add(25);
      msg.Int32Values.Add(26);

      // uint32Values
      msg.Uint32Values.Add(27);
      msg.Uint32Values.Add(28);
      msg.Uint32Values.Add(29);

      // int64Values
      msg.Int64Values.Add(30);
      msg.Int64Values.Add(31);
      msg.Int64Values.Add(32);

      // uint64Values
      msg.Uint64Values.Add(33);
      msg.Uint64Values.Add(34);
      msg.Uint64Values.Add(35);

      // stringValues
      msg.StringValues.Add("one");
      msg.StringValues.Add("two");
      msg.StringValues.Add("three");

      test_msgs.msg.BasicTypes basic_type_1 = new test_msgs.msg.BasicTypes();
      basic_type_1.BoolValue = true;
      basic_type_1.ByteValue = 36;
      basic_type_1.CharValue = 37;
      basic_type_1.Float32Value = 38.1f;
      basic_type_1.Float64Value = 39.1;
      basic_type_1.Int8Value = 40;
      basic_type_1.Uint8Value = 41;
      basic_type_1.Int16Value = 42;
      basic_type_1.Uint16Value = 43;
      basic_type_1.Int32Value = 44;
      basic_type_1.Uint32Value = 45;
      basic_type_1.Int64Value = 46;
      basic_type_1.Uint64Value = 47;

      test_msgs.msg.BasicTypes basic_type_2 = new test_msgs.msg.BasicTypes();
      basic_type_2.BoolValue = false;
      basic_type_2.ByteValue = 48;
      basic_type_2.CharValue = 49;
      basic_type_2.Float32Value = 50.1f;
      basic_type_2.Float64Value = 51.1;
      basic_type_2.Int8Value = 52;
      basic_type_2.Uint8Value = 53;
      basic_type_2.Int16Value = 54;
      basic_type_2.Uint16Value = 55;
      basic_type_2.Int32Value = 56;
      basic_type_2.Uint32Value = 57;
      basic_type_2.Int64Value = 58;
      basic_type_2.Uint64Value = 59;

      test_msgs.msg.BasicTypes basic_type_3 = new test_msgs.msg.BasicTypes();
      basic_type_3.BoolValue = true;
      basic_type_3.ByteValue = 60;
      basic_type_3.CharValue = 61;
      basic_type_3.Float32Value = 62.1f;
      basic_type_3.Float64Value = 63.1;
      basic_type_3.Int8Value = 64;
      basic_type_3.Uint8Value = 65;
      basic_type_3.Int16Value = 66;
      basic_type_3.Uint16Value = 67;
      basic_type_3.Int32Value = 68;
      basic_type_3.Uint32Value = 69;
      basic_type_3.Int64Value = 70;
      basic_type_3.Uint64Value = 71;

      msg.BasicTypesValues.Add(basic_type_1);
      msg.BasicTypesValues.Add(basic_type_2);
      msg.BasicTypesValues.Add(basic_type_3);

      bool received=false;
      Subscription<test_msgs.msg.BoundedSequences> chatter_sub = node_array_2.CreateSubscription<test_msgs.msg.BoundedSequences>(
        "topic_bounded_sequences", rcv_msg =>
        {
          received=true;
          msg2 = rcv_msg;
        }
      );

      while(!received)
      {
        chatter_pub.Publish(msg);

        RCLdotnet.SpinOnce(node_array_1, 500);
        RCLdotnet.SpinOnce(node_array_2, 500);
      }

      // boolValues
      Assert.Equal(3, test_msgs.msg.BoundedSequences.BoolValuesMaxCount);
      Assert.Equal(3, msg2.BoolValues.Count);
      Assert.True(msg2.BoolValues[0]);
      Assert.False(msg2.BoolValues[1]);
      Assert.True(msg2.BoolValues[2]);

      // byteValues
      Assert.Equal(3, test_msgs.msg.BoundedSequences.ByteValuesMaxCount);
      Assert.Equal(3, msg2.ByteValues.Count);
      Assert.Equal(0, msg2.ByteValues[0]);
      Assert.Equal(1, msg2.ByteValues[1]);
      Assert.Equal(2, msg2.ByteValues[2]);

      // charValues
      Assert.Equal(3, test_msgs.msg.BoundedSequences.CharValuesMaxCount);
      Assert.Equal(3, msg2.CharValues.Count);
      Assert.Equal(3, msg2.CharValues[0]);
      Assert.Equal(4, msg2.CharValues[1]);
      Assert.Equal(5, msg2.CharValues[2]);

      // float32Values
      Assert.Equal(3, test_msgs.msg.BoundedSequences.Float32ValuesMaxCount);
      Assert.Equal(3, msg2.Float32Values.Count);
      Assert.Equal(6.1f, msg2.Float32Values[0]);
      Assert.Equal(7.1f, msg2.Float32Values[1]);
      Assert.Equal(8.1f, msg2.Float32Values[2]);

      // float64Values
      Assert.Equal(3, test_msgs.msg.BoundedSequences.Float64ValuesMaxCount);
      Assert.Equal(3, msg2.Float64Values.Count);
      Assert.Equal(9.1, msg2.Float64Values[0]);
      Assert.Equal(10.1, msg2.Float64Values[1]);
      Assert.Equal(11.1, msg2.Float64Values[2]);

      // int8Values
      Assert.Equal(3, test_msgs.msg.BoundedSequences.Int8ValuesMaxCount);
      Assert.Equal(3, msg2.Int8Values.Count);
      Assert.Equal(12, msg2.Int8Values[0]);
      Assert.Equal(13, msg2.Int8Values[1]);
      Assert.Equal(14, msg2.Int8Values[2]);

      // uint8Values
      Assert.Equal(3, test_msgs.msg.BoundedSequences.Uint8ValuesMaxCount);
      Assert.Equal(3, msg2.Uint8Values.Count);
      Assert.Equal(15, msg2.Uint8Values[0]);
      Assert.Equal(16, msg2.Uint8Values[1]);
      Assert.Equal(17, msg2.Uint8Values[2]);

      // int16Values
      Assert.Equal(3, test_msgs.msg.BoundedSequences.Int16ValuesMaxCount);
      Assert.Equal(3, msg2.Int16Values.Count);
      Assert.Equal(18, msg2.Int16Values[0]);
      Assert.Equal(19, msg2.Int16Values[1]);
      Assert.Equal(20, msg2.Int16Values[2]);

      // uint16Values
      Assert.Equal(3, test_msgs.msg.BoundedSequences.Uint16ValuesMaxCount);
      Assert.Equal(3, msg2.Uint16Values.Count);
      Assert.Equal(21, msg2.Uint16Values[0]);
      Assert.Equal(22, msg2.Uint16Values[1]);
      Assert.Equal(23, msg2.Uint16Values[2]);

      // int32Values
      Assert.Equal(3, test_msgs.msg.BoundedSequences.Int32ValuesMaxCount);
      Assert.Equal(3, msg2.Int32Values.Count);
      Assert.Equal(24, msg2.Int32Values[0]);
      Assert.Equal(25, msg2.Int32Values[1]);
      Assert.Equal(26, msg2.Int32Values[2]);
      
      // uint32Values
      Assert.Equal(3, test_msgs.msg.BoundedSequences.Uint32ValuesMaxCount);
      Assert.Equal(3, msg2.Uint32Values.Count);
      Assert.Equal((uint)27, msg2.Uint32Values[0]);
      Assert.Equal((uint)28, msg2.Uint32Values[1]);
      Assert.Equal((uint)29, msg2.Uint32Values[2]);

      // int64Values
      Assert.Equal(3, test_msgs.msg.BoundedSequences.Int64ValuesMaxCount);
      Assert.Equal(3, msg2.Int64Values.Count);
      Assert.Equal(30, msg2.Int64Values[0]);
      Assert.Equal(31, msg2.Int64Values[1]);
      Assert.Equal(32, msg2.Int64Values[2]);
      
      // uint64Values
      Assert.Equal(3, test_msgs.msg.BoundedSequences.Uint64ValuesMaxCount);
      Assert.Equal(3, msg2.Uint64Values.Count);
      Assert.Equal((ulong)33, msg2.Uint64Values[0]);
      Assert.Equal((ulong)34, msg2.Uint64Values[1]);
      Assert.Equal((ulong)35, msg2.Uint64Values[2]);

      // stringValues
      Assert.Equal(3, test_msgs.msg.BoundedSequences.StringValuesMaxCount);
      Assert.Equal(3, msg2.StringValues.Count);
      Assert.Equal("one", msg2.StringValues[0]);
      Assert.Equal("two", msg2.StringValues[1]);
      Assert.Equal("three", msg2.StringValues[2]);
      
      Assert.Equal(3, test_msgs.msg.BoundedSequences.BasicTypesValuesMaxCount);
      Assert.Equal(3, msg2.BasicTypesValues.Count);
      Assert.True(msg2.BasicTypesValues[0].BoolValue);
      Assert.Equal(36, msg2.BasicTypesValues[0].ByteValue);
      Assert.Equal(37, msg2.BasicTypesValues[0].CharValue);
      Assert.Equal(38.1f, msg2.BasicTypesValues[0].Float32Value);
      Assert.Equal(39.1, msg2.BasicTypesValues[0].Float64Value);
      Assert.Equal(40, msg2.BasicTypesValues[0].Int8Value);
      Assert.Equal(41, msg2.BasicTypesValues[0].Uint8Value);
      Assert.Equal(42, msg2.BasicTypesValues[0].Int16Value);
      Assert.Equal(43, msg2.BasicTypesValues[0].Uint16Value);
      Assert.Equal(44, msg2.BasicTypesValues[0].Int32Value);
      Assert.Equal((uint)45, msg2.BasicTypesValues[0].Uint32Value);
      Assert.Equal(46, msg2.BasicTypesValues[0].Int64Value);
      Assert.Equal((ulong)47, msg2.BasicTypesValues[0].Uint64Value);

      Assert.False(msg2.BasicTypesValues[1].BoolValue);
      Assert.Equal(48, msg2.BasicTypesValues[1].ByteValue);
      Assert.Equal(49, msg2.BasicTypesValues[1].CharValue);
      Assert.Equal(50.1f, msg2.BasicTypesValues[1].Float32Value);
      Assert.Equal(51.1, msg2.BasicTypesValues[1].Float64Value);
      Assert.Equal(52, msg2.BasicTypesValues[1].Int8Value);
      Assert.Equal(53, msg2.BasicTypesValues[1].Uint8Value);
      Assert.Equal(54, msg2.BasicTypesValues[1].Int16Value);
      Assert.Equal(55, msg2.BasicTypesValues[1].Uint16Value);
      Assert.Equal(56, msg2.BasicTypesValues[1].Int32Value);
      Assert.Equal((uint)57, msg2.BasicTypesValues[1].Uint32Value);
      Assert.Equal(58, msg2.BasicTypesValues[1].Int64Value);
      Assert.Equal((ulong)59, msg2.BasicTypesValues[1].Uint64Value);
 
      Assert.True(msg2.BasicTypesValues[2].BoolValue);
      Assert.Equal(60, msg2.BasicTypesValues[2].ByteValue);
      Assert.Equal(61, msg2.BasicTypesValues[2].CharValue);
      Assert.Equal(62.1f, msg2.BasicTypesValues[2].Float32Value);
      Assert.Equal(63.1, msg2.BasicTypesValues[2].Float64Value);
      Assert.Equal(64, msg2.BasicTypesValues[2].Int8Value);
      Assert.Equal(65, msg2.BasicTypesValues[2].Uint8Value);
      Assert.Equal(66, msg2.BasicTypesValues[2].Int16Value);
      Assert.Equal(67, msg2.BasicTypesValues[2].Uint16Value);
      Assert.Equal(68, msg2.BasicTypesValues[2].Int32Value);
      Assert.Equal((uint)69, msg2.BasicTypesValues[2].Uint32Value);
      Assert.Equal(70, msg2.BasicTypesValues[2].Int64Value);
      Assert.Equal((ulong)71, msg2.BasicTypesValues[2].Uint64Value);
    }

    [Fact]
    public void TestPublishBoundedSequencesSizeCheck()
    {
      RCLdotnet.Init();
      var nodeBoundedSequencesSizeCheck = RCLdotnet.CreateNode("test_bounded_sequences_size_check");
      var chatterPub = nodeBoundedSequencesSizeCheck.CreatePublisher<test_msgs.msg.BoundedSequences>("topic_bounded_sequences_size_check");

      var msg = new test_msgs.msg.BoundedSequences();
      msg.StringValues = new List<string>
      {
        "0",
        "1",
        "2",
        "3",
      };
      
      var exception = Assert.Throws<Exception>(() => chatterPub.Publish(msg));
      Assert.Equal("Invalid size of bounded sequence 'StringValues'.", exception.Message);
    }
  }
}
