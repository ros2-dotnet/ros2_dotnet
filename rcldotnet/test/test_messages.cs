using System;
using System.Collections.Generic;
using System.Linq;
using ROS2;
using Xunit;

namespace RCLdotnetTests
{
    public class TestMessages
    {
        [Fact]
        public void TestPublishString()
        {
            RCLdotnet.Init();
            Node nodeString1 = RCLdotnet.CreateNode("test_string_1");
            Node nodeString2 = RCLdotnet.CreateNode("test_string_2");
            Publisher<std_msgs.msg.String> chatterPub = nodeString1.CreatePublisher<std_msgs.msg.String>("topic_string");

            std_msgs.msg.String msg = new std_msgs.msg.String();
            std_msgs.msg.String msg2 = new std_msgs.msg.String();
            msg.Data = "Hello";

            bool received = false;
            Subscription<std_msgs.msg.String> chatter_sub = nodeString2.CreateSubscription<std_msgs.msg.String>(
                "topic_string", rcvMsg =>
                {
                    received = true;
                    msg2 = rcvMsg;
                }
            );

            while (!received)
            {
                chatterPub.Publish(msg);

                RCLdotnet.SpinOnce(nodeString1, 500);
                RCLdotnet.SpinOnce(nodeString2, 500);
            }
            Assert.Equal("Hello", msg2.Data);
        }

        [Fact]
        public void TestPublishBuiltins()
        {
            RCLdotnet.Init();
            Node nodeBuiltins1 = RCLdotnet.CreateNode("test_builtins_1");
            Node nodeBuiltins2 = RCLdotnet.CreateNode("test_builtins_2");
            Publisher<test_msgs.msg.Builtins> chatterPub = nodeBuiltins1.CreatePublisher<test_msgs.msg.Builtins>("topic_builtins");

            test_msgs.msg.Builtins msg = new test_msgs.msg.Builtins();
            test_msgs.msg.Builtins msg2 = new test_msgs.msg.Builtins();
            msg.DurationValue.Sec = 1;
            msg.DurationValue.Nanosec = 2u;
            msg.TimeValue.Sec = 3;
            msg.TimeValue.Nanosec = 4u;

            bool received = false;
            Subscription<test_msgs.msg.Builtins> chatterSub = nodeBuiltins2.CreateSubscription<test_msgs.msg.Builtins>(
                "topic_builtins", rcvMsg =>
                {
                    received = true;
                    msg2 = rcvMsg;
                }
            );

            while (!received)
            {
                chatterPub.Publish(msg);

                RCLdotnet.SpinOnce(nodeBuiltins1, 500);
                RCLdotnet.SpinOnce(nodeBuiltins2, 500);
            }

            Assert.Equal(1, msg2.DurationValue.Sec);
            Assert.Equal(2u, msg2.DurationValue.Nanosec);
            Assert.Equal(3, msg2.TimeValue.Sec);
            Assert.Equal(4u, msg2.TimeValue.Nanosec);
        }

        [Fact]
        public void TestPublishArrays()
        {
            RCLdotnet.Init();
            Node nodeArray1 = RCLdotnet.CreateNode("test_arrays_1");
            Node nodeArray2 = RCLdotnet.CreateNode("test_arrays_2");
            Publisher<test_msgs.msg.Arrays> chatterPub = nodeArray1.CreatePublisher<test_msgs.msg.Arrays>("topic_array");

            test_msgs.msg.Arrays msg = new test_msgs.msg.Arrays();
            test_msgs.msg.Arrays msg2 = new test_msgs.msg.Arrays();

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
            test_msgs.msg.BasicTypes basicType1 = new test_msgs.msg.BasicTypes();
            basicType1.BoolValue = true;
            basicType1.ByteValue = 36;
            basicType1.CharValue = 37;
            basicType1.Float32Value = 38.1f;
            basicType1.Float64Value = 39.1;
            basicType1.Int8Value = 40;
            basicType1.Uint8Value = 41;
            basicType1.Int16Value = 42;
            basicType1.Uint16Value = 43;
            basicType1.Int32Value = 44;
            basicType1.Uint32Value = 45;
            basicType1.Int64Value = 46;
            basicType1.Uint64Value = 47;

            test_msgs.msg.BasicTypes basicType2 = new test_msgs.msg.BasicTypes();
            basicType2.BoolValue = false;
            basicType2.ByteValue = 48;
            basicType2.CharValue = 49;
            basicType2.Float32Value = 50.1f;
            basicType2.Float64Value = 51.1;
            basicType2.Int8Value = 52;
            basicType2.Uint8Value = 53;
            basicType2.Int16Value = 54;
            basicType2.Uint16Value = 55;
            basicType2.Int32Value = 56;
            basicType2.Uint32Value = 57;
            basicType2.Int64Value = 58;
            basicType2.Uint64Value = 59;

            test_msgs.msg.BasicTypes basicType3 = new test_msgs.msg.BasicTypes();
            basicType3.BoolValue = true;
            basicType3.ByteValue = 60;
            basicType3.CharValue = 61;
            basicType3.Float32Value = 62.1f;
            basicType3.Float64Value = 63.1;
            basicType3.Int8Value = 64;
            basicType3.Uint8Value = 65;
            basicType3.Int16Value = 66;
            basicType3.Uint16Value = 67;
            basicType3.Int32Value = 68;
            basicType3.Uint32Value = 69;
            basicType3.Int64Value = 70;
            basicType3.Uint64Value = 71;

            msg.BasicTypesValues[0] = basicType1;
            msg.BasicTypesValues[1] = basicType2;
            msg.BasicTypesValues[2] = basicType3;

            bool received = false;
            Subscription<test_msgs.msg.Arrays> chatterSub = nodeArray2.CreateSubscription<test_msgs.msg.Arrays>(
                "topic_array", rcvMsg =>
                {
                    received = true;
                    msg2 = rcvMsg;
                }
            );

            while (!received)
            {
                chatterPub.Publish(msg);

                RCLdotnet.SpinOnce(nodeArray1, 500);
                RCLdotnet.SpinOnce(nodeArray2, 500);
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
            Node nodeArray1 = RCLdotnet.CreateNode("test_unbounded_sequences_1");
            Node nodeArray2 = RCLdotnet.CreateNode("test_unbounded_sequences_2");
            Publisher<test_msgs.msg.UnboundedSequences> chatterPub = nodeArray1.CreatePublisher<test_msgs.msg.UnboundedSequences>("topic_unbounded_sequences");

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

            test_msgs.msg.BasicTypes basicType1 = new test_msgs.msg.BasicTypes();
            basicType1.BoolValue = true;
            basicType1.ByteValue = 36;
            basicType1.CharValue = 37;
            basicType1.Float32Value = 38.1f;
            basicType1.Float64Value = 39.1;
            basicType1.Int8Value = 40;
            basicType1.Uint8Value = 41;
            basicType1.Int16Value = 42;
            basicType1.Uint16Value = 43;
            basicType1.Int32Value = 44;
            basicType1.Uint32Value = 45;
            basicType1.Int64Value = 46;
            basicType1.Uint64Value = 47;

            test_msgs.msg.BasicTypes basicType2 = new test_msgs.msg.BasicTypes();
            basicType2.BoolValue = false;
            basicType2.ByteValue = 48;
            basicType2.CharValue = 49;
            basicType2.Float32Value = 50.1f;
            basicType2.Float64Value = 51.1;
            basicType2.Int8Value = 52;
            basicType2.Uint8Value = 53;
            basicType2.Int16Value = 54;
            basicType2.Uint16Value = 55;
            basicType2.Int32Value = 56;
            basicType2.Uint32Value = 57;
            basicType2.Int64Value = 58;
            basicType2.Uint64Value = 59;

            test_msgs.msg.BasicTypes basicType3 = new test_msgs.msg.BasicTypes();
            basicType3.BoolValue = true;
            basicType3.ByteValue = 60;
            basicType3.CharValue = 61;
            basicType3.Float32Value = 62.1f;
            basicType3.Float64Value = 63.1;
            basicType3.Int8Value = 64;
            basicType3.Uint8Value = 65;
            basicType3.Int16Value = 66;
            basicType3.Uint16Value = 67;
            basicType3.Int32Value = 68;
            basicType3.Uint32Value = 69;
            basicType3.Int64Value = 70;
            basicType3.Uint64Value = 71;

            msg.BasicTypesValues.Add(basicType1);
            msg.BasicTypesValues.Add(basicType2);
            msg.BasicTypesValues.Add(basicType3);

            bool received = false;
            Subscription<test_msgs.msg.UnboundedSequences> chatterSub = nodeArray2.CreateSubscription<test_msgs.msg.UnboundedSequences>(
                "topic_unbounded_sequences", rcvMsg =>
                {
                    received = true;
                    msg2 = rcvMsg;
                }
            );

            while (!received)
            {
                chatterPub.Publish(msg);

                RCLdotnet.SpinOnce(nodeArray1, 500);
                RCLdotnet.SpinOnce(nodeArray2, 500);
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
            Node nodeBoundedSequences1 = RCLdotnet.CreateNode("test_bounded_sequences_1");
            Node nodeBoundedSequences2 = RCLdotnet.CreateNode("test_bounded_sequences_2");
            Publisher<test_msgs.msg.BoundedSequences> chatter_pub = nodeBoundedSequences1.CreatePublisher<test_msgs.msg.BoundedSequences>("topic_bounded_sequences");

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

            test_msgs.msg.BasicTypes basicType1 = new test_msgs.msg.BasicTypes();
            basicType1.BoolValue = true;
            basicType1.ByteValue = 36;
            basicType1.CharValue = 37;
            basicType1.Float32Value = 38.1f;
            basicType1.Float64Value = 39.1;
            basicType1.Int8Value = 40;
            basicType1.Uint8Value = 41;
            basicType1.Int16Value = 42;
            basicType1.Uint16Value = 43;
            basicType1.Int32Value = 44;
            basicType1.Uint32Value = 45;
            basicType1.Int64Value = 46;
            basicType1.Uint64Value = 47;

            test_msgs.msg.BasicTypes basicType2 = new test_msgs.msg.BasicTypes();
            basicType2.BoolValue = false;
            basicType2.ByteValue = 48;
            basicType2.CharValue = 49;
            basicType2.Float32Value = 50.1f;
            basicType2.Float64Value = 51.1;
            basicType2.Int8Value = 52;
            basicType2.Uint8Value = 53;
            basicType2.Int16Value = 54;
            basicType2.Uint16Value = 55;
            basicType2.Int32Value = 56;
            basicType2.Uint32Value = 57;
            basicType2.Int64Value = 58;
            basicType2.Uint64Value = 59;

            test_msgs.msg.BasicTypes basicType3 = new test_msgs.msg.BasicTypes();
            basicType3.BoolValue = true;
            basicType3.ByteValue = 60;
            basicType3.CharValue = 61;
            basicType3.Float32Value = 62.1f;
            basicType3.Float64Value = 63.1;
            basicType3.Int8Value = 64;
            basicType3.Uint8Value = 65;
            basicType3.Int16Value = 66;
            basicType3.Uint16Value = 67;
            basicType3.Int32Value = 68;
            basicType3.Uint32Value = 69;
            basicType3.Int64Value = 70;
            basicType3.Uint64Value = 71;

            msg.BasicTypesValues.Add(basicType1);
            msg.BasicTypesValues.Add(basicType2);
            msg.BasicTypesValues.Add(basicType3);

            bool received = false;
            Subscription<test_msgs.msg.BoundedSequences> chatterSub = nodeBoundedSequences2.CreateSubscription<test_msgs.msg.BoundedSequences>(
                "topic_bounded_sequences", rcvMsg =>
                {
                    received = true;
                    msg2 = rcvMsg;
                }
            );

            while (!received)
            {
                chatter_pub.Publish(msg);

                RCLdotnet.SpinOnce(nodeBoundedSequences1, 500);
                RCLdotnet.SpinOnce(nodeBoundedSequences2, 500);
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

        [Fact]
        public void TestDefaults()
        {
            var defaultsMsg = new test_msgs.msg.Defaults();

            Assert.Equal(true, defaultsMsg.BoolValue);
            Assert.Equal(50, defaultsMsg.ByteValue);
            Assert.Equal(100, defaultsMsg.CharValue);
            Assert.Equal(1.125f, defaultsMsg.Float32Value);
            Assert.Equal(1.125, defaultsMsg.Float64Value);
            Assert.Equal(-50, defaultsMsg.Int8Value);
            Assert.Equal(200, defaultsMsg.Uint8Value);
            Assert.Equal(-1000, defaultsMsg.Int16Value);
            Assert.Equal(2000, defaultsMsg.Uint16Value);
            Assert.Equal(-30000, defaultsMsg.Int32Value);
            Assert.True(60000 == defaultsMsg.Uint32Value);
            Assert.Equal(-40000000, defaultsMsg.Int64Value);
            Assert.True(50000000 == defaultsMsg.Uint64Value);

            var stringsMsg = new test_msgs.msg.Strings();
            Assert.Equal("", stringsMsg.StringValue);
            Assert.Equal("Hello world!", stringsMsg.StringValueDefault1);
            Assert.Equal("Hello'world!", stringsMsg.StringValueDefault2);
            Assert.Equal("Hello\"world!", stringsMsg.StringValueDefault3);
            Assert.Equal("Hello'world!", stringsMsg.StringValueDefault4);
            Assert.Equal("Hello\"world!", stringsMsg.StringValueDefault5);
        }

        [Fact]
        public void TestDefaultsArrays()
        {
            var msg = new test_msgs.msg.Arrays();

            Assert.IsType<bool[]>(msg.BoolValues);
            Assert.Equal(new bool[3], msg.BoolValues);

            Assert.IsType<byte[]>(msg.ByteValues);
            Assert.Equal(new byte[3], msg.ByteValues);

            Assert.IsType<byte[]>(msg.CharValues);
            Assert.Equal(new byte[3], msg.CharValues);

            Assert.IsType<float[]>(msg.Float32Values);
            Assert.Equal(new float[3], msg.Float32Values);

            Assert.IsType<double[]>(msg.Float64Values);
            Assert.Equal(new double[3], msg.Float64Values);

            Assert.IsType<sbyte[]>(msg.Int8Values);
            Assert.Equal(new sbyte[3], msg.Int8Values);

            Assert.IsType<byte[]>(msg.Uint8Values);
            Assert.Equal(new byte[3], msg.Uint8Values);

            Assert.IsType<short[]>(msg.Int16Values);
            Assert.Equal(new short[3], msg.Int16Values);

            Assert.IsType<ushort[]>(msg.Uint16Values);
            Assert.Equal(new ushort[3], msg.Uint16Values);

            Assert.IsType<int[]>(msg.Int32Values);
            Assert.Equal(new int[3], msg.Int32Values);

            Assert.IsType<uint[]>(msg.Uint32Values);
            Assert.Equal(new uint[3], msg.Uint32Values);

            Assert.IsType<long[]>(msg.Int64Values);
            Assert.Equal(new long[3], msg.Int64Values);

            Assert.IsType<ulong[]>(msg.Uint64Values);
            Assert.Equal(new ulong[3], msg.Uint64Values);

            Assert.IsType<string[]>(msg.StringValues);
            Assert.Equal(
                Enumerable.Repeat("", 3).ToArray(),
                msg.StringValues);

            Assert.IsType<test_msgs.msg.BasicTypes[]>(msg.BasicTypesValues);
            Assert.Equal(3, msg.BasicTypesValues.Length);
            Assert.NotNull(msg.BasicTypesValues[0]);
            Assert.NotNull(msg.BasicTypesValues[1]);
            Assert.NotNull(msg.BasicTypesValues[2]);

            Assert.IsType<test_msgs.msg.Constants[]>(msg.ConstantsValues);
            Assert.Equal(3, msg.ConstantsValues.Length);
            Assert.NotNull(msg.ConstantsValues[0]);
            Assert.NotNull(msg.ConstantsValues[1]);
            Assert.NotNull(msg.ConstantsValues[2]);

            Assert.IsType<test_msgs.msg.Defaults[]>(msg.DefaultsValues);
            Assert.Equal(3, msg.DefaultsValues.Length);
            Assert.NotNull(msg.DefaultsValues[0]);
            Assert.NotNull(msg.DefaultsValues[1]);
            Assert.NotNull(msg.DefaultsValues[2]);

            Assert.IsType<bool[]>(msg.BoolValuesDefault);
            Assert.Equal(
                new bool[] { false, true, false },
                msg.BoolValuesDefault);

            Assert.IsType<byte[]>(msg.ByteValuesDefault);
            Assert.Equal(
                new byte[] { 0, 1, 255 },
                msg.ByteValuesDefault);

            Assert.IsType<byte[]>(msg.CharValuesDefault);
            Assert.Equal(
                new byte[] { 0, 1, 127 },
                msg.CharValuesDefault);

            Assert.IsType<float[]>(msg.Float32ValuesDefault);
            Assert.Equal(
                new float[] { 1.125f, 0.0f, -1.125f },
                msg.Float32ValuesDefault);

            Assert.IsType<double[]>(msg.Float64ValuesDefault);
            Assert.Equal(
                new double[] { 3.1415, 0.0, -3.1415 },
                msg.Float64ValuesDefault);

            Assert.IsType<sbyte[]>(msg.Int8ValuesDefault);
            Assert.Equal(
                new sbyte[] { 0, 127, -128 },
                msg.Int8ValuesDefault);

            Assert.IsType<byte[]>(msg.Uint8ValuesDefault);
            Assert.Equal(
                new byte[] { 0, 1, 255 },
                msg.Uint8ValuesDefault);

            Assert.IsType<short[]>(msg.Int16ValuesDefault);
            Assert.Equal(
                new short[] { 0, 32767, -32768 },
                msg.Int16ValuesDefault);

            Assert.IsType<ushort[]>(msg.Uint16ValuesDefault);
            Assert.Equal(
                new ushort[] { 0, 1, 65535 },
                msg.Uint16ValuesDefault);

            Assert.IsType<int[]>(msg.Int32ValuesDefault);
            Assert.Equal(
                new int[] { 0, 2147483647, -2147483648 },
                msg.Int32ValuesDefault);

            Assert.IsType<uint[]>(msg.Uint32ValuesDefault);
            Assert.Equal(
                new uint[] { 0, 1, 4294967295 },
                msg.Uint32ValuesDefault);

            Assert.IsType<long[]>(msg.Int64ValuesDefault);
            Assert.Equal(
                new long[] { 0, 9223372036854775807, -9223372036854775808 },
                msg.Int64ValuesDefault);

            Assert.IsType<ulong[]>(msg.Uint64ValuesDefault);
            Assert.Equal(
                new ulong[] { 0, 1, 18446744073709551615 },
                msg.Uint64ValuesDefault);

            Assert.IsType<string[]>(msg.StringValuesDefault);
            Assert.Equal(
                new string[] { "", "max value", "min value" },
                msg.StringValuesDefault);
        }

        [Fact]
        public void TestDefaultsUnboundedSequences()
        {
            var msg = new test_msgs.msg.UnboundedSequences();

            Assert.IsType<List<bool>>(msg.BoolValues);
            Assert.Equal(new List<bool>(), msg.BoolValues);

            Assert.IsType<List<byte>>(msg.ByteValues);
            Assert.Equal(new List<byte>(), msg.ByteValues);

            Assert.IsType<List<byte>>(msg.CharValues);
            Assert.Equal(new List<byte>(), msg.CharValues);

            Assert.IsType<List<float>>(msg.Float32Values);
            Assert.Equal(new List<float>(), msg.Float32Values);

            Assert.IsType<List<double>>(msg.Float64Values);
            Assert.Equal(new List<double>(), msg.Float64Values);

            Assert.IsType<List<sbyte>>(msg.Int8Values);
            Assert.Equal(new List<sbyte>(), msg.Int8Values);

            Assert.IsType<List<byte>>(msg.Uint8Values);
            Assert.Equal(new List<byte>(), msg.Uint8Values);

            Assert.IsType<List<short>>(msg.Int16Values);
            Assert.Equal(new List<short>(), msg.Int16Values);

            Assert.IsType<List<ushort>>(msg.Uint16Values);
            Assert.Equal(new List<ushort>(), msg.Uint16Values);

            Assert.IsType<List<int>>(msg.Int32Values);
            Assert.Equal(new List<int>(), msg.Int32Values);

            Assert.IsType<List<uint>>(msg.Uint32Values);
            Assert.Equal(new List<uint>(), msg.Uint32Values);

            Assert.IsType<List<long>>(msg.Int64Values);
            Assert.Equal(new List<long>(), msg.Int64Values);

            Assert.IsType<List<ulong>>(msg.Uint64Values);
            Assert.Equal(new List<ulong>(), msg.Uint64Values);

            Assert.IsType<List<string>>(msg.StringValues);
            Assert.Equal(new List<string>(), msg.StringValues);

            Assert.IsType<List<test_msgs.msg.BasicTypes>>(msg.BasicTypesValues);
            Assert.Equal(new List<test_msgs.msg.BasicTypes>(), msg.BasicTypesValues);

            Assert.IsType<List<test_msgs.msg.Constants>>(msg.ConstantsValues);
            Assert.Equal(new List<test_msgs.msg.Constants>(), msg.ConstantsValues);

            Assert.IsType<List<test_msgs.msg.Defaults>>(msg.DefaultsValues);
            Assert.Equal(new List<test_msgs.msg.Defaults>(), msg.DefaultsValues);

            Assert.IsType<List<bool>>(msg.BoolValuesDefault);
            Assert.Equal(
                new List<bool> { false, true, false },
                msg.BoolValuesDefault);

            Assert.IsType<List<byte>>(msg.ByteValuesDefault);
            Assert.Equal(
                new List<byte> { 0, 1, 255 },
                msg.ByteValuesDefault);

            Assert.IsType<List<byte>>(msg.CharValuesDefault);
            Assert.Equal(
                new List<byte> { 0, 1, 127 },
                msg.CharValuesDefault);

            Assert.IsType<List<float>>(msg.Float32ValuesDefault);
            Assert.Equal(
                new List<float> { 1.125f, 0.0f, -1.125f },
                msg.Float32ValuesDefault);

            Assert.IsType<List<double>>(msg.Float64ValuesDefault);
            Assert.Equal(
                new List<double> { 3.1415, 0.0, -3.1415 },
                msg.Float64ValuesDefault);

            Assert.IsType<List<sbyte>>(msg.Int8ValuesDefault);
            Assert.Equal(
                new List<sbyte> { 0, 127, -128 },
                msg.Int8ValuesDefault);

            Assert.IsType<List<byte>>(msg.Uint8ValuesDefault);
            Assert.Equal(
                new List<byte> { 0, 1, 255 },
                msg.Uint8ValuesDefault);

            Assert.IsType<List<short>>(msg.Int16ValuesDefault);
            Assert.Equal(
                new List<short> { 0, 32767, -32768 },
                msg.Int16ValuesDefault);

            Assert.IsType<List<ushort>>(msg.Uint16ValuesDefault);
            Assert.Equal(
                new List<ushort> { 0, 1, 65535 },
                msg.Uint16ValuesDefault);

            Assert.IsType<List<int>>(msg.Int32ValuesDefault);
            Assert.Equal(
                new List<int> { 0, 2147483647, -2147483648 },
                msg.Int32ValuesDefault);

            Assert.IsType<List<uint>>(msg.Uint32ValuesDefault);
            Assert.Equal(
                new List<uint> { 0, 1, 4294967295 },
                msg.Uint32ValuesDefault);

            Assert.IsType<List<long>>(msg.Int64ValuesDefault);
            Assert.Equal(
                new List<long> { 0, 9223372036854775807, -9223372036854775808 },
                msg.Int64ValuesDefault);

            Assert.IsType<List<ulong>>(msg.Uint64ValuesDefault);
            Assert.Equal(
                new List<ulong> { 0, 1, 18446744073709551615 },
                msg.Uint64ValuesDefault);

            Assert.IsType<List<string>>(msg.StringValuesDefault);
            Assert.Equal(
                new List<string> { "", "max value", "min value" },
                msg.StringValuesDefault);
        }

        [Fact]
        public void TestDefaultsBoundedSequences()
        {
            var msg = new test_msgs.msg.BoundedSequences();

            Assert.IsType<List<bool>>(msg.BoolValues);
            Assert.Equal(new List<bool>(), msg.BoolValues);

            Assert.IsType<List<byte>>(msg.ByteValues);
            Assert.Equal(new List<byte>(), msg.ByteValues);

            Assert.IsType<List<byte>>(msg.CharValues);
            Assert.Equal(new List<byte>(), msg.CharValues);

            Assert.IsType<List<float>>(msg.Float32Values);
            Assert.Equal(new List<float>(), msg.Float32Values);

            Assert.IsType<List<double>>(msg.Float64Values);
            Assert.Equal(new List<double>(), msg.Float64Values);

            Assert.IsType<List<sbyte>>(msg.Int8Values);
            Assert.Equal(new List<sbyte>(), msg.Int8Values);

            Assert.IsType<List<byte>>(msg.Uint8Values);
            Assert.Equal(new List<byte>(), msg.Uint8Values);

            Assert.IsType<List<short>>(msg.Int16Values);
            Assert.Equal(new List<short>(), msg.Int16Values);

            Assert.IsType<List<ushort>>(msg.Uint16Values);
            Assert.Equal(new List<ushort>(), msg.Uint16Values);

            Assert.IsType<List<int>>(msg.Int32Values);
            Assert.Equal(new List<int>(), msg.Int32Values);

            Assert.IsType<List<uint>>(msg.Uint32Values);
            Assert.Equal(new List<uint>(), msg.Uint32Values);

            Assert.IsType<List<long>>(msg.Int64Values);
            Assert.Equal(new List<long>(), msg.Int64Values);

            Assert.IsType<List<ulong>>(msg.Uint64Values);
            Assert.Equal(new List<ulong>(), msg.Uint64Values);

            Assert.IsType<List<string>>(msg.StringValues);
            Assert.Equal(new List<string>(), msg.StringValues);

            Assert.IsType<List<test_msgs.msg.BasicTypes>>(msg.BasicTypesValues);
            Assert.Equal(new List<test_msgs.msg.BasicTypes>(), msg.BasicTypesValues);

            Assert.IsType<List<test_msgs.msg.Constants>>(msg.ConstantsValues);
            Assert.Equal(new List<test_msgs.msg.Constants>(), msg.ConstantsValues);

            Assert.IsType<List<test_msgs.msg.Defaults>>(msg.DefaultsValues);
            Assert.Equal(new List<test_msgs.msg.Defaults>(), msg.DefaultsValues);

            Assert.IsType<List<bool>>(msg.BoolValuesDefault);
            Assert.Equal(
                new List<bool> { false, true, false },
                msg.BoolValuesDefault);

            Assert.IsType<List<byte>>(msg.ByteValuesDefault);
            Assert.Equal(
                new List<byte> { 0, 1, 255 },
                msg.ByteValuesDefault);

            Assert.IsType<List<byte>>(msg.CharValuesDefault);
            Assert.Equal(
                new List<byte> { 0, 1, 127 },
                msg.CharValuesDefault);

            Assert.IsType<List<float>>(msg.Float32ValuesDefault);
            Assert.Equal(
                new List<float> { 1.125f, 0.0f, -1.125f },
                msg.Float32ValuesDefault);

            Assert.IsType<List<double>>(msg.Float64ValuesDefault);
            Assert.Equal(
                new List<double> { 3.1415, 0.0, -3.1415 },
                msg.Float64ValuesDefault);

            Assert.IsType<List<sbyte>>(msg.Int8ValuesDefault);
            Assert.Equal(
                new List<sbyte> { 0, 127, -128 },
                msg.Int8ValuesDefault);

            Assert.IsType<List<byte>>(msg.Uint8ValuesDefault);
            Assert.Equal(
                new List<byte> { 0, 1, 255 },
                msg.Uint8ValuesDefault);

            Assert.IsType<List<short>>(msg.Int16ValuesDefault);
            Assert.Equal(
                new List<short> { 0, 32767, -32768 },
                msg.Int16ValuesDefault);

            Assert.IsType<List<ushort>>(msg.Uint16ValuesDefault);
            Assert.Equal(
                new List<ushort> { 0, 1, 65535 },
                msg.Uint16ValuesDefault);

            Assert.IsType<List<int>>(msg.Int32ValuesDefault);
            Assert.Equal(
                new List<int> { 0, 2147483647, -2147483648 },
                msg.Int32ValuesDefault);

            Assert.IsType<List<uint>>(msg.Uint32ValuesDefault);
            Assert.Equal(
                new List<uint> { 0, 1, 4294967295 },
                msg.Uint32ValuesDefault);

            Assert.IsType<List<long>>(msg.Int64ValuesDefault);
            Assert.Equal(
                new List<long> { 0, 9223372036854775807, -9223372036854775808 },
                msg.Int64ValuesDefault);

            Assert.IsType<List<ulong>>(msg.Uint64ValuesDefault);
            Assert.Equal(
                new List<ulong> { 0, 1, 18446744073709551615 },
                msg.Uint64ValuesDefault);

            Assert.IsType<List<string>>(msg.StringValuesDefault);
            Assert.Equal(
                new List<string> { "", "max value", "min value" },
                msg.StringValuesDefault);
        }
    }
}
