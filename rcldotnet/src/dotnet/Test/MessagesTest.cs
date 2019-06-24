using NUnit.Framework;
using System;
using System.Linq;
using System.Collections.Generic;

namespace rclcs.Test
{
    [TestFixture()]
    public class MessagesTest
    {
        [Test]
        public void CreateMessage()
        {
            std_msgs.msg.Bool msg = new std_msgs.msg.Bool();
        }

        [Test]
        public void SetBoolData()
        {
            std_msgs.msg.Bool msg = new std_msgs.msg.Bool();
            Assert.That(msg.data, Is.False);
            msg.data = true;
            Assert.That(msg.data, Is.True);
            msg.data = false;
            Assert.That(msg.data, Is.False);
        }

        [Test]
        public void SetInt64Data()
        {
            std_msgs.msg.Int64 msg = new std_msgs.msg.Int64();
            Assert.That(msg.data, Is.EqualTo(0));
            msg.data = 12345;
            Assert.That(msg.data, Is.EqualTo(12345));
        }

        [Test]
        public void SetStringData()
        {
            std_msgs.msg.String msg = new std_msgs.msg.String();
            Assert.That(msg.data, Is.EqualTo(""));
            msg.data = "Show me what you got!";
            Assert.That(msg.data, Is.EqualTo("Show me what you got!"));
        }

        [Test]
        public void SetPrimitives()
        {
            test_msgs.msg.Primitives msg = new test_msgs.msg.Primitives();
            msg.int32_value = 24;
            Assert.That(msg.int32_value, Is.EqualTo(24));
            msg.string_value = "Turtles all the way down";
            Assert.That(msg.string_value, Is.EqualTo("Turtles all the way down"));
            msg.float32_value = 3.14F;
            Assert.That(msg.float32_value, Is.EqualTo(3.14F));
        }

        [Test]
        public void SetDynamicArrayPrimitives()
        {
            test_msgs.msg.DynamicArrayPrimitives msg = new test_msgs.msg.DynamicArrayPrimitives();
            List<bool> setBoolList = new List<bool>();
            setBoolList.Add(true);
            setBoolList.Add(false);
            msg.bool_values = setBoolList;
            List<bool> getBoolList = msg.bool_values;
            Assert.That(getBoolList.Count, Is.EqualTo(2));
            Assert.That(getBoolList[0], Is.True);
            Assert.That(getBoolList[1], Is.False);

            List<int> setIntList = new List<int>();
            setIntList.Add(123);
            setIntList.Add(456);
            test_msgs.msg.DynamicArrayPrimitives msg2 = new test_msgs.msg.DynamicArrayPrimitives();
            msg2.int32_values = setIntList;
            List<int> getIntList = msg2.int32_values;
            Assert.That(getIntList.Count, Is.EqualTo(2));
            Assert.That(getIntList[0], Is.EqualTo(123));

            List<string> setStringList = new List<string>();
            setStringList.Add("Hello");
            setStringList.Add("world");
            test_msgs.msg.DynamicArrayPrimitives msg3 = new test_msgs.msg.DynamicArrayPrimitives();
            msg3.string_values = setStringList;
            List<string> getStringList = msg3.string_values;
            Assert.That(getStringList.Count, Is.EqualTo(2));
            Assert.That(getStringList[0], Is.EqualTo("Hello"));
            Assert.That(getStringList[1], Is.EqualTo("world"));
        }

        [Test]
        public void SetBoundedArrayPrimitives()
        {
            test_msgs.msg.BoundedArrayPrimitives msg = new test_msgs.msg.BoundedArrayPrimitives();
            List<bool> setBoolList = new List<bool>();
            setBoolList.Add(true);
            setBoolList.Add(false);
            msg.bool_values = setBoolList;
            List<bool> getBoolList = msg.bool_values;
            Assert.That(getBoolList.Count, Is.EqualTo(2));
            Assert.That(getBoolList[0], Is.True);
            Assert.That(getBoolList[1], Is.False);

            List<int> setIntList = new List<int>();
            setIntList.Add(123);
            setIntList.Add(456);
            test_msgs.msg.BoundedArrayPrimitives msg2 = new test_msgs.msg.BoundedArrayPrimitives();
            msg2.int32_values = setIntList;
            List<int> getIntList = msg2.int32_values;
            Assert.That(getIntList.Count, Is.EqualTo(2));
            Assert.That(getIntList[0], Is.EqualTo(123));

            List<string> setStringList = new List<string>();
            setStringList.Add("Hello");
            setStringList.Add("world");
            test_msgs.msg.BoundedArrayPrimitives msg3 = new test_msgs.msg.BoundedArrayPrimitives();
            msg3.string_values = setStringList;
            List<string> getStringList = msg3.string_values;
            Assert.That(getStringList.Count, Is.EqualTo(2));
            Assert.That(getStringList[0], Is.EqualTo("Hello"));
            Assert.That(getStringList[1], Is.EqualTo("world"));
        }

        [Test]
        public void SetNested()
        {
            test_msgs.msg.Nested msg = new test_msgs.msg.Nested();
            test_msgs.msg.Primitives prim_msg = msg.primitive_values;
            Assert.That(prim_msg.int32_value, Is.EqualTo(0));
            prim_msg.int32_value = 25;
            prim_msg.string_value = "wubalubadubdub";
            Assert.That(prim_msg.int32_value, Is.EqualTo(25));
            test_msgs.msg.Primitives prim_msg2 = msg.primitive_values;
            Assert.That(prim_msg2.int32_value, Is.EqualTo(25));
            Assert.That(prim_msg2.string_value, Is.EqualTo("wubalubadubdub"));
        }

        [Test]
        public void SetDynamicArrayNested()
        {
            test_msgs.msg.DynamicArrayNested msg = new test_msgs.msg.DynamicArrayNested();

            msg.primitive_values_init(3);
            Assert.That(msg.primitive_values.Count, Is.EqualTo(3));

            var listOfNestedMsgs = msg.primitive_values;

            listOfNestedMsgs[0].string_value = "hello";
            Assert.That(listOfNestedMsgs[0].string_value, Is.EqualTo("hello"));

            listOfNestedMsgs[1].string_value = "world";
            Assert.That(listOfNestedMsgs[1].string_value, Is.EqualTo("world"));

            var readListOfNestedMsgs = msg.primitive_values;
            Assert.That(readListOfNestedMsgs[0].string_value, Is.EqualTo("hello"));
            Assert.That(readListOfNestedMsgs[1].string_value, Is.EqualTo("world"));
        }

        // NOTE(samiam): memory issues, does not work yet...
        [Test]
        public void SetStaticArrayPrimitives()
        {
            test_msgs.msg.StaticArrayPrimitives msg = new test_msgs.msg.StaticArrayPrimitives();
            List<bool> setBoolList = new List<bool>();
            setBoolList.Add(true);
            setBoolList.Add(false);
            msg.bool_values = setBoolList;
            List<bool> getBoolList = msg.bool_values;
            Assert.That(getBoolList.Count, Is.EqualTo(3));
            Assert.That(getBoolList[0], Is.True);
            Assert.That(getBoolList[1], Is.False);

            //List<int> setIntList = new List<int>();
            //setIntList.Add(123);
            //setIntList.Add(456);
            //test_msgs.msg.StaticArrayPrimitives msg2 = new test_msgs.msg.StaticArrayPrimitives();
            //msg2.int32_values = setIntList;
            //List<int> getIntList = msg2.int32_values;
            //Assert.That(getIntList.Count, Is.EqualTo(3));
            //Assert.That(getIntList[0], Is.EqualTo(123));

            //List<string> setStringList = new List<string>();
            //setStringList.Add("Hello");
            //setStringList.Add("world");
            //test_msgs.msg.StaticArrayPrimitives msg3 = new test_msgs.msg.StaticArrayPrimitives();
            //msg3.string_values = setStringList;
            //List<string> getStringList = msg3.string_values;
            //Assert.That(getStringList.Count, Is.EqualTo(3));
            //Assert.That(getStringList[0], Is.EqualTo("Hello"));
            //Assert.That(getStringList[1], Is.EqualTo("world"));
        }
    }
}
