using NUnit.Framework;
using System;

namespace rclcs.Test
{
    [TestFixture]
    public class NodeTest
    {
        Context context;
        Node node;

        string TEST_NODE = "my_node";
        string TEST_NAMESPACE = "/my_ns";


        [SetUp]
        public void SetUp()
        {
            context = new Context();
            Rclcs.Init(context);
            node = Rclcs.CreateNode(TEST_NODE, nodeNamespace: TEST_NAMESPACE, context: context);
        }

        [TearDown]
        public void TearDown()
        {
            node.Dispose();
            Rclcs.Shutdown(context);
        }

        [Test]
        public void Accessors()
        {
            Assert.That(node.Name, Is.EqualTo("my_node"));
            Assert.That(node.Namespace, Is.EqualTo("/my_ns"));
        }

        [Test]
        public void CreatePublisher()
        {
            Publisher<std_msgs.msg.Bool> publisher = node.CreatePublisher<std_msgs.msg.Bool>("test_topic");
            publisher.Dispose();

            using (publisher = node.CreatePublisher<std_msgs.msg.Bool>("test_topic"))
            {
            }
        }

        [Test]
        public void Publish()
        {
            using (Publisher<std_msgs.msg.Bool> publisher = node.CreatePublisher<std_msgs.msg.Bool>("test_topic"))
            {
                publisher.Publish(new std_msgs.msg.Bool());
            }
        }

        [Test]
        public void CreateSubscription()
        {
            Subscription<std_msgs.msg.Bool> subscription = node.CreateSubscription<std_msgs.msg.Bool>(
                "/subscription_topic", msg => Console.WriteLine("I heard: [" + msg.data + "]"));
            subscription.Dispose();

            using (subscription = node.CreateSubscription<std_msgs.msg.Bool>("test_topic", msg => Console.WriteLine("Got message")))
            {
            }
        }

    }
}
