using NUnit.Framework;
using System;
namespace rclcs.Test
{
    [TestFixture]
    public class WaitTest
    {
        Context context;
        Node node;

        [SetUp]
        public void SetUp()
        {
            context = new Context();
            Rclcs.Init(context);
            node = new Node("test_node", context);
            Subscription<std_msgs.msg.Int64> subscription = node.CreateSubscription<std_msgs.msg.Int64>("/test_topic", (msg) => { });
        }

        [TearDown]
        public void TearDown()
        {
            node.Dispose();
            Rclcs.Shutdown(context);
        }

        [Test]
        public void TimeoutSecToNsec()
        {
            Assert.That(Utils.TimeoutSecToNsec(0.1), Is.EqualTo(100000000));
            Assert.That(Utils.TimeoutSecToNsec(0), Is.EqualTo(0));

            Assert.Throws<RuntimeError>( () => { Utils.TimeoutSecToNsec(-0.1); });
        }

        [Test]
        public void Create()
        {
            WaitSet waitSet = new WaitSet();
        }

        [Test]
        public void WaitForReadySubscriptionCallback()
        {
            WaitSet waitSet = new WaitSet(node.Subscriptions);
            waitSet.Wait(0.1);
        }
    }
}
