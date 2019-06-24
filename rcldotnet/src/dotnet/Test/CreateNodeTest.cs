using NUnit.Framework;
using System;
namespace rclcs.Test
{
    [TestFixture]
    public class CreateNodeTest
    {
        Context context;

        [SetUp]
        public void SetUp()
        {
            context = new Context();
            Rclcs.Init(context);
        }

        [TearDown]
        public void TearDown()
        {
            Rclcs.Shutdown(context);
        }

        [Test]
        public void CreateNode()
        {
            string nodeName = "create_node_test";
            Rclcs.CreateNode(nodeName, context).Dispose();
        }

        [Test]
        public void CreateNodeWithNamespace()
        {
            string nodeName = "create_node_test";
            string nodeNamespace = "/ns";
            Node node = Rclcs.CreateNode(nodeName, context, nodeNamespace: nodeNamespace);
            Assert.That(node.Namespace, Is.EqualTo("/ns"));
            node.Dispose();
        }

        [Test]
        public void CreateNodeWithEmptyNamespace()
        {
            string nodeName = "create_node_test";
            string nodeNamespace = "";
            Node node = Rclcs.CreateNode(nodeName, context, nodeNamespace: nodeNamespace);
            Assert.That(node.Namespace, Is.EqualTo("/"));
            node.Dispose();
        }

        [Test]
        public void CreateNodeWithRelativeNamespace()
        {
            string nodeName = "create_node_test";
            string nodeNamespace = "ns";
            Node node = Rclcs.CreateNode(nodeName, context, nodeNamespace: nodeNamespace);
            Assert.That(node.Namespace, Is.EqualTo("/ns"));
            node.Dispose();
        }

        [Test]
        public void CreateNodeWithInvalidName()
        {
            string nodeName = "create_node_test_invaild_name?";
            Assert.That( () => { Rclcs.CreateNode(nodeName, context).DestroyNode(); }, 
                         Throws.TypeOf<InvalidNodeNameException>());
        }

        [Test]
        public void CreateNodeWithInvalidRelativeNamespace()
        {
            string nodeName = "create_node_test_invalid_namespace";
            string nodeNamespace = "invalid_namespace?";

            Assert.That( () => { Rclcs.CreateNode(nodeName, context, nodeNamespace: nodeNamespace).DestroyNode(); }, 
                         Throws.TypeOf<InvalidNamespaceException>());
        }
    }
}