using NUnit.Framework;
using System;

using rclcs;

namespace rclcs.Test
{
    [TestFixture]
    public class InitShutdownTest
    {
        [Test]
        public void Init()
        {
            Context context = new Context();
            Rclcs.Init(context);
            try
            {
                Rclcs.Shutdown(context);
            }
            catch (RuntimeError)
            {
            }
        }

        [Test]
        public void InitShutdown()
        {
           Context context = new Context();
            Rclcs.Init(context);
            Rclcs.Shutdown(context);
        }

        [Test]
        public void InitShutdownSequence()
        {
            // local
            Context context = new Context();
            Rclcs.Init(context);
            Rclcs.Shutdown(context);
            context = new Context();
            Rclcs.Init(context);
            Rclcs.Shutdown(context);
        }

        [Test]
        public void DoubleInit()
        {
            Context context = new Context();
            Rclcs.Init(context);
            Assert.That(() => { Rclcs.Init(context); }, Throws.TypeOf<RuntimeError>());
            Rclcs.Shutdown(context);
        }

        [Test]
        public void DoubleShutdown()
        {
            Context context = new Context();
            Rclcs.Init(context);
            Rclcs.Shutdown(context);
            Assert.That(() => { Rclcs.Shutdown(context); }, Throws.TypeOf<RuntimeError>());
        }

        [Test]
        public void CreateNodeWithoutInit()
        {
            Context context = new Context();
            Assert.That(() => { Rclcs.CreateNode("foo", context); }, Throws.TypeOf<NotInitializedException>());
        }

        
    }

}
