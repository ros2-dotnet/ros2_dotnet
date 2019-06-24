using NUnit.Framework;
using System;
namespace rclcs.Test
{
    [TestFixture]
    public class ClockTest
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
        public void CreateClock()
        {
            Clock clock = new Clock();
        }

        [Test]
        public void ClockGetNow()
        {
            Clock clock = new Clock();
            RosTime timeNow = clock.Now;
            Assert.That(timeNow.sec, Is.Not.EqualTo(0));
        }

        [Test]
        public void RosTimeGreaterThan()
        {
            Clock clock = new Clock();
            RosTime firstTime = clock.Now;
            System.Threading.Thread.Sleep(10);
            RosTime secondTime = clock.Now;

            Assert.That(secondTime, Is.GreaterThan(firstTime));
        }

        [Test]
        public void RosTimeLessThan()
        {
            Clock clock = new Clock();
            RosTime firstTime = clock.Now;
            System.Threading.Thread.Sleep(10);
            RosTime secondTime = clock.Now;

            Assert.That(firstTime, Is.LessThan(secondTime));
        }

        [Test]
        public void RosTimeEquals()
        {
            Clock clock = new Clock();
            RosTime timeNow = clock.Now;

            Assert.That(timeNow, Is.EqualTo(timeNow));
        }


        [Test]
        public void RosTimeSeconds()
        {
            Clock clock = new Clock();

            RosTime oneSecond = clock.CreateRosTime(1.0d);
            Assert.That(oneSecond.Seconds, Is.EqualTo(1.0d));

            RosTime moreThanOneSecond = clock.CreateRosTime(2.3d);
            Assert.That(moreThanOneSecond.Seconds, Is.EqualTo(2.3d));
        }

        [Test]
        public void RosTimeAdd()
        {
            Clock clock = new Clock();
            RosTime overOneSecond = clock.CreateRosTime(1.1d);
            RosTime twoSeconds = clock.CreateRosTime(2.0d);
            RosTime sumSeconds = overOneSecond + twoSeconds;
            Assert.That(sumSeconds.Seconds, Is.EqualTo(3.1d));

            RosTime oneAndAHalfSeconds = clock.CreateRosTime(1.5d);
            RosTime moreThanTwoSeconds = clock.CreateRosTime(2.1d);
            sumSeconds = oneAndAHalfSeconds + moreThanTwoSeconds;
            Assert.That(sumSeconds.Seconds, Is.EqualTo(3.6d));

            RosTime twoAndAHalfSeconds = clock.CreateRosTime(2.5d);
            sumSeconds = oneAndAHalfSeconds + twoAndAHalfSeconds;
            Assert.That(sumSeconds.Seconds, Is.EqualTo(4.0d));
        }

        [Test]
        public void RosTimeSubtract()
        {
            Clock clock = new Clock();
            RosTime oneSecond = clock.CreateRosTime(1.0d);
            RosTime twoSeconds = clock.CreateRosTime(2.0d);
            RosTime sumSeconds = twoSeconds - oneSecond;
            Assert.That(sumSeconds.Seconds, Is.EqualTo(1.0d));

            RosTime oneAndAHalfSeconds = clock.CreateRosTime(1.5d);
            RosTime moreThanTwoSeconds = clock.CreateRosTime(2.1d);
            sumSeconds = moreThanTwoSeconds - oneAndAHalfSeconds;
            Assert.That(sumSeconds.Seconds, Is.EqualTo(0.6d));

            RosTime moreThanTwoAndAHalfSeconds = clock.CreateRosTime(2.9d);
            sumSeconds = moreThanTwoAndAHalfSeconds - oneAndAHalfSeconds;
            Assert.That(sumSeconds.Seconds, Is.EqualTo(1.4d));
        }

        [Test]
        public void RosTimeDelay()
        {
            Clock clock = new Clock();
            RosTime oneSecond = clock.CreateRosTime(1.0d);

            RosTime delayedTime = oneSecond.Delay(1.0d);
            Assert.That(delayedTime.Seconds, Is.EqualTo(2.0d));

            delayedTime = oneSecond.Delay(3.4d);
            Assert.That(delayedTime.Seconds, Is.EqualTo(4.4d));
        }

        [Test]
        public void RosTimeIsInThePast()
        {
            Clock clock = new Clock();

            RosTime timeNow = clock.Now;
            Assert.That(timeNow.IsInThePast, Is.True);

            timeNow += clock.CreateRosTime(1.0d);
            Assert.That(timeNow.IsInThePast, Is.False);
        }

        [Test]
        public void RosTimeIsInTheFuture()
        {
            Clock clock = new Clock();

            RosTime timeNow = clock.Now;
            Assert.That(timeNow.IsInTheFuture, Is.False);

            timeNow += clock.CreateRosTime(1.0d);
            Assert.That(timeNow.IsInTheFuture, Is.True);
        }

    }
}
