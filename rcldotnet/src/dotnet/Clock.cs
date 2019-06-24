/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0 (the "License");

You may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System;
using System.Collections.Generic;

namespace rclcs
{
    public struct RosTime: IComparable<RosTime>
    {
        private Clock clock;
        public int sec;
        public uint nanosec;

        public RosTime(double seconds, Clock rosClock)
        {
            clock = rosClock;
            sec = 0;
            nanosec = 0;
            SetTimeFromSeconds(seconds);
        }

        public RosTime(Clock rosClock)
        {
            clock = rosClock;
            sec = 0;
            nanosec = 0;
        }

        public void SetTimeFromSeconds(double seconds)
        {
            sec = (int)seconds;
            nanosec = (uint)(seconds*1e9 - sec*1e9);
        }

        public bool IsInThePast
        {
            get
            {
                return this < clock.Now;
            }
        }

        public bool IsInTheFuture
        {
            get
            {
                return this > clock.Now;
            }
        }

        public int CompareTo(RosTime other)
        {
            if ((sec == other.sec) && (nanosec == other.nanosec))
            {
                return 0;
            }
            else if (sec > other.sec)
            {
                return 1; 
            }
            else if ((sec == other.sec) && (nanosec > other.nanosec))
            {
                return 1; 
            }
            else
            {
                return -1;
            }
        }

        public RosTime Delay(double duration)
        {
            RosTime delayedTime = this + clock.CreateRosTime(duration);
            return delayedTime;
        }

        public double Seconds
        {
            get { return sec + nanosec/1e9; }
        }

        public override bool Equals(object obj)
        {
            if (!(obj is RosTime))
            {
                return false;
            }

            var time = (RosTime)obj;
            return EqualityComparer<Clock>.Default.Equals(clock, time.clock) &&
                   sec == time.sec &&
                   nanosec == time.nanosec;
        }

        public override int GetHashCode()
        {
            var hashCode = 1865303823;
            hashCode = hashCode * -1521134295 + EqualityComparer<Clock>.Default.GetHashCode(clock);
            hashCode = hashCode * -1521134295 + sec.GetHashCode();
            hashCode = hashCode * -1521134295 + nanosec.GetHashCode();
            return hashCode;
        }

        public static bool operator > (RosTime a, RosTime b)
        {
            return a.CompareTo(b) == 1;
        }

        public static bool operator < (RosTime a, RosTime b)
        {
            return a.CompareTo(b) == -1;
        }

        public static bool operator >= (RosTime a, RosTime b)
        {
            return a.CompareTo(b) >= 0;
        }

        public static bool operator <= (RosTime a, RosTime b)
        {
            return a.CompareTo(b) <= 0;
        }

        public static bool operator == (RosTime a, RosTime b)
        {
            return a.CompareTo(b) == 0;
        }

        public static bool operator != (RosTime a, RosTime b)
        {
            return a.CompareTo(b) != 0;
        }

        public static RosTime operator + (RosTime a, RosTime b)
        {
            int sec = a.sec + b.sec;
            long nanosec = (long)a.nanosec + (long)b.nanosec;
            if (nanosec >= (long)1e9)
            {
                nanosec -= (long)1e9;
                sec++;
            }

            RosTime result = new RosTime(a.clock)
            {
                sec = sec,
                nanosec = (uint)nanosec
            };

            return result;
        }

        public static RosTime operator - (RosTime a, RosTime b)
        {
            int sec = a.sec - b.sec;
            long nanosec = (long)a.nanosec - (long)b.nanosec;
            if (nanosec < (long)1e9)
            {
                nanosec += (long)1e9;
                sec--;
            }

            RosTime result = new RosTime(a.clock)
            {
                sec = sec,
                nanosec = (uint)nanosec
            };

            return result;
        }
    }


    public class Clock : IDisposable
    {
        private bool disposed;

        internal IntPtr handle;


        public RosTime Now
        {
            get
            {
                RosTime time = new RosTime(this);
                long queryNowNanoseconds = 0;
                NativeMethods.rcl_clock_get_now(handle, ref queryNowNanoseconds);
                time.sec = (int)(queryNowNanoseconds / (long)1e9);
                time.nanosec = (uint)(queryNowNanoseconds - time.sec*((long)1e9));
                return time;
            }
        }

        public Clock()
        {
            rcl_allocator_t allocator = NativeMethods.rcl_get_default_allocator();
            handle = NativeMethods.rclcs_ros_clock_create(ref allocator);
        }

        public RosTime CreateRosTime(double seconds)
        {
            return new RosTime(seconds, this);
        }

        ~Clock()
        {
            Dispose();
        }

        public void Dispose()
        {
            if(!disposed)
            {
                NativeMethods.rclcs_ros_clock_dispose(handle);
                disposed = true;
            }
        }
    }
}
