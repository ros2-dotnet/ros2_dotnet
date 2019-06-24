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

namespace rclcs
{
	/// <summary>
	/// Represents a managed ROS context
	/// </summary>
	public class Context: IDisposable
	{
        //TODO: Init context when created?

        internal rcl_context_t handle;
        private rcl_allocator_t allocator;

        public bool isInit;
        private bool disposed;

        public Context()
        {
            handle = NativeMethods.rcl_get_zero_initialized_context();
            allocator = NativeMethods.rcl_get_default_allocator();
        }

        public void Init()
        {
            rcl_init_options_t init_options = NativeMethods.rcl_get_zero_initialized_init_options();

            Utils.CheckReturnEnum(NativeMethods.rcl_init_options_init(ref init_options, allocator));
            Utils.CheckReturnEnum(NativeMethods.rcl_init(0, null, ref init_options, ref handle));

            isInit = true;
        }

        public void Shutdown()
        {
            Utils.CheckReturnEnum(NativeMethods.rcl_shutdown(ref handle));
            isInit = false;
        }

        public bool Ok
        {
            get { return NativeMethods.rcl_context_is_valid(ref handle); }
        }

        public void Dispose()
        {
            Dispose(true);
        }

        private void Dispose(bool disposing)
        {
            if (!disposed)
            {
                if (disposing)
                {
                    // Dispose managed resources.
                }

                if(isInit)
                {
                    Shutdown();
                }
                NativeMethods.rcl_context_fini(ref handle);

                disposed = true;
            }
        }

        ~Context()
        {
            Dispose(false);
        }

    }

}

