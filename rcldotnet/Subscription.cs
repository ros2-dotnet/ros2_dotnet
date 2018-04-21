/* Copyright 2016-2018 Esteve Fernandez <esteve@apache.org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

using ROS2.Interfaces;
using ROS2.Utils;

namespace ROS2 {
  public class Subscription<T> : ISubscription<T>
  where T : IMessage, new() {
    private IntPtr subscription_handle_;
    private Action<T> callback_;

    public Subscription (IntPtr subscription_handle, Action<T> callback) {
      subscription_handle_ = subscription_handle;
      callback_ = callback;
    }

    public IntPtr Handle { get { return subscription_handle_; } }

    public IMessage CreateMessage () {
      IMessage msg = (IMessage) new T ();
      return msg;
    }

    public void TriggerCallback (IMessage message) {
      callback_ ((T) message);
    }
  }
}
