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

namespace ROS2 {

  /// <summary>
  /// Base class of a Subscription without generic type arguments for use in collections or so.
  /// </summary>
  public abstract class Subscription
  {
    // Only allow internal subclasses.
    internal Subscription()
    {
    }

    // Subscription does intentionaly (for now) not implement IDisposable as this
    // needs some extra consideration how the type works after its
    // internal handle is disposed.
    // By relying on the GC/Finalizer of SafeHandle the handle only gets
    // Disposed if the subscription is not live anymore.
    internal abstract SafeSubscriptionHandle Handle { get; }

    internal abstract IRosMessage CreateMessage();

    internal abstract void TriggerCallback(IRosMessage message);
  }

  public class Subscription<T> : Subscription
    where T : IRosMessage, new() {
    private Action<T> callback_;

    internal Subscription(SafeSubscriptionHandle handle, Action<T> callback) {
      Handle = handle;
      callback_ = callback;
    }

    internal override SafeSubscriptionHandle Handle { get; }

    internal override IRosMessage CreateMessage() => (IRosMessage)new T();

    internal override void TriggerCallback(IRosMessage message) {
      callback_((T) message);
    }
  }
}
