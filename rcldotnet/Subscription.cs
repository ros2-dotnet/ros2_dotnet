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
