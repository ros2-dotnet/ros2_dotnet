using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;

namespace rcldotnet
{
public class Subscription<T>
{
  private IntPtr node_handle_;

  public Subscription(IntPtr node_handle)
  {
    node_handle_ = node_handle;
  }
}
}
