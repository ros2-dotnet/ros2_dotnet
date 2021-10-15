/*
MIT License

Copyright (c) 2021 Microsoft

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using ROS2.Common;
using ROS2.Interfaces;
using ROS2.Utils;

namespace ROS2 {
  internal class ClientDelegates {
    internal static readonly DllLoadUtils dllLoadUtils;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate int NativeRCLSendRequestType (
      IntPtr clientHandle, IntPtr messageHandle, ref long sequenceNumber);

    internal static NativeRCLSendRequestType native_rcl_send_request = null;

    internal delegate int NativeRCLTakeResponseType (
      IntPtr clientHandle, IntPtr messageHandle);
    internal static NativeRCLTakeResponseType native_rcl_take_response = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate int NativeRCLServerAvailableType (
      IntPtr clientHandle, IntPtr nodeHandle, ref bool isAvailable);

    internal static NativeRCLServerAvailableType native_rcl_service_server_is_available = null;

    static ClientDelegates () {
      dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils ();
      IntPtr nativelibrary = dllLoadUtils.LoadLibrary ("rcldotnet_client");
      IntPtr native_rcl_send_ptr = dllLoadUtils.GetProcAddress (nativelibrary, "native_rcl_send_request");
      ClientDelegates.native_rcl_send_request = (NativeRCLSendRequestType) Marshal.GetDelegateForFunctionPointer (
        native_rcl_send_ptr, typeof (NativeRCLSendRequestType));
      IntPtr native_rcl_take_ptr = dllLoadUtils.GetProcAddress (nativelibrary, "native_rcl_take_response");
      ClientDelegates.native_rcl_take_response = (NativeRCLTakeResponseType) Marshal.GetDelegateForFunctionPointer (
        native_rcl_take_ptr, typeof (NativeRCLTakeResponseType));
      IntPtr native_rcl_service_server_is_available_ptr = dllLoadUtils.GetProcAddress (nativelibrary, "native_rcl_service_server_is_available");
      ClientDelegates.native_rcl_service_server_is_available = (NativeRCLServerAvailableType) Marshal.GetDelegateForFunctionPointer (
        native_rcl_service_server_is_available_ptr, typeof (NativeRCLServerAvailableType));
    }
  }

  public class Client<TRequest, TResponse> : IClient<TRequest, TResponse>
    where TRequest : IMessage, new()
    where TResponse : IMessage, new()
  {
      public Client (IntPtr clientHandle, IntPtr nodeHandle) {
        Handle = clientHandle;
        NodeHandle = nodeHandle;
      }

      public IntPtr Handle { get; }

      public IntPtr NodeHandle { get; }

      public async Task<bool> WaitForServiceAsync(int timeoutSeconds)
      {
        int maxTries = timeoutSeconds * 4;
        int iteration = 0;
        while (iteration < maxTries)
        {
          iteration++;
          bool isReady = false;
          RCLRet ret = (RCLRet) ClientDelegates.native_rcl_service_server_is_available(Handle, NodeHandle, ref isReady);
          if (ret != RCLRet.Ok)
          {
            // TODO: SHS: log the error
            RCLdotnet.ResetRclError();
            return false;
          }

          if (isReady)
          {
            return true;
          }

          await Task.Delay(250);
        }

        return false;
      }

      private async Task<bool> WaitForClientToBeReadyAsync(int maxTries, int periodMillis)
      {
        IntPtr waitSetHandle = RCLdotnet.GetZeroInitializedWaitSet ();
        if (!RCLdotnet.WaitSetInit (waitSetHandle, 0, 0, 0, 1, 0, 0))
        {
          // TODO: SHS: Log error
          return false;
        }

        int iteration = 0;
        while (iteration < maxTries)
        {
          iteration++;

          RCLdotnet.WaitSetClear (waitSetHandle);

          RCLdotnet.WaitSetAddClient (waitSetHandle, Handle);

          RCLRet ret = (RCLRet) RCLdotnet.Wait (waitSetHandle, periodMillis);
          if (ret == RCLRet.Timeout)
          {
            continue;
          }

          if (ret != RCLRet.Ok)
          {
            // TODO: SHS: Log this somewhere.
            RCLdotnet.ResetRclError();
            RCLdotnet.DestroyWaitSet (waitSetHandle);
            return false;
          }

          RCLdotnet.DestroyWaitSet (waitSetHandle);
          return true;
        }

        RCLdotnet.DestroyWaitSet (waitSetHandle);
        return false;
      }

      public async Task<TResponse> SendRequestAsync (TRequest request) {
        IntPtr requestHandle = request._CREATE_NATIVE_MESSAGE ();

        request._WRITE_HANDLE (requestHandle);

        long sequenceNumber = 0;
        RCLRet ret = (RCLRet) ClientDelegates.native_rcl_send_request (Handle, requestHandle, ref sequenceNumber);
        if (ret != RCLRet.Ok)
        {
          throw new RCLException();
        }

        request._DESTROY_NATIVE_MESSAGE (requestHandle);

        if (! await WaitForClientToBeReadyAsync (30, 100))
        {
          throw new RCLException("Client did not receive response from server to the request");
        }

        TResponse response = new TResponse();
        IntPtr responseHandle = response._CREATE_NATIVE_MESSAGE ();

        ret = (RCLRet) ClientDelegates.native_rcl_take_response (Handle, responseHandle);
        if (ret != RCLRet.Ok)
        {
          throw new RCLException();
        }

        response._READ_HANDLE(responseHandle);

        response._DESTROY_NATIVE_MESSAGE(responseHandle);

        return response;
      }
    }
}
