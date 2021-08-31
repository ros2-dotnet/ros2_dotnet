using System;
using System.Collections.Generic;
using ROS2;
using ROS2.Utils;
using Xunit;

namespace RCLdotnetTests
{
    public sealed class TestServices
    {
        [Fact]
        public void TestServiceAndClient()
        {
            RCLdotnet.Init();
            var serviceNode = RCLdotnet.CreateNode("service_node");
            var clientNode = RCLdotnet.CreateNode("client_node");

            test_msgs.srv.BasicTypes_Request serviceReceivedRequest = null;
            test_msgs.srv.BasicTypes_Response clientReceivedResponse = null;
            
            var service = serviceNode.CreateService<test_msgs.srv.BasicTypes, test_msgs.srv.BasicTypes_Request, test_msgs.srv.BasicTypes_Response>("unittest_dotnet_service", HandleRequest);

            var client = clientNode.CreateClient<test_msgs.srv.BasicTypes, test_msgs.srv.BasicTypes_Request, test_msgs.srv.BasicTypes_Response>("unittest_dotnet_service");
            
            var request = new test_msgs.srv.BasicTypes_Request();
            request.Bool_value = true;
            request.Byte_value = 36;
            request.Char_value = 37;
            request.Float32_value = 38.1f;
            request.Float64_value = 39.1;
            request.Int8_value = 40;
            request.Uint8_value = 41;
            request.Int16_value = 42;
            request.Uint16_value = 43;
            request.Int32_value = 44;
            request.Uint32_value = 45;
            request.Int64_value = 46;
            request.Uint64_value = 47;
            request.String_value = "one";

            var task = client.SendRequestAsync(request);

            while (clientReceivedResponse == null)
            {
                RCLdotnet.SpinOnce(serviceNode, 500);
                RCLdotnet.SpinOnce(clientNode, 500);

                if (task.IsCompleted)
                {
                    clientReceivedResponse = task.Result;
                }
            }

            Assert.True(serviceReceivedRequest.Bool_value);
            Assert.Equal(36, serviceReceivedRequest.Byte_value);
            Assert.Equal(37, serviceReceivedRequest.Char_value);
            Assert.Equal(38.1f, serviceReceivedRequest.Float32_value);
            Assert.Equal(39.1, serviceReceivedRequest.Float64_value);
            Assert.Equal(40, serviceReceivedRequest.Int8_value);
            Assert.Equal(41, serviceReceivedRequest.Uint8_value);
            Assert.Equal(42, serviceReceivedRequest.Int16_value);
            Assert.Equal(43, serviceReceivedRequest.Uint16_value);
            Assert.Equal(44, serviceReceivedRequest.Int32_value);
            Assert.Equal((uint)45, serviceReceivedRequest.Uint32_value);
            Assert.Equal(46, serviceReceivedRequest.Int64_value);
            Assert.Equal((ulong)47, serviceReceivedRequest.Uint64_value);
            Assert.Equal("one", serviceReceivedRequest.String_value);

            Assert.True(clientReceivedResponse.Bool_value);
            Assert.Equal(36, clientReceivedResponse.Byte_value);
            Assert.Equal(37, clientReceivedResponse.Char_value);
            Assert.Equal(38.1f, clientReceivedResponse.Float32_value);
            Assert.Equal(39.1, clientReceivedResponse.Float64_value);
            Assert.Equal(40, clientReceivedResponse.Int8_value);
            Assert.Equal(41, clientReceivedResponse.Uint8_value);
            Assert.Equal(42, clientReceivedResponse.Int16_value);
            Assert.Equal(43, clientReceivedResponse.Uint16_value);
            Assert.Equal(144, clientReceivedResponse.Int32_value);
            Assert.Equal((uint)45, clientReceivedResponse.Uint32_value);
            Assert.Equal(46, clientReceivedResponse.Int64_value);
            Assert.Equal((ulong)47, clientReceivedResponse.Uint64_value);
            Assert.Equal("one", clientReceivedResponse.String_value);

            void HandleRequest(test_msgs.srv.BasicTypes_Request request, test_msgs.srv.BasicTypes_Response response)
            {
                serviceReceivedRequest = request;

                response.Bool_value = request.Bool_value;
                response.Byte_value = request.Byte_value;
                response.Char_value = request.Char_value;
                response.Float32_value = request.Float32_value;
                response.Float64_value = request.Float64_value;
                response.Int8_value = request.Int8_value;
                response.Uint8_value = request.Uint8_value;
                response.Int16_value = request.Int16_value;
                response.Uint16_value = request.Uint16_value;
                response.Int32_value = request.Int32_value + 100;
                response.Uint32_value = request.Uint32_value;
                response.Int64_value = request.Int64_value;
                response.Uint64_value = request.Uint64_value;
                response.String_value = request.String_value;
            }
        }
    }
}
