/* Copyright 2021-2022 Stefan Hoffmann <stefan.hoffmann@schiller.de>
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

using ROS2;
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
            request.BoolValue = true;
            request.ByteValue = 36;
            request.CharValue = 37;
            request.Float32Value = 38.1f;
            request.Float64Value = 39.1;
            request.Int8Value = 40;
            request.Uint8Value = 41;
            request.Int16Value = 42;
            request.Uint16Value = 43;
            request.Int32Value = 44;
            request.Uint32Value = 45;
            request.Int64Value = 46;
            request.Uint64Value = 47;
            request.StringValue = "one";

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

            Assert.True(serviceReceivedRequest.BoolValue);
            Assert.Equal(36, serviceReceivedRequest.ByteValue);
            Assert.Equal(37, serviceReceivedRequest.CharValue);
            Assert.Equal(38.1f, serviceReceivedRequest.Float32Value);
            Assert.Equal(39.1, serviceReceivedRequest.Float64Value);
            Assert.Equal(40, serviceReceivedRequest.Int8Value);
            Assert.Equal(41, serviceReceivedRequest.Uint8Value);
            Assert.Equal(42, serviceReceivedRequest.Int16Value);
            Assert.Equal(43, serviceReceivedRequest.Uint16Value);
            Assert.Equal(44, serviceReceivedRequest.Int32Value);
            Assert.Equal((uint)45, serviceReceivedRequest.Uint32Value);
            Assert.Equal(46, serviceReceivedRequest.Int64Value);
            Assert.Equal((ulong)47, serviceReceivedRequest.Uint64Value);
            Assert.Equal("one", serviceReceivedRequest.StringValue);

            Assert.True(clientReceivedResponse.BoolValue);
            Assert.Equal(36, clientReceivedResponse.ByteValue);
            Assert.Equal(37, clientReceivedResponse.CharValue);
            Assert.Equal(38.1f, clientReceivedResponse.Float32Value);
            Assert.Equal(39.1, clientReceivedResponse.Float64Value);
            Assert.Equal(40, clientReceivedResponse.Int8Value);
            Assert.Equal(41, clientReceivedResponse.Uint8Value);
            Assert.Equal(42, clientReceivedResponse.Int16Value);
            Assert.Equal(43, clientReceivedResponse.Uint16Value);
            Assert.Equal(144, clientReceivedResponse.Int32Value);
            Assert.Equal((uint)45, clientReceivedResponse.Uint32Value);
            Assert.Equal(46, clientReceivedResponse.Int64Value);
            Assert.Equal((ulong)47, clientReceivedResponse.Uint64Value);
            Assert.Equal("one", clientReceivedResponse.StringValue);

            void HandleRequest(test_msgs.srv.BasicTypes_Request req, test_msgs.srv.BasicTypes_Response response)
            {
                serviceReceivedRequest = req;

                response.BoolValue = req.BoolValue;
                response.ByteValue = req.ByteValue;
                response.CharValue = req.CharValue;
                response.Float32Value = req.Float32Value;
                response.Float64Value = req.Float64Value;
                response.Int8Value = req.Int8Value;
                response.Uint8Value = req.Uint8Value;
                response.Int16Value = req.Int16Value;
                response.Uint16Value = req.Uint16Value;
                response.Int32Value = req.Int32Value + 100;
                response.Uint32Value = req.Uint32Value;
                response.Int64Value = req.Int64Value;
                response.Uint64Value = req.Uint64Value;
                response.StringValue = req.StringValue;
            }
        }
    }
}
