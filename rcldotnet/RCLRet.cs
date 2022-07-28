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

namespace ROS2
{
    internal enum RCLRet
    {
        Ok = 0,
        Error = 1,
        Timeout = 2,
        Unsupported = 3,
        BadAlloc = 10,
        InvalidArgument = 11,
        AlreadyInit = 100,
        NotInit = 101,
        MismatchedRmwId = 102,
        TopicNameInvalid = 103,
        ServiceNameInvalid = 104,
        UnknownSubstitution = 105,
        AlreadyShutdown = 106,
        NodeInvalid = 200,
        NodeInvalidName = 201,
        NodeInvalidNamespace = 202,
        PublisherInvalid = 300,
        SubscriptionInvalid = 400,
        SubscriptionTakeFailed = 401,
        ClientInvalid = 500,
        ClientTakeFailed = 501,
        ServiceInvalid = 600,
        ServiceTakeFailed = 601,
        TimerInvalid = 800,
        TimerCanceled = 801,
        WaitSetInvalid = 900,
        WaitSetEmpty = 901,
        WaitSetFull = 902,
        InvalidRemapRule = 1001,
        WrongLexme = 1002,
        InvalidParamRule = 1010,
        InvalidLogLevelRule = 1020,
        EventInvalid = 2000,
        EventTakeFailed = 2001,
        ActionClientTakeFailed = 2103,
        ActionServerTakeFailed = 2201,
    }
}
