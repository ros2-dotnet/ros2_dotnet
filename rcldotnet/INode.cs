/* Copyright 2018 Esteve Fernandez <esteve@apache.org>
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
using ROS2.Interfaces;

namespace ROS2 {
    public interface INode {
        IList<ISubscriptionBase> Subscriptions { get; }

        IList<IServiceBase> Services { get; }

        IList<Client> Clients { get; }

        IPublisher<T> CreatePublisher<T> (string topic) where T : IMessage;

        ISubscription<T> CreateSubscription<T> (string topic, Action<T> callback) where T : IMessage, new ();

        Service<TService, TRequest, TResponse> CreateService<TService, TRequest, TResponse>(string topic, Action<TRequest, TResponse> callback)
            where TService : IRosServiceDefinition<TRequest, TResponse>
            where TRequest : IMessage, new()
            where TResponse : IMessage, new();

        Client<TService, TRequest, TResponse> CreateClient<TService, TRequest, TResponse>(string serviceName)
            where TService : IRosServiceDefinition<TRequest, TResponse>
            where TRequest : IMessage, new()
            where TResponse : IMessage, new();
    }
}
