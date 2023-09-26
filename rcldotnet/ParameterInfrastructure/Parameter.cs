/* Copyright 2023 Queensland University of Technology.
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

namespace ROS2.ParameterInfrastructure
{
    public class InvalidTypeException : Exception
    {
        public InvalidTypeException(string message) : base(message)
        {
        }
    }

    public class InvalidGetValueException : Exception
    {
        public InvalidGetValueException(Type parameterType, Type valueType) :
            base($"Unable to retrieve value: {parameterType} does not contain a {valueType}!")
        {
        }
    }

    internal interface ParameterBuilder
    {
        Parameter BuildParameter(ParameterServer server, string name, object defaultValue);
    }

    public abstract class Parameter
    {
        private static readonly IDictionary<Type, byte> _allowedTypes = new Dictionary<Type, byte>();
        private static readonly IDictionary<Type, ParameterBuilder> _parameterBuilders = new Dictionary<Type, ParameterBuilder>();

        private readonly ParameterServer _server;

        protected Parameter(ParameterServer server, string name)
        {
            _server = server;
            Name = name;
        }

        internal abstract byte Type { get; }

        public string Name { get; }

        protected void OnValueChanged()
        {
            _server.OnParameterValueChanged(this);
        }

        public static bool IsOfValidType(Type type)
        {
            return _allowedTypes.ContainsKey(type);
        }

        internal static bool TryGetTypeIndex(Type type, out byte index)
        {
            return _allowedTypes.TryGetValue(type, out index);
        }

        internal static void RegisterBuilder<T>(byte typeIndex, ParameterBuilder builder) where T : struct
        {
            Type type = typeof(T);

            _allowedTypes.Add(type, typeIndex);
            _parameterBuilders.Add(type, builder);
        }

        internal static Parameter BuildParameter<T>(ParameterServer server, string name, T defaultValue) where T : struct
        {
            if (!_parameterBuilders.TryGetValue(typeof(T), out ParameterBuilder builder))
            {
                throw new InvalidTypeException($"Unable to build parameter for type: {nameof(T)}. Invalid type!");
            }

            return builder.BuildParameter(server, name, defaultValue);
        }

        public abstract T GetValue<T>() where T : struct;
    }

    public abstract class ParameterTyped<ParameterT> : Parameter where ParameterT : struct
    {
        private ParameterT? _value;

        public ParameterT? Value
        {
            get => _value;
            set
            {
                if (_value.Equals(value)) return;

                _value = value;

                OnValueChanged();
            }
        }

        protected readonly ParameterT _defaultValue;

        protected ParameterTyped(ParameterServer server, string name, ParameterT defaultValue) : base(server, name)
        {
            _defaultValue = defaultValue;
        }

        public bool IsSet() => _value.HasValue;

        public override T GetValue<T>()
        {
            if (Value == null && _defaultValue is T outDefaultValue) return outDefaultValue;

            if (Value is T outValue) return outValue;

            throw new InvalidGetValueException(typeof(ParameterTyped<ParameterT>), typeof(T));
        }
    }
}
