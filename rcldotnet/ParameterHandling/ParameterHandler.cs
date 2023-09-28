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
using rcl_interfaces.msg;
using rcl_interfaces.srv;
using ROS2.ParameterHandling.Exceptions;

namespace ROS2
{
    public class ParameterHandler
    {
        private static readonly IDictionary<Type, byte> _typeToParameterType = new Dictionary<Type, byte>
        {
            {typeof(bool), ParameterType.PARAMETER_BOOL},
            {typeof(long), ParameterType.PARAMETER_INTEGER},
            {typeof(double), ParameterType.PARAMETER_DOUBLE},
            {typeof(string), ParameterType.PARAMETER_STRING},
            {typeof(List<byte>), ParameterType.PARAMETER_BYTE_ARRAY},
            {typeof(List<bool>), ParameterType.PARAMETER_BOOL_ARRAY},
            {typeof(List<long>), ParameterType.PARAMETER_INTEGER_ARRAY},
            {typeof(List<double>), ParameterType.PARAMETER_DOUBLE_ARRAY},
            {typeof(List<string>), ParameterType.PARAMETER_STRING_ARRAY}
        };

        private readonly Node _node;
        private readonly IDictionary<string, Parameter> _parameters = new Dictionary<string, Parameter>();
        private readonly IDictionary<string, ParameterDescriptor> _descriptors = new Dictionary<string, ParameterDescriptor>();

        private readonly Publisher<ParameterEvent> _publisherEvent;

        private Action<List<Parameter>> _onSetParameterCallback;

        internal ParameterHandler(Node node)
        {
            _node = node;
            _publisherEvent = node.CreatePublisher<ParameterEvent>("/parameter_events", QosProfile.ParameterEventsProfile);

            node.CreateService<DescribeParameters, DescribeParameters_Request, DescribeParameters_Response>("~/describe_parameters", OnDescribeParametersServiceRequest);
            node.CreateService<GetParameterTypes, GetParameterTypes_Request, GetParameterTypes_Response>("~/get_parameter_types", OnGetParameterTypesServiceRequest);
            node.CreateService<GetParameters, GetParameters_Request, GetParameters_Response>("~/get_parameters", OnGetParametersServiceRequest);
            node.CreateService<ListParameters, ListParameters_Request, ListParameters_Response>("~/list_parameters", OnListParametersServiceRequest);
            node.CreateService<SetParameters, SetParameters_Request, SetParameters_Response>("~/set_parameters", OnSetParametersServiceRequest);
            node.CreateService<SetParametersAtomically, SetParametersAtomically_Request, SetParametersAtomically_Response>("~/set_parameters_atomically", OnSetParametersAtomicallyServiceRequest);
        }

        #region Service Request Handlers

        private void OnDescribeParametersServiceRequest(DescribeParameters_Request request, DescribeParameters_Response response)
        {
            foreach (string name in request.Names)
            {
                response.Descriptors.Add(
                    _descriptors.TryGetValue(name, out ParameterDescriptor descriptor)
                        ? descriptor
                        : new ParameterDescriptor());
            }
        }

        private void OnGetParameterTypesServiceRequest(GetParameterTypes_Request request, GetParameterTypes_Response response)
        {
            foreach (Parameter parameter in _parameters.Values)
            {
                response.Types.Add(parameter.Value.Type);
            }
        }

        private void OnGetParametersServiceRequest(GetParameters_Request request, GetParameters_Response response)
        {
            response.Values.AddRange(GetParameters(request.Names));
        }

        private void OnListParametersServiceRequest(ListParameters_Request request, ListParameters_Response response)
        {
            bool hasPrefixes = request.Prefixes.Count != 0;
            foreach (Parameter parameter in _parameters.Values)
            {
                bool matchesCriteria = !hasPrefixes;

                if (hasPrefixes)
                {
                    foreach (string prefix in request.Prefixes)
                    {
                        if (parameter.Name.StartsWith(prefix))
                        {
                            matchesCriteria = true;
                            break;
                        }
                    }
                }

                if (matchesCriteria) response.Result.Names.Add(parameter.Name);
            }
        }

        private void OnSetParametersServiceRequest(SetParameters_Request request, SetParameters_Response response)
        {
            response.Results.AddRange(SetParameters(request.Parameters));
        }

        private void OnSetParametersAtomicallyServiceRequest(SetParametersAtomically_Request request, SetParametersAtomically_Response response)
        {
            response.Result = SetParametersAtomically(request.Parameters);
        }

        #endregion

        public void AddOnSetParameterCallback(Action<List<Parameter>> callback)
        {
            _onSetParameterCallback += callback;
        }

        public void RemoveOnSetParameterCallback(Action<List<Parameter>> callback)
        {
            _onSetParameterCallback -= callback;
        }

        private ParameterEvent GenerateParameterEventMessage()
        {
            return new ParameterEvent
            {
                Node = $"{_node.GetNamespace()}{_node.GetName()}",
                Stamp = _node.Clock.Now()
            };
        }

        private void PublishParametersDeclaredEvent(IEnumerable<Parameter> parameters)
        {
            ParameterEvent parameterEvent = GenerateParameterEventMessage();
            parameterEvent.NewParameters.AddRange(parameters);
            _publisherEvent.Publish(parameterEvent);
        }

        private void PublishParametersChangedEvent(IEnumerable<Parameter> parameters)
        {
            ParameterEvent parameterEvent = GenerateParameterEventMessage();
            parameterEvent.ChangedParameters.AddRange(parameters);
            _publisherEvent.Publish(parameterEvent);
        }

        private void PublishParametersDeletedEvent(IEnumerable<Parameter> parameters)
        {
            ParameterEvent parameterEvent = GenerateParameterEventMessage();
            parameterEvent.DeletedParameters.AddRange(parameters);
            _publisherEvent.Publish(parameterEvent);
        }

        private void DeclareParameter(string name, Type type, Action<ParameterValue> assignDefaultCallback, ParameterDescriptor descriptor = null)
        {
            if (!_typeToParameterType.TryGetValue(type, out byte typeCode))
            {
                throw new InvalidParameterTypeException(type);
            }

            if (descriptor == null)
            {
                descriptor = new ParameterDescriptor
                {
                    Name = name,
                    Type = typeCode
                };
            }

            if (_parameters.TryGetValue(name, out Parameter parameter))
            {

                if (parameter.Value.Type != typeCode)
                {
                    throw new ParameterTypeMismatchException(
                        $"Attempted to redefine parameter \"{name}\" from type {parameter.Value.Type} to {typeCode}!");
                }

                // TODO: Should we update the description if it doesn't match or throw an error?
                return;
            }

            Parameter declaredParameter = new Parameter { Name = name, Value = { Type = typeCode } };
            _parameters.Add(name, declaredParameter);
            _descriptors.Add(name, descriptor);

            assignDefaultCallback?.Invoke(declaredParameter.Value);

            PublishParametersDeclaredEvent(new List<Parameter> { declaredParameter });
        }

        public void DeclareParameter(string name, bool defaultValue = false, ParameterDescriptor descriptor = null)
        {
            DeclareParameter(name, typeof(bool), value => { value.BoolValue = defaultValue; }, descriptor);
        }

        public void DeclareParameter(string name, int defaultValue = 0, ParameterDescriptor descriptor = null) => DeclareParameter(name, (long)defaultValue, descriptor);

        public void DeclareParameter(string name, long defaultValue = 0L, ParameterDescriptor descriptor = null)
        {
            DeclareParameter(name, typeof(long), value => { value.IntegerValue = defaultValue; }, descriptor);
        }

        public void DeclareParameter(string name, float defaultValue = 0.0f, ParameterDescriptor descriptor = null) => DeclareParameter(name, (double)defaultValue, descriptor);

        public void DeclareParameter(string name, double defaultValue = 0.0, ParameterDescriptor descriptor = null)
        {
            DeclareParameter(name, typeof(double), value => { value.DoubleValue = defaultValue; }, descriptor);
        }

        public void DeclareParameter(string name, string defaultValue = "", ParameterDescriptor descriptor = null)
        {
            DeclareParameter(name, typeof(string), value => { value.StringValue = defaultValue; }, descriptor);
        }

        public void DeclareParameter(string name, IEnumerable<byte> defaultValue = null, ParameterDescriptor descriptor = null)
        {
            DeclareParameter(name, typeof(List<byte>), value =>
            {
                if (defaultValue != null) value.ByteArrayValue.AddRange(defaultValue);
            }, descriptor);
        }

        public void DeclareParameter(string name, IEnumerable<bool> defaultValue = null, ParameterDescriptor descriptor = null)
        {
            DeclareParameter(name, typeof(List<bool>), value =>
            {
                if (defaultValue != null) value.BoolArrayValue.AddRange(defaultValue);
            }, descriptor);
        }

        public void DeclareParameter(string name, IEnumerable<long> defaultValue = null, ParameterDescriptor descriptor = null)
        {
            DeclareParameter(name, typeof(List<long>), value =>
            {
                if (defaultValue != null) value.IntegerArrayValue.AddRange(defaultValue);
            }, descriptor);
        }

        public void DeclareParameter(string name, IEnumerable<double> defaultValue = null, ParameterDescriptor descriptor = null)
        {
            DeclareParameter(name, typeof(List<double>), value =>
            {
                if (defaultValue != null) value.DoubleArrayValue.AddRange(defaultValue);
            }, descriptor);
        }

        public void DeclareParameter(string name, IEnumerable<string> defaultValue = null, ParameterDescriptor descriptor = null)
        {
            DeclareParameter(name, typeof(List<string>), value =>
            {
                if (defaultValue != null) value.StringArrayValue.AddRange(defaultValue);
            }, descriptor);
        }

        public void UndeclareParameter(string name)
        {
            if (!_descriptors.TryGetValue(name, out ParameterDescriptor descriptor))
            {
                throw new ParameterNotDeclaredException(name);
            }

            if (descriptor.ReadOnly) throw new ParameterImmutableException(name);

            Parameter parameter = _parameters[name];

            _parameters.Remove(name);
            _descriptors.Remove(name);

            PublishParametersDeletedEvent(new List<Parameter> { parameter });
        }

        private ParameterValue CloneParameterValue(ParameterValue toClone)
        {
            byte type = toClone.Type;
            ParameterValue clone = new ParameterValue
            {
                Type = type
            };

            switch (type)
            {
                case ParameterType.PARAMETER_BOOL:
                    clone.BoolValue = toClone.BoolValue;
                    break;
                case ParameterType.PARAMETER_INTEGER:
                    clone.IntegerValue = toClone.IntegerValue;
                    break;
                case ParameterType.PARAMETER_DOUBLE:
                    clone.DoubleValue = toClone.DoubleValue;
                    break;
                case ParameterType.PARAMETER_STRING:
                    clone.StringValue = toClone.StringValue;
                    break;
                case ParameterType.PARAMETER_BYTE_ARRAY:
                    clone.ByteArrayValue.AddRange(toClone.ByteArrayValue);
                    break;
                case ParameterType.PARAMETER_BOOL_ARRAY:
                    clone.BoolArrayValue.AddRange(toClone.BoolArrayValue);
                    break;
                case ParameterType.PARAMETER_INTEGER_ARRAY:
                    clone.IntegerArrayValue.AddRange(toClone.IntegerArrayValue);
                    break;
                case ParameterType.PARAMETER_DOUBLE_ARRAY:
                    clone.DoubleArrayValue.AddRange(toClone.DoubleArrayValue);
                    break;
                case ParameterType.PARAMETER_STRING_ARRAY:
                    clone.StringArrayValue.AddRange(toClone.StringArrayValue);
                    break;
                default:
                    throw new InvalidParameterTypeException(type);
            }

            return clone;
        }

        public ParameterValue GetParameter(string name)
        {
            if (_parameters.TryGetValue(name, out Parameter parameter))
            {
                return CloneParameterValue(parameter.Value);
            }

            throw new ParameterNotDeclaredException(name);
        }

        public List<ParameterValue> GetParameters(IEnumerable<string> names)
        {
            List<ParameterValue> results = new List<ParameterValue>();

            foreach (string parameterName in names)
            {
                if (_parameters.TryGetValue(parameterName, out Parameter parameter))
                {
                    results.Add(CloneParameterValue(parameter.Value));
                }
            }

            return results;
        }

        private SetParametersResult CheckParameterCompatibility(Parameter update)
        {
            SetParametersResult result = new SetParametersResult();
            if (!_descriptors.TryGetValue(update.Name, out ParameterDescriptor descriptor))
            {
                result.Successful = false;
                result.Reason = "Parameter was not declared!";
            }
            else if (descriptor.ReadOnly)
            {
                result.Successful = false;
                result.Reason = "Parameter is read-only!";
            }
            else if (update.Value.Type != descriptor.Type)
            {
                result.Successful = false;
                result.Reason = $"Parameter type mismatch: {descriptor.Type} != {update.Value.Type}!";
            }
            // TODO: Check value compatibility against ParameterDescriptor constraints.
            else
            {
                result.Successful = true;
            }

            return result;
        }

        private void UpdateParameter(Parameter source)
        {
            Parameter target = _parameters[source.Name];

            switch (source.Value.Type)
            {
                case ParameterType.PARAMETER_BOOL:
                    target.Value.BoolValue = source.Value.BoolValue;
                    break;
                case ParameterType.PARAMETER_INTEGER:
                    target.Value.IntegerValue = source.Value.IntegerValue;
                    break;
                case ParameterType.PARAMETER_DOUBLE:
                    target.Value.DoubleValue = source.Value.DoubleValue;
                    break;
                case ParameterType.PARAMETER_STRING:
                    target.Value.StringValue = source.Value.StringValue;
                    break;
                case ParameterType.PARAMETER_BYTE_ARRAY:
                    target.Value.ByteArrayValue = source.Value.ByteArrayValue;
                    break;
                case ParameterType.PARAMETER_BOOL_ARRAY:
                    target.Value.BoolArrayValue = source.Value.BoolArrayValue;
                    break;
                case ParameterType.PARAMETER_INTEGER_ARRAY:
                    target.Value.IntegerArrayValue = source.Value.IntegerArrayValue;
                    break;
                case ParameterType.PARAMETER_DOUBLE_ARRAY:
                    target.Value.DoubleArrayValue = source.Value.DoubleArrayValue;
                    break;
                case ParameterType.PARAMETER_STRING_ARRAY:
                    target.Value.StringArrayValue = source.Value.StringArrayValue;
                    break;
                default:
                    throw new InvalidParameterTypeException(source.Value.Type);
            }
        }

        public SetParametersResult SetParameter(Parameter parameter)
        {
            return SetParametersAtomically(new List<Parameter> { parameter });
        }

        public List<SetParametersResult> SetParameters(List<Parameter> parameters)
        {
            List<SetParametersResult> results = new List<SetParametersResult>();

            foreach (Parameter source in parameters)
            {
                results.Add(SetParametersAtomically(new List<Parameter> { source }));
            }

            return results;
        }

        public SetParametersResult SetParametersAtomically(List<Parameter> parameters)
        {
            SetParametersResult result = new SetParametersResult();

            foreach (Parameter source in parameters)
            {
                result = CheckParameterCompatibility(source);
                if (!result.Successful) break;
            }

            if (!result.Successful) return result;

            foreach (Parameter source in parameters)
            {
                UpdateParameter(source);
            }

            PublishParametersChangedEvent(parameters);
            _onSetParameterCallback?.Invoke(parameters);

            return result;
        }

        public bool HasParameter(string name) => _parameters.ContainsKey(name);
    }
}
