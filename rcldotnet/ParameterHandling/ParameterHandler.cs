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

        private readonly IDictionary<string, Parameter> _parameters;

        // TODO: Implement parameter event publishing
        private Publisher<ParameterEvent> _publisherEvent;

        private Action<List<Parameter>> _onSetParameterCallback;

        internal ParameterHandler(Node node)
        {
            _parameters = new Dictionary<string, Parameter>();

            _publisherEvent = node.CreatePublisher<ParameterEvent>("/parameter_events");

            node.CreateService<DescribeParameters, DescribeParameters_Request, DescribeParameters_Response>("~/describe_parameters", OnDescribeParameter);
            node.CreateService<GetParameterTypes, GetParameterTypes_Request, GetParameterTypes_Response>("~/get_parameter_types", OnGetParameterTypes);
            node.CreateService<GetParameters, GetParameters_Request, GetParameters_Response>("~/get_parameters", OnGetParameters);
            node.CreateService<ListParameters, ListParameters_Request, ListParameters_Response>("~/list_parameters", OnListParameters);
            node.CreateService<SetParameters, SetParameters_Request, SetParameters_Response>("~/set_parameters", OnSetParameters);
            node.CreateService<SetParametersAtomically, SetParametersAtomically_Request, SetParametersAtomically_Response>("~/set_parameters_atomically", OnSetParametersAtomically);
        }

        public void AddOnSetParameterCallback(Action<List<Parameter>> callback)
        {
            _onSetParameterCallback += callback;
        }

        public void RemoveOnSetParameterCallback(Action<List<Parameter>> callback)
        {
            _onSetParameterCallback -= callback;
        }

        private Parameter DeclareParameter(string name, Type type)
        {
            if (!_typeToParameterType.TryGetValue(type, out byte typeCode))
            {
                throw new InvalidParameterTypeException(type);
            }

            if (_parameters.TryGetValue(name, out Parameter parameter))
            {
                if (parameter.Value.Type != typeCode)
                {
                    throw new ParameterTypeMismatchException(
                        $"Attempted to redefine parameter \"{name}\" from type {parameter.Value.Type} to {typeCode}!");
                }

                // TODO: Consider updating description

                return parameter;
            }

            Parameter declaredParameter = new Parameter { Name = name, Value = { Type = typeCode } };
            _parameters.Add(name, declaredParameter);
            return declaredParameter;
        }

        public void DeclareParameter(string name, bool defaultValue = false)
        {
            DeclareParameter(name, typeof(bool)).Value.BoolValue = defaultValue;
        }

        public void DeclareParameter(string name, int defaultValue = 0) => DeclareParameter(name, (long)defaultValue);

        public void DeclareParameter(string name, long defaultValue = 0L)
        {
            DeclareParameter(name, typeof(long)).Value.IntegerValue = defaultValue;
        }

        public void DeclareParameter(string name, float defaultValue = 0.0f) => DeclareParameter(name, (double)defaultValue);

        public void DeclareParameter(string name, double defaultValue = 0.0)
        {
            DeclareParameter(name, typeof(double)).Value.DoubleValue = defaultValue;
        }

        public void DeclareParameter(string name, string defaultValue = "")
        {
            DeclareParameter(name, typeof(string)).Value.StringValue = defaultValue;
        }

        public void DeclareParameter(string name, List<byte> defaultValue = null)
        {
            DeclareParameter(name, typeof(List<byte>)).Value.ByteArrayValue = defaultValue;
        }

        public void DeclareParameter(string name, List<bool> defaultValue = null)
        {
            DeclareParameter(name, typeof(List<bool>)).Value.BoolArrayValue = defaultValue;
        }

        public void DeclareParameter(string name, List<long> defaultValue = null)
        {
            DeclareParameter(name, typeof(List<long>)).Value.IntegerArrayValue = defaultValue;
        }

        public void DeclareParameter(string name, List<double> defaultValue = null)
        {
            DeclareParameter(name, typeof(List<double>)).Value.DoubleArrayValue = defaultValue;
        }

        public void DeclareParameter(string name, List<string> defaultValue = null)
        {
            DeclareParameter(name, typeof(List<string>)).Value.StringArrayValue = defaultValue;
        }

        private void OnDescribeParameter(DescribeParameters_Request request, DescribeParameters_Response response)
        {
            // TODO: Implement parameter descriptions.
        }

        private void OnGetParameterTypes(GetParameterTypes_Request request, GetParameterTypes_Response response)
        {
            foreach (Parameter parameter in _parameters.Values)
            {
                response.Types.Add(parameter.Value.Type);
            }
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

        private void OnGetParameters(GetParameters_Request request, GetParameters_Response response)
        {
            response.Values.AddRange(GetParameters(request.Names));
        }

        public ParameterValue GetParameter(string name)
        {
            if (_parameters.TryGetValue(name, out Parameter parameter))
            {
                return CloneParameterValue(parameter.Value);
            }

            throw new ParameterNotDeclaredException(name);
        }

        private void OnListParameters(ListParameters_Request request, ListParameters_Response response)
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

        private SetParametersResult CheckParameterCompatibility(Parameter update)
        {
            SetParametersResult result = new SetParametersResult();
            if (!_parameters.TryGetValue(update.Name, out Parameter parameter))
            {
                result.Successful = false;
                result.Reason = "Parameter was not declared!";
            }
            else if (update.Value.Type != parameter.Value.Type)
            {
                result.Successful = false;
                result.Reason = $"Parameter type mismatch: {parameter.Value.Type} != {update.Value.Type}!";
            }
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

        public List<SetParametersResult> SetParameters(List<Parameter> parameters)
        {
            List<SetParametersResult> results = new List<SetParametersResult>();

            foreach (Parameter source in parameters)
            {
                results.Add(SetParametersAtomically(new List<Parameter> { source }));
            }

            return results;
        }

        private void OnSetParameters(SetParameters_Request request, SetParameters_Response response)
        {
            response.Results.AddRange(SetParameters(request.Parameters));
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

            _onSetParameterCallback?.Invoke(parameters);

            return result;
        }

        private void OnSetParametersAtomically(SetParametersAtomically_Request request, SetParametersAtomically_Response response)
        {
            response.Result = SetParametersAtomically(request.Parameters);
        }
    }
}
