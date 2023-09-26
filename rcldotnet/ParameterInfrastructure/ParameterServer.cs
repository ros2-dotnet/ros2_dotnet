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

using System.Collections.Generic;
using rcl_interfaces.msg;
using rcl_interfaces.srv;

namespace ROS2.ParameterInfrastructure
{
    public class ParameterServer
    {
        private Node _node;
        private IDictionary<string, Parameter> _parameters;

        private Publisher<ParameterEvent> _publisherEvent;
        private Service<DescribeParameters, DescribeParameters_Request, DescribeParameters_Response> _serviceDescribeParameters;
        private Service<GetParameterTypes, GetParameterTypes_Request, GetParameterTypes_Response> _serviceGetParameterTypes;
        private Service<GetParameters, GetParameters_Request, GetParameters_Response> _serviceGetParameters;
        private Service<ListParameters, ListParameters_Request, ListParameters_Response> _serviceListParameters;
        private Service<SetParameters, SetParameters_Request, SetParameters_Response> _serviceSetParameters;
        private Service<SetParametersAtomically, SetParametersAtomically_Request, SetParametersAtomically_Response> _serviceSetParametersAtomically;

        static ParameterServer()
        {
            Parameter.RegisterBuilder<bool>(ParameterType.PARAMETER_BOOL, new ParameterBuilderBool());
        }

        internal ParameterServer(Node node)
        {
            _node = node;
            _parameters = new Dictionary<string, Parameter>();

            _publisherEvent = _node.CreatePublisher<ParameterEvent>("/parameter_events");
            _serviceDescribeParameters = _node.CreateService<DescribeParameters, DescribeParameters_Request, DescribeParameters_Response>("~/describe_parameters", OnDescribeParameter);
            _serviceGetParameterTypes = _node.CreateService<GetParameterTypes, GetParameterTypes_Request, GetParameterTypes_Response>("~/get_parameter_types", OnGetParameterTypes);
            _serviceGetParameters = _node.CreateService<GetParameters, GetParameters_Request, GetParameters_Response>("~/get_parameter", OnGetParameters);
            _serviceListParameters = _node.CreateService<ListParameters, ListParameters_Request, ListParameters_Response>("~/list_parameters", OnListParameters);
            _serviceSetParameters = _node.CreateService<SetParameters, SetParameters_Request, SetParameters_Response>("~/set_parameters", OnSetParameters);
            _serviceSetParametersAtomically = _node.CreateService<SetParametersAtomically, SetParametersAtomically_Request, SetParametersAtomically_Response>("~/set_parameters_atomically", OnSetParametersAtomically);

            DeclareParameter("use_sim_time", false);
        }

        internal void OnParameterValueChanged(Parameter parameter)
        {
        }

        public bool DeclareParameter<ParameterT>(string name, ParameterT defaultValue = default) where ParameterT : struct
        {
            if (!Parameter.IsOfValidType(typeof(ParameterT))) return false;

            if (_parameters.TryGetValue(name, out Parameter existingParameter))
            {
                Parameter.TryGetTypeIndex(typeof(ParameterT), out byte type);

                return existingParameter.Type == type;
            }

            _parameters.Add(name, Parameter.BuildParameter(this, name, defaultValue));
            return true;
        }

        public void SetParameter<ParameterT>(string name, ParameterT value)
        {

        }

        private void OnDescribeParameter(DescribeParameters_Request request, DescribeParameters_Response response)
        {

        }

        private void OnGetParameterTypes(GetParameterTypes_Request request, GetParameterTypes_Response response)
        {

        }

        private void OnGetParameters(GetParameters_Request request, GetParameters_Response response)
        {

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

        private void OnSetParameters(SetParameters_Request request, SetParameters_Response response)
        {

        }

        private void OnSetParametersAtomically(SetParametersAtomically_Request request, SetParametersAtomically_Response response)
        {

        }
    }
}
