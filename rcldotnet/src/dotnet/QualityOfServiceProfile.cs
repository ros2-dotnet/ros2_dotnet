/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0 (the "License");

You may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace rclcs
{
    public enum QosProfiles
    {
       SENSOR_DATA,
       PARAMETERS,
       DEFAULT,
       SERVICES_DEFAULT,
       PARAMETER_EVENTS,
       SYSTEM_DEFAULT
    }

    public class QualityOfServiceProfile : IDisposable
    {
        private bool disposed;

        internal rmw_qos_profile_t handle;

        public QosProfiles Profile;

        public QualityOfServiceProfile(QosProfiles profile)
        {
            handle = new rmw_qos_profile_t();
            SetProfile(profile);
        }

        public void SetProfile(QosProfiles profile)
        {
            Profile = profile;
            switch(profile)
            {
                case QosProfiles.SENSOR_DATA:
                    SetProfileSensorData();
                    break;
                case QosProfiles.PARAMETERS:
                    SetProfileParameters();
                    break;
                case QosProfiles.DEFAULT:
                    SetProfileDefault();
                    break;
                case QosProfiles.SERVICES_DEFAULT:
                    SetProfileServicesDefault();
                    break;
                case QosProfiles.PARAMETER_EVENTS:
                    SetProfileParameterEvents();
                    break;
                case QosProfiles.SYSTEM_DEFAULT:
                    SetProfileSystemDefault();
                    break;
                default:
                    break;
            }
            SetProfileDefault();
        }

        private void SetProfileSensorData()
        {
            handle.history = rmw_qos_history_policy_t.RMW_QOS_POLICY_HISTORY_KEEP_LAST;
            handle.depth = 5;
            handle.reliability = rmw_qos_reliability_policy_t.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
            handle.durability = rmw_qos_durability_policy_t.RMW_QOS_POLICY_DURABILITY_VOLATILE;
        }

        private void SetProfileParameters()
        {
            handle.history = rmw_qos_history_policy_t.RMW_QOS_POLICY_HISTORY_KEEP_LAST;
            handle.depth = 1000;
            handle.reliability = rmw_qos_reliability_policy_t.RMW_QOS_POLICY_RELIABILITY_RELIABLE;
            handle.durability = rmw_qos_durability_policy_t.RMW_QOS_POLICY_DURABILITY_VOLATILE;
        }

        private void SetProfileDefault()
        {
            handle.history = rmw_qos_history_policy_t.RMW_QOS_POLICY_HISTORY_KEEP_LAST;
            handle.depth = 10;
            handle.reliability = rmw_qos_reliability_policy_t.RMW_QOS_POLICY_RELIABILITY_RELIABLE;
            handle.durability = rmw_qos_durability_policy_t.RMW_QOS_POLICY_DURABILITY_VOLATILE;
        }

        private void SetProfileServicesDefault()
        {
            handle.history = rmw_qos_history_policy_t.RMW_QOS_POLICY_HISTORY_KEEP_LAST;
            handle.depth = 10;
            handle.reliability = rmw_qos_reliability_policy_t.RMW_QOS_POLICY_RELIABILITY_RELIABLE;
            handle.durability = rmw_qos_durability_policy_t.RMW_QOS_POLICY_DURABILITY_VOLATILE;
        }

        private void SetProfileParameterEvents()
        {
            handle.history = rmw_qos_history_policy_t.RMW_QOS_POLICY_HISTORY_KEEP_LAST;
            handle.depth = 1000;
            handle.reliability = rmw_qos_reliability_policy_t.RMW_QOS_POLICY_RELIABILITY_RELIABLE;
            handle.durability = rmw_qos_durability_policy_t.RMW_QOS_POLICY_DURABILITY_VOLATILE;
        }

        private void SetProfileSystemDefault()
        {
            
            handle.history = rmw_qos_history_policy_t.RMW_QOS_POLICY_HISTORY_KEEP_LAST;
            handle.depth = 1000;
            handle.reliability = rmw_qos_reliability_policy_t.RMW_QOS_POLICY_RELIABILITY_RELIABLE;
            handle.durability = rmw_qos_durability_policy_t.RMW_QOS_POLICY_DURABILITY_VOLATILE;
        }

        public void Dispose()
        {
            if (!disposed)
            {
                //TODO(sam): dispose handle?
                disposed = true;
            }
        }
    }
}
