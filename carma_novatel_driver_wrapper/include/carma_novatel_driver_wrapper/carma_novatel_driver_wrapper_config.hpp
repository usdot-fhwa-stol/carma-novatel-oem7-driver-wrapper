#pragma once

/*
 * Copyright (C) 2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <iostream>
#include <vector>

namespace carma_novatel_driver_wrapper
{
     /**
  * \brief Stuct containing the algorithm configuration values for the CarmaNovatelDriverWrapper
  */

    struct CarmaNovatelDriverWrapperConfig
    {
        //! Node Namespace. Any node under this namespace shall have its lifecycle managed by this controller
        std::string subsystem_namespace = "/false_namespace";

        //! Timeout in ms for service availability
        uint64_t service_timeout_ms = 1000;

        //! Timeout in ms for service calls
        uint64_t call_timeout_ms = 1000;

        uint64_t imu_timeout = 1000;
        uint64_t gnss_timeout = 1000;

        // Stream operator for this config
        friend std::ostream &operator<<(std::ostream &output, const CarmaNovatelDriverWrapperConfig &c)
        {
            output  << "CarmaNovatelDriverWrapperConfig { "<<std::endl
                    << "service_timeout_ms: " << c.service_timeout_ms << std::endl
                    << "call_timeout_ms: " << c.call_timeout_ms << std::endl
                    << "subsystem_namespace: " << c.subsystem_namespace << std::endl
                    <<"imu_timeout: "<< c.imu_timeout << std::endl
                    <<"gnss_timeout: "<< c.gnss_timeout<< std::endl; 
        }

    };
}  //namespace carma_novatel_driver_wrapper