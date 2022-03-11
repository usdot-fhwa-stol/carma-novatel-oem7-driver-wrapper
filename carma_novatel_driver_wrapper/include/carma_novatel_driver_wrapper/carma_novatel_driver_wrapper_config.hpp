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
        double imu_timeout = 0.25;
        double gnss_timeout = 0.25;
        int timer_callback = 500;

        // Stream operator for this config
        friend std::ostream &operator<<(std::ostream &output, const CarmaNovatelDriverWrapperConfig &c)
        {
            output  << "CarmaNovatelDriverWrapperConfig { "<<std::endl
                    <<"imu_timeout: "<< c.imu_timeout << std::endl
                    <<"gnss_timeout: "<< c.gnss_timeout<< std::endl
                    <<"timer_callback: "<<c.timer_callback<<std::endl;
            
            return output;
        }

    };
}  //namespace carma_novatel_driver_wrapper