#!/bin/bash

#  Copyright (C) 2021 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

cd ~/
# Source Environment variables
source /opt/ros/foxy/setup.bash
source /home/carma/catkin/setup.bash
# Install dependencies
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
# Build 
colcon build --packages-up-to novatel_oem7_driver carma_novatel_driver_wrapper