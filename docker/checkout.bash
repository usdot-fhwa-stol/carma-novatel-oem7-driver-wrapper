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

# CARMA packages checkout script
# Optional argument to set the root checkout directory with no ending '/' default is '~'

set -exo pipefail

dir=~
while [[ $# -gt 0 ]]; do
      arg="$1"
      case $arg in
            -d|--develop)
                  BRANCH=foxy/develop
                  shift
            ;;
            -r|--root)
                  dir=$2
                  shift
                  shift
            ;;
      esac
done

cd ~/src
####Add dependencies#######

if [[ "$BRANCH" = "foxy/develop" ]]; then
      sudo git clone https://github.com/usdot-fhwa-stol/carma-msgs.git ${dir}/src/CARMAMsgs --branch $BRANCH
      sudo git clone https://github.com/usdot-fhwa-stol/carma-utils.git --branch $BRANCH
else
      sudo git clone https://github.com/usdot-fhwa-stol/carma-msgs.git ${dir}/src/CARMAMsgs --branch foxy/develop
      sudo git clone https://github.com/usdot-fhwa-stol/carma-utils.git --branch $BRANCH
fi

# novatel driver 
sudo git clone https://github.com/novatel/novatel_oem7_driver.git --branch ros2-dev 
# Checkout verified commit
cd novatel_oem7_driver
sudo git checkout 3055e220bb9715b59c3ef53ab0aba05a495d9d5c