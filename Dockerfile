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
ARG DOCKER_ORG="usdotfhwastoldev"
ARG DOCKER_TAG="develop"
FROM ${DOCKER_ORG}/carma-base:${DOCKER_TAG} as base_image

FROM base_image as source_code
ARG GIT_BRANCH="develop" 


# Get source code
RUN mkdir ~/src
COPY --chown=carma . /home/carma/src/
RUN ~/src/docker/checkout.bash -b ${GIT_BRANCH}

FROM base_image as install

ARG ROS1_PACKAGES=""
ENV ROS1_PACKAGES=${ROS1_PACKAGES}
ARG ROS2_PACKAGES=""
ENV ROS2_PACKAGES=${ROS2_PACKAGES}

RUN mkdir ~/src
COPY --from=source_code --chown=carma /home/carma/src /home/carma/src

RUN ~/src/docker/install.sh

FROM base_image

ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="carma-novatel-gps-driver-wrapper"
LABEL org.label-schema.description="carma novatel driver wrapper for the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/carma-novatel-oem7-driver-wrapper/"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

COPY --from=install --chown=carma /home/carma/install /opt/carma/install
COPY --from=install --chown=carma /opt/ros /opt/ros

CMD [ "wait-for-it.sh", "localhost:11311", "--", "ros2","launch", "carma_novatel_driver_wrapper", "carma-novatel-driver-wrapper-launch.py"]
