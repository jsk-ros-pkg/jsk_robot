#!/bin/bash

IMAGE_NAME="${IMAGE_NAME:-ros1-unitree}"
TARGET_MACHINE="${TARGET_MACHINE:-arm64v8}"
HOST_INSTALL_ROOT="${BASE_ROOT:-${PWD}}/"${TARGET_MACHINE}_System
INSTALL_ROOT=System
SOURCE_ROOT=${TARGET_MACHINE}_User

set -xeuf -o pipefail

#  -v ${PWD}/${TARGET_MACHINE}_ws_system:/home/user/${TARGET_MACHINE}_ws_system:rw \
# run on docker
docker run -it --rm \
  -e HOST_UID=$(id -u) -e HOST_GID=$(id -g) \
  -e INSTALL_ROOT=${INSTALL_ROOT} \
  -v ${HOST_INSTALL_ROOT}/ros1_dependencies:/opt/jsk/${INSTALL_ROOT}/ros1_dependencies:ro \
  -v ${HOST_INSTALL_ROOT}/Python:/opt/jsk/${INSTALL_ROOT}/Python:ro \
  -v ${HOST_INSTALL_ROOT}/ros1_inst:/opt/jsk/${INSTALL_ROOT}/ros1_inst:ro \
  -v ${HOST_INSTALL_ROOT}/ros1_dependencies_setup.bash:/opt/jsk/${INSTALL_ROOT}/ros1_dependencies_setup.bash:ro \
  -v ${HOST_INSTALL_ROOT}/system_setup.bash:/opt/jsk/${INSTALL_ROOT}/system_setup.bash:ro \
  -v ${HOST_INSTALL_ROOT}/usercustomize.py:/home/user/.local/lib/python2.7/site-packages/usercustomize.py:ro \
  -v ${HOST_INSTALL_ROOT}/usercustomize.py:/home/user/.local/lib/python3.6/site-packages/usercustomize.py:ro \
  -v ${PWD}/${SOURCE_ROOT}:/opt/jsk/User:rw \
  ${IMAGE_NAME}:${TARGET_MACHINE} \
  bash -c "echo 'source /opt/jsk/User/user_setup.bash; env; cd /opt/jsk/User' > ~/.bashrc; exec \"\$0\""

# https://stackoverflow.com/questions/59814742/docker-run-bash-init-file
