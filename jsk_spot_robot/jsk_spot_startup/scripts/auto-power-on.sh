#!/bin/bash

# claim, power-on
rosservice call /spot/claim "{}"
rosservice call /spot/power_on "{}"

# stand when not connected to a docking station
if [[ -z ${USE_DOCKING_STATION} ]]; then
  echo "please set USE_DOCKING_STATION variable"
  exit 1
elif "${USE_DOCKING_STATION}"; then
  echo "Do not stand because USE_DOCKING_STATION is set to true" 1>&2
else
  rosservice call /spot/stand "{}"
fi
