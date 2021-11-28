#!/bin/bash

# claim, power-on
rosservice call /spot/claim "{}"
rosservice call /spot/power_on "{}"

# stand when not connected to a docking station
if [[ -z ${STAND_AFTER_AUTO_POWER_ON} ]]; then
  echo "please set STAND_AFTER_AUTO_POWER_ON variable"
  exit 1
elif "${STAND_AFTER_AUTO_POWER_ON}"; then
  rosservice call /spot/stand "{}"
else
  echo "Do not stand because STAND_AFTER_AUTO_POWER_ON is set to false" 1>&2
fi
