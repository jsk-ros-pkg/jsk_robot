#!/bin/bash

# claim, power-on and stand
rosservice call /spot/claim "{}"
rosservice call /spot/power_on "{}"
# depend on docking station
if "${DOCK}"; then
  rosservice call /spot/stand "{}"
fi
