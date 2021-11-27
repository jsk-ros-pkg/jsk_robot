#!/bin/bash

# claim, power-on and stand
rosservice call /spot/claim "{}"

# depend on docking station
if ! "${IS_DOCK}"; then
  rosservice call /spot/power_on "{}"
  rosservice call /spot/stand "{}"
else
  echo "not standing because of dock" 1>&2
fi
