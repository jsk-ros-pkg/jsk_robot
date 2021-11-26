#!/bin/bash

# claim, power-on and stand
rosservice call /spot/claim "{}"
rosservice call /spot/power_on "{}"
# Strelka falls down because of docking station
# rosservice call /spot/stand "{}"
