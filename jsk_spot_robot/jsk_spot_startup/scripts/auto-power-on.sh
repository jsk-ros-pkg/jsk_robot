#!/bin/bash

# claim, power-on and stand
rosservice call /spot/claim "{}"
rosservice call /spot/power_on "{}"
rosservice call /spot/stand "{}"
