#!/bin/bash

# sit, power-off and releae
rosservice call /spot/sit "{}"
rosservice call /spot/power_off "{}"
rosservice call /spot/release "{}"
