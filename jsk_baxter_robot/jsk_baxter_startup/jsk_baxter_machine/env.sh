#!/usr/bin/env bash

# shellcheck source=/dev/null
if [ -e "$HOME/ros/melodic/" ]; then
  source "$HOME/ros/melodic/devel/setup.bash"
fi
if [ -e "$HOME/ros/noetic/" ]; then
  source "$HOME/ros/noetic/devel/setup.bash"
fi
exec "$@"
