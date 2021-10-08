#!/bin/bash

jsk_fetch_startup=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`"/.. > /dev/null && pwd)

set -x

if [ ! command -v tmuxinator &> /dev/null ]; then
  sudo apt update
  sudo apt install tmuxinator
fi
if [ ! -d "$HOME/.tmuxinator" ]; then
  mkdir $HOME/.tmuxinator
fi
ln -sf $jsk_fetch_startup/tmuxinator_yaml/log.yml $HOME/.tmuxinator/log.yml

set +x
