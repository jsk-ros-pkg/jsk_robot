#!/bin/bash
set +e

yes | ssh applications@pr1012 "(PS1=applications source .bashrc && robot stop)"
yes | ssh applications@pr1012 "(PS1=applications source .bashrc && robot release)"
# pkill -9 rviz
