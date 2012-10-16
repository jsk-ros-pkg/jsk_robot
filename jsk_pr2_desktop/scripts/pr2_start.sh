#!/bin/bash
set +e

yes | ssh applications@pr1012 "(PS1=applications source .bashrc && robot claim)"
yes | ssh applications@pr1012 "(PS1=applications source .bashrc && robot stop)"
sleep 2
yes | ssh applications@pr1012 "(PS1=applications source .bashrc && robot start)"
sleep 5
