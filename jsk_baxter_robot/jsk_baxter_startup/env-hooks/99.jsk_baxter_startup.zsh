#!/bin/zsh
# -*- mode: shell-script -*-

function rossetbaxter() {
    rossetmaster baxter.jsk.imi.i.u-tokyo.ac.jp
    export ROBOT=BAXTER
    rossetip
}