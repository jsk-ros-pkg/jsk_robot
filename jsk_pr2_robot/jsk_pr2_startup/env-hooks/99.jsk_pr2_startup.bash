#!/bin/bash
# -*- mode: shell-script -*-

function rossetpr1040() {
    rossetmaster pr1040.jsk.imi.i.u-tokyo.ac.jp
    export ROBOT=PR2
    rossetip
}

function rossetpr1012() {
    rossetmaster pr1012.jsk.imi.i.u-tokyo.ac.jp
    export ROBOT=PR2
    rossetip
}

function rossetc1() {
    rossetmaster c1
    export ROBOT=PR2
    rossetip
}
