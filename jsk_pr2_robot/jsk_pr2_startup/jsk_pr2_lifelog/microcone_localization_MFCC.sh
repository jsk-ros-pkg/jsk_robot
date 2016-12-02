#!/bin/sh

DIR=$(rospack find jsk_pr2_startup)/jsk_pr2_lifelog
CONF=${DIR}/hark-config
BATCHFLOW=$(which batchflow)
${BATCHFLOW} ${DIR}/microcone_localization_MFCC.n ${CONF}/microcone_rectf.zip ${CONF}/pr2_noise.txt
