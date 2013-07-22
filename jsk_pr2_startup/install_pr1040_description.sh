#!/usr/bin/env bash

echo "This script install red colored pr2 description on to you computer"
echo "  This does not brakes you existing models"

if ! type wget ;then 
    echo "this script needs wget command." 
    exit 1
fi

wget http://www.jsk.t.u-tokyo.ac.jp/~k-okada/pr2/pr2_description.pr1040ver.tgz 
sudo tar axvf pr2_description.pr1040ver.tgz -C `rospack find pr2_description`/../
rm pr2_description.pr1040ver.tgz

echo "Done."
