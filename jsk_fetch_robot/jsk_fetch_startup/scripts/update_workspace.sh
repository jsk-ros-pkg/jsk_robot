#!/usr/bin/env bash

cp $(rospack find jsk_fetch_startup)/scripts/update_workspace_main.sh /tmp/update_workspace.sh
/tmp/update_workspace.sh $@
