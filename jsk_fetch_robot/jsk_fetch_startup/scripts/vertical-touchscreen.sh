# Set monitor orientation to vertical.
# monitor port (fetch has only one external HDMI port)
# You can list up monitor names by 'xrandr'
MONITOR="HDMI2"
xrandr --output $MONITOR --rotate right # right rotation
# xrandr --output $MONITOR --rotate normal # reset to normal

# Set touchpad orientation to vertical.
# See https://stackoverflow.com/questions/18755967/how-to-make-a-program-that-finds-ids-of-xinput-devices-and-sets-xinput-some-set/18756948#18756948
# You can list up input devices by 'xinput --list'
SEARCH="ILITEK" # touchpad name
ids=$(xinput --list | awk -v search="$SEARCH" \
    '$0 ~ search {match($0, /id=[0-9]+/);\
                  if (RSTART) \
                    print substr($0, RSTART+3, RLENGTH-3)\
                 }'\
     )
for i in $ids
do
    xinput set-prop $i 'Coordinate Transformation Matrix' 0 1 0 -1 0 1 0 0 1 # right rotation
    # xinput set-prop $i 'Coordinate Transformation Matrix' 1 0 0 0 1 0 0 0 1 # reset to normal
done
