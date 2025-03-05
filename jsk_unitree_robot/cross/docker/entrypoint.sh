#!/bin/bash

function fail {
    printf '\033[31m%s\033[0m\n' "$1" >&2 ## Send message to stderr.
    exit "${2-1}" ## Return a code specified by $2, or 1 by default.
}
[ -z "${HOST_UID}" ] && fail "ERROR: HOST_UID is requried, add '-e HOST_UID=$(id -u)' to your docker commandline option"
[ -z "${HOST_GID}" ] && fail "ERROR: HOST_GID is requried, add '-e HOST_GID=$(id -g)' to your docker commandline option"

# create user with same UID as the host user
# -g The numerical value of the group's ID.
groupadd -g $HOST_GID user
# -u The numerical value of the user's ID.
# -o Allow the creation of a user account with a duplicate (non-unique) UID.
# -m Create the user's home directory if it does not exist.
# -g The group name or number of the user's initial login group.
# -s The name of the user's login shell.
useradd -u $HOST_UID -o -m -g $HOST_GID -s /usr/bin user -G sudo
echo user:user | chpasswd
export HOME=/home/user
chown $HOST_UID:$HOST_GID $HOME
cd $HOME

runuser -u user -- "$@"
