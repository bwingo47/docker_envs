#!/usr/bin/env bash

# Default GDB port
GDB_SSH_PORT=7777

while getopts p: flag
do
    case "${flag}" in
        p) GDB_SSH_PORT=${OPTARG};;
        *) echo "!! "
    esac
done

ssh-keygen -f "$HOME/.ssh/known_hosts" -R "[localhost]:$GDB_SSH_PORT"

ssh root@localhost -p $GDB_SSH_PORT