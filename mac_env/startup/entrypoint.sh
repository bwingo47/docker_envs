#!/usr/bin/env bash
set -ex

# # Default GDB port
# GDB_SSH_PORT=7777

# while getopts p: flag
# do
#     case "${flag}" in
#         p) GDB_SSH_PORT=${OPTARG};;
#         *) echo "!! "
#     esac
# done

# source /root/.profile
exec supervisord -c /root/startup/supervisord.conf