#!/usr/bin/env bash
set -ex

exec supervisord -c /root/startup/supervisord.conf