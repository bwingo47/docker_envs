#!/usr/bin/env bash

ssh-keygen -f "$HOME/.ssh/known_hosts" -R "[localhost]:7777"

ssh root@localhost -p 7777