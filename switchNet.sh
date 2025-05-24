#!/bin/bash

mode="$1"

if [ "$mode" = "ap" ]; then
    nmcli connection up "AAAfly"
elif [ "$mode" = "client" ]; then
    nmcli connection down "AAAfly"
else
    echo "Usage: $0 [ap|client]"
    exit 1
fi
