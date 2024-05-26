#!/bin/bash

while true; do
    if ! lsof -i :5900 > /dev/null; then
        x11vnc --cursor arrow 
    fi
    sleep 20
done
