#!/bin/sh
until simspark; do
    echo "simspark crashed with exit code $?.  Respawning.." >&2
    sleep 1
done
