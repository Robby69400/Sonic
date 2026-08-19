#!/usr/bin/env bash

source load_settings.sh

python flash.py $DEFAULT_LOAD_FILE -p $UPLOAD_PORT
