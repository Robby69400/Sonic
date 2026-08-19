#!/usr/bin/env bash

if [ ! -e .env ] ; then
  cp env.example .env
  echo " 🌟 Config created from example, please check it! TIP: Look for \`.env\` file"
  echo ""
fi

source .env
