#!/bin/bash

set -e

if [ -z "$CARLA_HOME" ]; then
    echo "Error: Please set \$CARLA_HOME before running this script"
    return 0
fi

if [ -z "$CARLA_VERSION" ]; then
   CARLA_VERSION="0.9.11"
fi

CARLA_EGG_FILE=${CARLA_HOME}/PythonAPI/carla/dist/carla-"${CARLA_VERSION}"-py3.7-linux-x86_64.egg
if [ ! -f "$CARLA_EGG_FILE" ]; then
    echo "Error: $CARLA_EGG_FILE can not be found. Please make sure you are using python3.7 and carla 0.9.11. "
    return 0
fi

CACHE=${PWD}/cache
if [ ! -d "$CACHE" ]; then
  echo "creating cache folder for carla PythonAPI egg file"
  mkdir -p "$CACHE"
fi

echo "copying egg file to cache folder"
cp  $CARLA_EGG_FILE $CACHE

echo "unzip egg file"
unzip "${CACHE}"/carla-"${CARLA_VERSION}"-py3.7-linux-x86_64.egg -d "${CACHE}"/carla-"${CARLA_VERSION}"-py3.7-linux-x86_64

echo "copy setup file to egg folder"
SETUP_PY=${PWD}/scripts/setup.py
cp "$SETUP_PY"  "${CACHE}"/carla-"${CARLA_VERSION}"-py3.7-linux-x86_64/

echo "Successful! Run 'pip install -e ${CACHE}/carla-${CARLA_VERSION}-py3.7-linux-x86_64' to install carla into your python package "
conda activate opencda
pip install -e ${CACHE}/carla-"${CARLA_VERSION}"-py3.7-linux-x86_64

echo "Sucessful Setup!"