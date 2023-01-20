#!/usr/bin/bash

export LD_LIBRARY_PATH=/data/data/com.termux/files/usr/lib
export HOME=/data/data/com.termux/files/home
export PATH=/usr/local/bin:/data/data/com.termux/files/usr/bin:/data/data/com.termux/files/usr/sbin:/data/data/com.termux/files/usr/bin/applets:/bin:/sbin:/vendor/bin:/system/sbin:/system/bin:/system/xbin:/data/data/com.termux/files/usr/bin/python
export PYTHONPATH=/data/openpilot

cd /data/openpilot
IP_FILE=$(cat /data/params/d/ExternalDeviceIP)

OIFS=$IFS
IFS=',' read -r -a array <<< "$IP_FILE"

for NUM in "${!array[@]}"; do
  ping -c 1 -W 1 ${array[NUM]}
  if [ $(echo $?) == "0" ]; then
    nc -vz ${array[NUM]} 5555
    if [ $(echo $?) == "0" ]; then
      echo -n ${array[NUM]} > /data/params/d/ExternalDeviceIPNow
    fi
  fi
done;
