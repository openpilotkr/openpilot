#!/usr/bin/bash

export LD_LIBRARY_PATH=/data/data/com.termux/files/usr/lib
export HOME=/data/data/com.termux/files/home
export PATH=/usr/local/bin:/data/data/com.termux/files/usr/bin:/data/data/com.termux/files/usr/sbin:/data/data/com.termux/files/usr/bin/applets:/bin:/sbin:/vendor/bin:/system/sbin:/system/bin:/system/xbin:/data/data/com.termux/files/usr/bin/python
export PYTHONPATH=/data/openpilot

cd /data/openpilot
ping -q -c 1 -w 1 google.com &> /dev/null
if [ "$?" == "0" ]; then
  REMOVED_BRANCH=$(git branch -vv | grep ': gone]' | awk '{print $1}')
  if [ "$REMOVED_BRANCH" != "" ]; then
    if [ "$REMOVED_BRANCH" == "*" ]; then
      REMOVED_BRANCH=$(git branch -vv | grep ': gone]' | awk '{print $2}')
    fi
    git remote prune origin --dry-run
    echo $REMOVED_BRANCH | xargs git branch -D
    sed -i "/$REMOVED_BRANCH/d" .git/config
  fi
  CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
  /data/data/com.termux/files/usr/bin/git clean -d -f -f
  /data/data/com.termux/files/usr/bin/git fetch --all
  /data/data/com.termux/files/usr/bin/git reset --hard origin/$CURRENT_BRANCH
  /data/data/com.termux/files/usr/bin/git pull origin $CURRENT_BRANCH

  if [ -f "/data/openpilot/prebuilt" ]; then
    pkill -f thermald
    rm -f /data/openpilot/prebuilt
  fi

  reboot
fi