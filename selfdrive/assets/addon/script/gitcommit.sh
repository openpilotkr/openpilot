#!/usr/bin/bash

export LD_LIBRARY_PATH=/data/data/com.termux/files/usr/lib
export HOME=/data/data/com.termux/files/home
export PATH=/usr/local/bin:/data/data/com.termux/files/usr/bin:/data/data/com.termux/files/usr/sbin:/data/data/com.termux/files/usr/bin/applets:/bin:/sbin:/vendor/bin:/system/sbin:/system/bin:/system/xbin:/data/data/com.termux/files/usr/bin/python
export PYTHONPATH=/data/openpilot

# acquire git hash from remote
cd /data/openpilot
ping -q -c 1 -w 1 google.com &> /dev/null
if [ "$?" == "0" ]; then
  /data/openpilot/selfdrive/assets/addon/script/git_remove.sh
  CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
  LOCAL_HASH=$(git rev-parse HEAD)
  /data/data/com.termux/files/usr/bin/git fetch
  REMOTE_HASH=$(git rev-parse --verify origin/$CURRENT_BRANCH)
  echo -n "$REMOTE_HASH" > /data/params/d/GitCommitRemote
  if [ "$LOCAL_HASH" != "$REMOTE_HASH" ]; then
    wget https://raw.githubusercontent.com/openpilotkr/openpilot/$CURRENT_BRANCH/OPKR_Updates.txt -O /data/OPKR_Updates.txt
  else
    if [ -f "/data/OPKR_Updates.txt" ]; then
      rm -f /data/OPKR_Updates.txt
    fi
  fi
fi
