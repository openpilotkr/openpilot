#!/usr/bin/bash

ALIAS_CHECK=$(/usr/bin/grep gitpull /system/comma/home/.bash_profile)
# GET_PROP1=$(getprop persist.sys.locale)
# GET_PROP2=$(getprop persist.sys.local)
GET_PROP_ATZ=$(getprop persist.sys.timezone)
if [ -f "/data/params/d/OPKRTimeZone" ]; then
    GET_PROP_STZ=$(cat /data/params/d/OPKRTimeZone)
fi

if [ "$ALIAS_CHECK" == "" ]; then
    sleep 3
    mount -o remount,rw /system
    echo "alias gi='/data/openpilot/selfdrive/assets/addon/script/gitpull.sh'" >> /system/comma/home/.bash_profile
    mount -o remount,r /system
fi

# if [ "$GET_PROP1" != "ko-KR" ]; then
#     setprop persist.sys.locale ko-KR
# fi
# if [ "$GET_PROP2" != "ko-KR" ]; then
#     setprop persist.sys.local ko-KR
# fi
if [ "$GET_PROP_STZ" != "" ] && [ "$GET_PROP_ATZ" != "$GET_PROP_STZ" ]; then
    setprop persist.sys.timezone $GET_PROP_STZ
fi

if [ ! -f "/system/fonts/KaiGenGothicKR-Normal.ttf" ]; then
    sleep 3
    mount -o remount,rw /system
    cp -rf /data/openpilot/selfdrive/assets/addon/font/KaiGenGothicKR* /system/fonts/
    cp -rf /data/openpilot/selfdrive/assets/addon/font/fonts.xml /system/etc/fonts.xml
    chmod 644 /system/etc/fonts.xml
    chmod 644 /system/fonts/KaiGenGothicKR*
    mount -o remount,r /system
fi

if [ ! -f "/system/fonts/NotoSansArabic-Regular.ttf" ]; then
    sleep 3
    mount -o remount,rw /system
    cp -rf /data/openpilot/selfdrive/assets/addon/font/NotoSansArabic* /system/fonts/
    cp -rf /data/openpilot/selfdrive/assets/addon/font/fonts.xml /system/etc/fonts.xml
    chmod 644 /system/etc/fonts.xml
    chmod 644 /system/fonts/NotoSansArabic*
    mount -o remount,r /system
    reboot
fi

if [ -f "/data/bootanimation.zip" ]; then
    DIFF=$(diff /data/bootanimation.zip /system/media/bootanimation.zip)
    if [ "$DIFF" != "" ]; then
        sleep 3
        mount -o remount,rw /system
        cp -f /data/bootanimation.zip /system/media/bootanimation.zip
        chmod 644 /system/media/bootanimation.zip
        mount -o remount,r /system
    fi
fi

if [ ! -f "/data/openpilot/selfdrive/modeld/models/supercombo.dlc" ]; then
    cat /data/openpilot/selfdrive/modeld/models/supercombo.dlca* > /data/openpilot/selfdrive/modeld/models/supercombo.dlc
fi

if [ ! -f "/data/openpilot/selfdrive/modeld/models/supercombo.onnx" ]; then
    cat /data/openpilot/selfdrive/modeld/models/supercombo.onnxa* > /data/openpilot/selfdrive/modeld/models/supercombo.onnx
fi

export PASSIVE="0"
exec ./launch_chffrplus.sh

