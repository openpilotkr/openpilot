#!/usr/bin/env python3
import os
import queue
import threading
import time
import subprocess
import re

import cereal.messaging as messaging
from common.params import Params
from common.realtime import DT_TRML

import zmq

# OPKR, this is for getting navi data from external device.

def navid_thread(end_event, nv_queue):
  pm = messaging.PubMaster(['liveENaviData'])
  count = 0

  spd_limit = 0
  safety_distance = 0
  sign_type = 0
  turn_info = 0
  turn_distance = 0
  road_limit_speed = 0
  link_length = 0
  current_link_angle = 0
  next_link_angle = 0
  pos_road_name = ""
  is_highway = 0
  is_tunnel = 0

  OPKR_Debug = Params().get_bool("OPKRDebug")
  if OPKR_Debug:
    opkr_0 = ""
    opkr_1 = ""
    opkr_2 = ""
    opkr_3 = ""
    opkr_4 = ""
    opkr_5 = ""
    opkr_6 = ""
    opkr_7 = ""
    opkr_8 = ""
    opkr_9 = ""


  ip_add = ""
  ip_bind = False
 
  check_connection = False

  ip_count = int(len(Params().get("ExternalDeviceIP", encoding="utf8").split(',')))

  is_metric = Params().get_bool("IsMetric")

  navi_selection = int(Params().get("OPKRNaviSelect", encoding="utf8"))
  if navi_selection == 5:
    waze_alert_id = ""
    waze_alert_distance = ""
    waze_road_speed_limit = ""
    waze_current_speed = ""
    waze_road_name = ""
    waze_nav_sign = ""
    waze_nav_distance = ""

  while not end_event.is_set():
    if not ip_bind:
      if (count % int(max(60., ip_count) / DT_TRML)) == 0:
        os.system("/data/openpilot/selfdrive/assets/addon/script/find_ip.sh &")
      if (count % int((63+ip_count) / DT_TRML)) == 0:
        ip_add = Params().get("ExternalDeviceIPNow", encoding="utf8")
        if ip_add is not None:
          ip_bind = True
          check_connection = True

    if ip_bind:
      spd_limit = 0
      safety_distance = 0
      sign_type = 0
      turn_info = 0
      turn_distance = 0
      road_limit_speed = 0
      link_length = 0
      current_link_angle = 0
      next_link_angle = 0
      pos_road_name = ""
      is_highway = 0
      is_tunnel = 0

      if navi_selection == 5:
        waze_alert_id = ""
        waze_alert_distance = ""
        waze_road_speed_limit = ""
        waze_current_speed = ""
        waze_road_name = ""
        waze_nav_sign = ""
        waze_nav_distance = ""

      if OPKR_Debug:
        opkr_0 = ""
        opkr_1 = ""
        opkr_2 = ""
        opkr_3 = ""
        opkr_4 = ""
        opkr_5 = ""
        opkr_6 = ""
        opkr_7 = ""
        opkr_8 = ""
        opkr_9 = ""

      context = zmq.Context()
      socket = context.socket(zmq.SUB)

      try:
        socket.connect("tcp://" + str(ip_add) + ":5555")
      except:
        socket.connect("tcp://127.0.0.1:5555")
        pass
      socket.subscribe("")

      message = str(socket.recv(), 'utf-8')

      if (count % int(30. / DT_TRML)) == 0:
        try:
          rtext = subprocess.check_output(["netstat", "-tp"])
          check_connection = True if str(rtext).find('navi') else False
        except:
          pass
      
      for line in message.split('\n'):
        if "opkrspdlimit" in line:
          arr = line.split('opkrspdlimit: ')
          spd_limit = arr[1]
        if "opkrspddist" in line:
          arr = line.split('opkrspddist: ')
          safety_distance = arr[1]
        if "opkrsigntype" in line:
          arr = line.split('opkrsigntype: ')
          sign_type = arr[1]
        if "opkrturninfo" in line:
          arr = line.split('opkrturninfo: ')
          turn_info = arr[1]
        if "opkrdistancetoturn" in line:
          arr = line.split('opkrdistancetoturn: ')
          turn_distance = arr[1]
        if "opkrroadlimitspd" in line:
          arr = line.split('opkrroadlimitspd: ')
          road_limit_speed = arr[1]
        if "opkrlinklength" in line:
          arr = line.split('opkrlinklength: ')
          link_length = arr[1]
        if "opkrcurrentlinkangle" in line:
          arr = line.split('opkrcurrentlinkangle: ')
          current_link_angle = arr[1]
        if "opkrnextlinkangle" in line:
          arr = line.split('opkrnextlinkangle: ')
          next_link_angle = arr[1]
        if "opkrposroadname" in line:
          arr = line.split('opkrposroadname: ')
          pos_road_name = arr[1]
        if "opkrishighway" in line:
          arr = line.split('opkrishighway: ')
          is_highway = arr[1]
        if "opkristunnel" in line:
          arr = line.split('opkristunnel: ')
          is_tunnel = arr[1]
        if navi_selection == 5: # NAV unit should be metric. Do not use miles unit.(Distance factor is not detailed.)
          if "opkrwazereportid" in line:
            arr = line.split('opkrwazereportid: ')
            try:
              if "icon_report_speedlimit" in arr[1]:
                waze_alert_id = 1
              elif "icon_report_camera" in arr[1]:
                waze_alert_id = 1
              elif "icon_report_speedcam" in arr[1]:
                waze_alert_id = 1
              elif "icon_report_police" in arr[1]:
                waze_alert_id = 2
              elif "icon_report_hazard" in arr[1]:
                waze_alert_id = 3
              elif "icon_report_traffic" in arr[1]:
                waze_alert_id = 4
            except:
              pass
          if "opkrwazealertdist" in line:
            arr = line.split('opkrwazealertdist: ')
            try:
              if arr[1] is None or arr[1] == "":
                waze_alert_distance = 0
              else:
                waze_alert_distance = re.sub(r'[^0-9]', '', arr[1])
            except:
              pass
          if "opkrwazeroadspdlimit" in line:
            arr = line.split('opkrwazeroadspdlimit: ')
            try:
              if arr[1] == "-1":
                waze_road_speed_limit = 0
              elif arr[1] is None or arr[1] == "":
                waze_road_speed_limit = 0
              else:
                waze_road_speed_limit = arr[1]
            except:
              waze_road_speed_limit = 0
              pass
          if "opkrwazecurrentspd" in line:
            arr = line.split('opkrwazecurrentspd: ')
            try:
              waze_current_speed = arr[1]
            except:
              pass
          if "opkrwazeroadname" in line: # route should be set.
            arr = line.split('opkrwazeroadname: ')
            try:
              waze_road_name = arr[1]
            except:
              pass
          if "opkrwazenavsign" in line: # route should be set.
            arr = line.split('opkrwazenavsign: ')
            try:
              waze_nav_sign = arr[1]
            except:
              pass
          if "opkrwazenavdist" in line: # route should be set.
            arr = line.split('opkrwazenavdist: ')
            try:
              waze_nav_distance = arr[1]
            except:
              pass

        if OPKR_Debug:
          try:
            if "opkr0" in line:
              arr = line.split('opkr0   : ')
              opkr_0 = arr[1]
          except:
            pass
          try:
            if "opkr1" in line:
              arr = line.split('opkr1   : ')
              opkr_1 = arr[1]
          except:
            pass
          try:
            if "opkr2" in line:
              arr = line.split('opkr2   : ')
              opkr_2 = arr[1]
          except:
            pass
          try:
            if "opkr3" in line:
              arr = line.split('opkr3   : ')
              opkr_3 = arr[1]
          except:
            pass
          try:
            if "opkr4" in line:
              arr = line.split('opkr4   : ')
              opkr_4 = arr[1]
          except:
            pass
          try:
            if "opkr5" in line:
              arr = line.split('opkr5   : ')
              opkr_5 = arr[1]
          except:
            pass
          try:
            if "opkr6" in line:
              arr = line.split('opkr6   : ')
              opkr_6 = arr[1]
          except:
            pass
          try:
            if "opkr7" in line:
              arr = line.split('opkr7   : ')
              opkr_7 = arr[1]
          except:
            pass
          try:
            if "opkr8" in line:
              arr = line.split('opkr8   : ')
              opkr_8 = arr[1]
          except:
            pass
          try:
            if "opkr9" in line:
              arr = line.split('opkr9   : ')
              opkr_9 = arr[1]
          except:
            pass

      navi_msg = messaging.new_message('liveENaviData')
      navi_msg.liveENaviData.speedLimit = int(spd_limit)
      navi_msg.liveENaviData.safetyDistance = float(safety_distance)
      navi_msg.liveENaviData.safetySign = int(sign_type)
      navi_msg.liveENaviData.turnInfo = int(turn_info)
      navi_msg.liveENaviData.distanceToTurn = float(turn_distance)
      navi_msg.liveENaviData.connectionAlive = bool(check_connection)
      navi_msg.liveENaviData.roadLimitSpeed = int(road_limit_speed)
      navi_msg.liveENaviData.linkLength = int(link_length)
      navi_msg.liveENaviData.currentLinkAngle = int(current_link_angle)
      navi_msg.liveENaviData.nextLinkAngle = int(next_link_angle)
      navi_msg.liveENaviData.posRoadName = str(pos_road_name)
      navi_msg.liveENaviData.isHighway = bool(int(is_highway))
      navi_msg.liveENaviData.isTunnel = bool(int(is_tunnel))

      if OPKR_Debug:
        navi_msg.liveENaviData.opkr0 = str(opkr_0)
        navi_msg.liveENaviData.opkr1 = str(opkr_1)
        navi_msg.liveENaviData.opkr2 = str(opkr_2)
        navi_msg.liveENaviData.opkr3 = str(opkr_3)
        navi_msg.liveENaviData.opkr4 = str(opkr_4)
        navi_msg.liveENaviData.opkr5 = str(opkr_5)
        navi_msg.liveENaviData.opkr6 = str(opkr_6)
        navi_msg.liveENaviData.opkr7 = str(opkr_7)
        navi_msg.liveENaviData.opkr8 = str(opkr_8)
        navi_msg.liveENaviData.opkr9 = str(opkr_9)

      if navi_selection == 5:
        navi_msg.liveENaviData.wazeReportId = int(waze_alert_id)
        navi_msg.liveENaviData.wazeAlertDistance = int(waze_alert_distance)
        navi_msg.liveENaviData.wazeRoadSpeedLimit = round(int(waze_road_speed_limit) * 0.6214) if not is_metric else int(waze_road_speed_limit)
        navi_msg.liveENaviData.wazeCurrentSpeed = round(int(waze_current_speed) * 0.6214) if not is_metric else int(waze_current_speed)
        navi_msg.liveENaviData.wazeRoadName = str(waze_road_name)
        navi_msg.liveENaviData.wazeNavSign = str(hex(waze_nav_sign))
        navi_msg.liveENaviData.wazeNavDistance = int(waze_nav_distance)

      pm.send('liveENaviData', navi_msg)

    count += 1
    time.sleep(DT_TRML)


def main():
  nv_queue = queue.Queue(maxsize=1)
  end_event = threading.Event()

  t = threading.Thread(target=navid_thread, args=(end_event, nv_queue))

  t.start()

  try:
    while True:
      time.sleep(1)
      if not t.is_alive():
        break
  finally:
    end_event.set()

  t.join()


if __name__ == "__main__":
  main()