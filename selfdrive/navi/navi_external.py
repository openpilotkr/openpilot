#!/usr/bin/env python3
import os
import queue
import threading
import time
import subprocess

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

  ip_add = ""
  ip_bind = False
 
  check_connection = False

  ip_count = int(len(Params().get("ExternalDeviceIP", encoding="utf8").split(',')))


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