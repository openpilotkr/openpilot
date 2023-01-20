#!/usr/bin/env python3
import json
import os
import subprocess
import time
import numpy as np
import unittest
from collections import Counter
from pathlib import Path

from cereal import car
import cereal.messaging as messaging
from cereal.services import service_list
from common.basedir import BASEDIR
from common.timeout import Timeout
from common.params import Params
from selfdrive.controls.lib.events import EVENTS, ET
from selfdrive.hardware import EON, TICI
from selfdrive.loggerd.config import ROOT
from selfdrive.test.helpers import set_params_enabled, release_only
from tools.lib.logreader import LogReader
from selfdrive.swaglog import cloudlog

class TestOnroad():
  def __init__(self, sm=None, pm=None, can_sock=None):

    # Setup sockets
    ignore = ['driverCameraState', 'managerState']
    self.sm = messaging.SubMaster(['deviceState', 'pandaStates', 'modelV2', 'liveCalibration',
                                    'driverMonitoringState', 'longitudinalPlan', 'lateralPlan', 'liveLocationKalman',
                                    'managerState', 'liveParameters', 'radarState', 'liveNaviData', 'liveMapData'],
                                    ignore_alive=ignore, ignore_avg_freq=['radarState', 'longitudinalPlan'])





  def update(self):
    print('tesetddddd')    
    self.sm.update(0)

    print('test message start!')
    if not self.sm.all_alive():
      service_list = self.sm.alive.keys()
      for s in service_list:
        print('{} = alive={} freq_ok={} valid={}'.format( s, self.sm.alive[s], self.sm.freq_ok[s], self.sm.valid[s] ) )

        if s in self.sm.ignore_alive:
          print('    ignore_alive={}'.format(s) )
     
    if not self.sm.all_freq_ok():
      invalid = [s for s, valid in self.sm.valid.items() if not valid]
      not_alive = [s for s, alive in self.sm.alive.items() if not alive]






if __name__ == "__main__":
  to = TestOnroad()
  to.update()
