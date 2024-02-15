from dataclasses import dataclass
from typing import Dict, List, Union

from cereal import car
from common.conversions import Conversions as CV
from selfdrive.car import dbc_dict
from selfdrive.car.docs_definitions import CarInfo, Harness
from common.params import Params
Ecu = car.CarParams.Ecu

# Steer torque limits
class CarControllerParams:
  ACCEL_MIN = -4.0 # m/s
  ACCEL_MAX = 2.0 # m/s

  def __init__(self, CP):
    self.STEER_MAX = int(Params().get("SteerMaxAdj", encoding="utf8"))  # default 384
    self.STEER_DELTA_UP = int(Params().get("SteerDeltaUpAdj", encoding="utf8"))  # default 3
    self.STEER_DELTA_DOWN = int(Params().get("SteerDeltaDownAdj", encoding="utf8"))  # default 7
    self.STEER_DRIVER_ALLOWANCE = 50
    self.STEER_DRIVER_MULTIPLIER = 2
    self.STEER_DRIVER_FACTOR = 1

class CAR:
  # KGM
  TORRES_EVX = "KGM TORRES EVX"

@dataclass
class KgmCarInfo(CarInfo):
  package: str="SCC + LKAS"
  good_torque: bool = True


CAR_INFO: Dict[str, Union[KgmCarInfo, List[KgmCarInfo]]] = {
  CAR.TORRES_EVX: KgmCarInfo("Kgm Torres EVX", "All"),
}

class Buttons:
  NONE = 0
  RES_ACCEL = 1
  SET_DECEL = 2
  GAP_DIST = 3
  CANCEL = 4

FINGERPRINTS = {
  CAR.TORRES_EVX: [{
  }]
}

if Params().get_bool("FingerprintTwoSet"):
  FW_VERSIONS = {
    CAR.TORRES_EVX: {
      (Ecu.fwdRadar, 0x7d0, None): [b'\xf1\x00IK__ SCC F-CUP      1.00 1.02 96400-G9100         \xf1\xa01.02',],
      (Ecu.esp, 0x7d1, None): [b'\xf1\x00\x00\x00\x00\x00\x00\x00',],
      (Ecu.engine, 0x7e0, None): [b'\xf1\x81640F0051\x00\x00\x00\x00\x00\x00\x00\x00',],
      (Ecu.eps, 0x7d4, None): [b'\xf1\x00IK  MDPS R 1.00 1.06 57700-G9420 4I4VL106',],
      (Ecu.fwdCamera, 0x7c4, None): [b'\xf1\x00IK  MFC  AT USA LHD 1.00 1.01 95740-G9000 170920',],
      (Ecu.transmission, 0x7e1, None): [b'\xf1\x87VDJLT17895112DN4\x88fVf\x99\x88\x88\x88\x87fVe\x88vhwwUFU\x97eFex\x99\xff\xb7\x82\xf1\x81E25\x00\x00\x00\x00\x00\x00\x00\xf1\x00bcsh8p54  E25\x00\x00\x00\x00\x00\x00\x00SIK0T33NB2\x11\x1am\xda',],
    },
  }

CHECKSUM = {
  "crc8": [],
  "6B": [],
}

FEATURES = {
  # Use Cluster for Gear Selection, rather than Transmission
  "use_cluster_gears": {},
  # Use TCU Message for Gear Selection
  "use_tcu_gears": {},
  # Use E_GEAR Message for Gear Selection
  "use_elect_gears": {CAR.TORRES_EVX},

  # send LFA MFA message for new HKG models
  # Insert your car in this if you want turn LFA icon on.
  # need to add lfa modded cars which are changed from lkas to lfa cam
  "send_lfahda_mfa": {CAR.TORRES_EVX},

  "send_hda_mfa": {CAR.TORRES_EVX},
  # these cars use the FCA11 message for the AEB and FCW signals, all others use SCC12
  # Insert your car in this if you see front collision error on your cluster.
  "use_fca": {CAR.TORRES_EVX},
}

HYBRID_CAR = {}
EV_CAR = {CAR.TORRES_EVX}

DBC = {
  CAR.TORRES_EVX: dbc_dict('kgm_generic', None),
}

STEER_THRESHOLD = int(Params().get("SteerThreshold", encoding="utf8"))
