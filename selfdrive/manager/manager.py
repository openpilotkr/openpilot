#!/usr/bin/env python3
import datetime
import os
import signal
import subprocess
import sys
import traceback
from typing import List, Tuple, Union

import cereal.messaging as messaging
import selfdrive.sentry as sentry
from common.basedir import BASEDIR
from common.params import Params, ParamKeyType
from common.text_window import TextWindow
from selfdrive.boardd.set_time import set_time
from selfdrive.hardware import HARDWARE, PC, EON
from selfdrive.hardware.eon.apk import (pm_apply_packages, update_apks)
from selfdrive.manager.helpers import unblock_stdout
from selfdrive.manager.process import ensure_running
from selfdrive.manager.process_config import managed_processes
from selfdrive.athena.registration import register, UNREGISTERED_DONGLE_ID
from selfdrive.swaglog import cloudlog, add_file_handler
from selfdrive.version import is_dirty, get_commit, get_version, get_origin, get_short_branch, \
                              terms_version, training_version


sys.path.append(os.path.join(BASEDIR, "pyextra"))


def manager_init() -> None:
  # update system time from panda
  set_time(cloudlog)

  # save boot log
  # subprocess.call("./bootlog", cwd=os.path.join(BASEDIR, "selfdrive/loggerd"))

  params = Params()
  params.clear_all(ParamKeyType.CLEAR_ON_MANAGER_START)

  default_params: List[Tuple[str, Union[str, bytes]]] = [
    ("CompletedTrainingVersion", "0"),
    ("HasAcceptedTerms", "0"),
    ("OpenpilotEnabledToggle", "1"),
    ("IsMetric", "1"),
    ("EndToEndToggle", "1"),
    ("IsOpenpilotViewEnabled", "0"),
    ("OpkrAutoShutdown", "2"),
    ("OpkrForceShutdown", "5"),
    ("OpkrAutoScreenOff", "-2"),
    ("OpkrUIBrightness", "0"),
    ("OpkrUIVolumeBoost", "0"),
    ("OpkrEnableDriverMonitoring", "1"),
    ("OpkrEnableLogger", "0"),
    ("OpkrEnableUploader", "0"),
    ("OpkrEnableGetoffAlert", "0"),
    ("OpkrAutoResume", "1"),
    ("OpkrVariableCruise", "1"),
    ("OpkrLaneChangeSpeed", "30"),
    ("OpkrAutoLaneChangeDelay", "0"),
    ("OpkrSteerAngleCorrection", "0"),
    ("PutPrebuiltOn", "1"),
    ("LdwsCarFix", "0"),
    ("LateralControlMethod", "3"),
    ("CruiseStatemodeSelInit", "1"),
    ("InnerLoopGain", "35"),
    ("OuterLoopGain", "20"),
    ("TimeConstant", "14"),
    ("ActuatorEffectiveness", "20"),
    ("Scale", "1500"),
    ("LqrKi", "16"),
    ("DcGain", "265"),
    ("PidKp", "25"),
    ("PidKi", "40"),
    ("PidKd", "150"),
    ("PidKf", "7"),
    ("TorqueKp", "10"),
    ("TorqueKf", "10"),
    ("TorqueKi", "1"),
    ("TorqueFriction", "65"),
    ("TorqueUseAngle", "1"),
    ("TorqueMaxLatAccel", "27"),
    ("TorqueAngDeadZone", "10"),
    ("CameraOffsetAdj", "60"),
    ("PathOffsetAdj", "0"),
    ("SteerRatioAdj", "1550"),
    ("SteerRatioMaxAdj", "1750"),
    ("SteerActuatorDelayAdj", "36"),
    ("SteerLimitTimerAdj", "100"),
    ("TireStiffnessFactorAdj", "85"),
    ("SteerMaxBaseAdj", "384"),
    ("SteerMaxAdj", "384"),
    ("SteerDeltaUpBaseAdj", "3"),
    ("SteerDeltaUpAdj", "3"),
    ("SteerDeltaDownBaseAdj", "7"),
    ("SteerDeltaDownAdj", "7"),
    ("OpkrBatteryChargingControl", "1"),
    ("OpkrBatteryChargingMin", "50"),
    ("OpkrBatteryChargingMax", "60"),
    ("LeftCurvOffsetAdj", "0"),
    ("RightCurvOffsetAdj", "0"),
    ("DebugUi1", "0"),
    ("DebugUi2", "0"),
    ("DebugUi3", "0"),
    ("LongLogDisplay", "0"),
    ("OpkrBlindSpotDetect", "1"),
    ("OpkrMaxAngleLimit", "90"),
    ("OpkrSteerMethod", "0"),
    ("OpkrMaxSteeringAngle", "90"),
    ("OpkrMaxDriverAngleWait", "0.002"),
    ("OpkrMaxSteerAngleWait", "0.001"),
    ("OpkrDriverAngleWait", "0.001"),
    ("OpkrSpeedLimitOffset", "0"),
    ("OpkrLiveSteerRatio", "1"),
    ("OpkrVariableSteerMax", "0"),
    ("OpkrVariableSteerDelta", "0"),
    ("FingerprintTwoSet", "0"),
    ("OpkrDrivingRecord", "0"),
    ("OpkrTurnSteeringDisable", "0"),
    ("CarModel", ""),
    ("OpkrHotspotOnBoot", "0"),
    ("OpkrSSHLegacy", "1"),
    ("CruiseOverMaxSpeed", "0"),
    ("JustDoGearD", "0"),
    ("LanelessMode", "2"),
    ("ComIssueGone", "1"),
    ("MaxSteer", "384"),
    ("MaxRTDelta", "112"),
    ("MaxRateUp", "3"),
    ("MaxRateDown", "7"),
    ("SteerThreshold", "150"),
    ("RecordingCount", "200"),
    ("RecordingQuality", "1"),
    ("CruiseGapAdjust", "0"),
    ("AutoEnable", "1"),
    ("CruiseAutoRes", "0"),
    ("AutoResOption", "0"),
    ("AutoResCondition", "0"),
    ("OpkrMonitoringMode", "0"),
    ("OpkrMonitorEyesThreshold", "45"),
    ("OpkrMonitorNormalEyesThreshold", "45"),
    ("OpkrMonitorBlinkThreshold", "35"),
    ("UFCModeEnabled", "1"),
    ("WhitePandaSupport", "0"),
    ("SteerWarningFix", "0"),
    ("OpkrRunNaviOnBoot", "0"),
    ("CruiseGap1", "12"),
    ("CruiseGap2", "13"),
    ("CruiseGap3", "14"),
    ("CruiseGap4", "16"),
    ("DynamicTRGap", "1"),
    ("DynamicTRSpd", "0,20,40,60,110"),
    ("DynamicTRSet", "1.2,1.3,1.4,1.5,1.6"),
    ("OpkrBattLess", "0"),
    ("LCTimingFactorUD", "1"),
    ("LCTimingFactor30", "10"),
    ("LCTimingFactor60", "40"),
    ("LCTimingFactor80", "60"),
    ("LCTimingFactor110", "80"),
    ("OpkrUIBrightnessOff", "10"),
    ("LCTimingFactorEnable", "1"),
    ("AutoEnableSpeed", "9"),
    ("SafetyCamDecelDistGain", "0"),
    ("OpkrLiveTunePanelEnable", "0"),
    ("RadarLongHelper", "2"),
    ("GitPullOnBoot", "0"),
    ("LiveSteerRatioPercent", "-5"),
    ("StoppingDistAdj", "0"),
    ("ShowError", "1"),
    ("AutoResLimitTime", "0"),
    ("VCurvSpeedC", "30,50,70,90"),
    ("VCurvSpeedT", "43,58,73,87"),
    ("OCurvSpeedC", "30,40,50,60,70"),
    ("OCurvSpeedT", "35,45,60,70,80"),
    ("OSMCustomSpeedLimitC", "30,40,50,60,70,90"),
    ("OSMCustomSpeedLimitT", "30,40,65,72,80,95"),
    ("StockNaviSpeedEnabled", "0"),
    ("OPKRNaviSelect", "4"),
    ("dp_atl", "1"),
    ("E2ELong", "0"),
    ("GoogleMapEnabled", "0"),
    ("OPKRServer", "0"),
    ("OPKRMapboxStyleSelect", "0"),
    ("IgnoreCANErroronISG", "0"),
    ("RESCountatStandstill", "19"),
    ("OpkrSpeedLimitOffsetOption", "0"),
    ("OpkrSpeedLimitSignType", "0"),
    ("StockLKASEnabled", "1"),
    ("SpeedLimitDecelOff", "0"),
    ("CurvDecelOption", "2"),
    ("FCA11Message", "0"),
    ("StandstillResumeAlt", "0"),
    ("MapboxEnabled", "0"),
    ("AutoRESDelay", "1"),
    ("UseRadarTrack", "0"),
    ("RadarDisable", "0"),
    ("DesiredCurvatureLimit", "10"),
    ("C2WithCommaPower", "0"),
    ("CustomTREnabled", "1"),
    ("RoadList", "RoadName1,+0.0,RoadName2,-0.0\nRoadName3,30,RoadName4,60"),
    ("LaneWidth", "37"),
    ("SpdLaneWidthSpd", "0,31"),
    ("SpdLaneWidthSet", "2.8,3.5"),
    ("TopTextView", "0"),
    ("CloseToRoadEdge", "0"),
    ("LeftEdgeOffset", "0"),
    ("RightEdgeOffset", "0"),
    ("AvoidLKASFaultEnabled", "0"),
    ("AvoidLKASFaultMaxAngle", "85"),
    ("AvoidLKASFaultMaxFrame", "90"),
    ("AvoidLKASFaultBeyond", "0"),
    ("UseStockDecelOnSS", "0"),
    ("AnimatedRPM", "1"),
    ("AnimatedRPMMax", "3600"),
    ("ShowStopLine", "0"),
    ("RoutineDriveOption", "OPKR"),
    ("SshEnabled", "1"),
    ("UserSpecificFeature", "0"),
    ("OpkrWakeUp", "0"),
    ("MultipleLateralUse", "2"),
    ("MultipleLateralOpS", "3,3,0"),
    ("MultipleLateralSpd", "60,90"),
    ("MultipleLateralOpA", "3,3,0"),
    ("MultipleLateralAng", "20,35"),
    ("StoppingDist", "38"),
    ("SpeedCameraOffset", "0"),
    ("HoldForSetting", "1"),
    ("RTShield", "1"),
    ("OSMOfflineUse", "0"),
    ("StopAtStopSign", "0"),
    ("VarCruiseSpeedFactor", "10"),
    ("LanguageSetting", "main_en"),
    ("OPKRSpeedBump", "0"),
    ("OPKREarlyStop", "1"),
    ("DoNotDisturbMode", "0"),
    ("DepartChimeAtResume", "0"),
    ("CommaStockUI", "0"),
    ("CruiseGapBySpdOn", "0"),
    ("CruiseGapBySpdSpd", "25,65,130"),
    ("CruiseGapBySpdGap", "1,2,3,4"),
    ("CruiseSetwithRoadLimitSpeedEnabled", "0"),
    ("CruiseSetwithRoadLimitSpeedOffset", "0"),
   ]
  if not PC:
    default_params.append(("LastUpdateTime", datetime.datetime.utcnow().isoformat().encode('utf8')))

  if params.get_bool("RecordFrontLock"):
    params.put_bool("RecordFront", True)

  if not params.get_bool("DisableRadar_Allow"):
    params.delete("DisableRadar")

  # set unset params
  for k, v in default_params:
    if params.get(k) is None:
      params.put(k, v)

  # is this dashcam?
  if os.getenv("PASSIVE") is not None:
    params.put_bool("Passive", bool(int(os.getenv("PASSIVE", "0"))))

  if params.get("Passive") is None:
    raise Exception("Passive must be set to continue")

  if EON:
    update_apks(show_spinner=True)
  # Create folders needed for msgq
  try:
    os.mkdir("/dev/shm")
  except FileExistsError:
    pass
  except PermissionError:
    print("WARNING: failed to make /dev/shm")

  # set version params
  params.put("Version", get_version())
  params.put("TermsVersion", terms_version)
  params.put("TrainingVersion", training_version)
  params.put("GitCommit", get_commit(default=""))
  params.put("GitBranch", get_short_branch(default=""))
  params.put("GitRemote", get_origin(default=""))

  # set dongle id
  reg_res = register(show_spinner=True)
  if reg_res:
    dongle_id = reg_res
  elif not reg_res:
    dongle_id = "maintenance"
  else:
    serial = params.get("HardwareSerial")
    raise Exception(f"Registration failed for device {serial}")
  os.environ['DONGLE_ID'] = dongle_id  # Needed for swaglog

  if not is_dirty():
    os.environ['CLEAN'] = '1'

  # init logging
  sentry.init(sentry.SentryProject.SELFDRIVE)
  cloudlog.bind_global(dongle_id=dongle_id, version=get_version(), dirty=is_dirty(),
                       device=HARDWARE.get_device_type())

  # opkr
  if os.path.isfile('/data/log/error.txt'):
    os.remove('/data/log/error.txt')
  if os.path.isfile('/data/log/can_missing.txt'):
    os.remove('/data/log/can_missing.txt')
  if os.path.isfile('/data/log/can_timeout.txt'):
    os.remove('/data/log/can_timeout.txt')

  # ensure shared libraries are readable by apks
  if EON:
    os.chmod(BASEDIR, 0o755)
    os.chmod("/dev/shm", 0o777)
    os.chmod(os.path.join(BASEDIR, "cereal"), 0o755)
    os.chmod(os.path.join(BASEDIR, "cereal", "libmessaging_shared.so"), 0o755)

def manager_prepare() -> None:
  for p in managed_processes.values():
    p.prepare()


def manager_cleanup() -> None:
  if EON:
    pm_apply_packages('disable')

  # send signals to kill all procs
  for p in managed_processes.values():
    p.stop(block=False)

  # ensure all are killed
  for p in managed_processes.values():
    p.stop(block=True)

  cloudlog.info("everything is dead")


def manager_thread() -> None:
  cloudlog.bind(daemon="manager")
  cloudlog.info("manager start")
  cloudlog.info({"environ": os.environ})

  params = Params()

  ignore: List[str] = []
  if params.get("DongleId", encoding='utf8') in (None, UNREGISTERED_DONGLE_ID):
    ignore += ["manage_athenad", "uploader"]
  if os.getenv("NOBOARD") is not None:
    ignore.append("pandad")
  ignore += [x for x in os.getenv("BLOCK", "").split(",") if len(x) > 0]

  if EON:
    pm_apply_packages('enable')

  ensure_running(managed_processes.values(), started=False, not_run=ignore)

  started_prev = False
  sm = messaging.SubMaster(['deviceState'])
  pm = messaging.PubMaster(['managerState'])

  while True:
    sm.update()
    not_run = ignore[:]

    started = sm['deviceState'].started
    driverview = params.get_bool("IsDriverViewEnabled")
    ensure_running(managed_processes.values(), started, driverview, not_run)

    # trigger an update after going offroad
    if started_prev and not started and 'updated' in managed_processes:
      os.sync()
      managed_processes['updated'].signal(signal.SIGHUP)

    started_prev = started

    running = ' '.join("%s%s\u001b[0m" % ("\u001b[32m" if p.proc.is_alive() else "\u001b[31m", p.name)
                       for p in managed_processes.values() if p.proc)
    print(running)
    cloudlog.debug(running)

    # send managerState
    msg = messaging.new_message('managerState')
    msg.managerState.processes = [p.get_process_state_msg() for p in managed_processes.values()]
    pm.send('managerState', msg)

    # Exit main loop when uninstall/shutdown/reboot is needed
    shutdown = False
    for param in ("DoUninstall", "DoShutdown", "DoReboot"):
      if params.get_bool(param):
        shutdown = True
        params.put("LastManagerExitReason", param)
        cloudlog.warning(f"Shutting down manager - {param} set")

    if shutdown:
      break


def main() -> None:
  prepare_only = os.getenv("PREPAREONLY") is not None

  manager_init()

  # Start UI early so prepare can happen in the background
  if not prepare_only:
    managed_processes['ui'].start()

  manager_prepare()

  if prepare_only:
    return

  # SystemExit on sigterm
  signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(1))

  try:
    manager_thread()
  except Exception:
    traceback.print_exc()
    sentry.capture_exception()
  finally:
    manager_cleanup()

  params = Params()
  if params.get_bool("DoUninstall"):
    cloudlog.warning("uninstalling")
    HARDWARE.uninstall()
  elif params.get_bool("DoReboot"):
    cloudlog.warning("reboot")
    HARDWARE.reboot()
  elif params.get_bool("DoShutdown"):
    cloudlog.warning("shutdown")
    HARDWARE.shutdown()


if __name__ == "__main__":
  unblock_stdout()

  try:
    main()
  except Exception:
    add_file_handler(cloudlog)
    cloudlog.exception("Manager failed to start")

    try:
      managed_processes['ui'].stop()
    except Exception:
      pass

    # Show last 3 lines of traceback
    error = traceback.format_exc(-3)
    error = "Manager failed to start\n\n" + error
    with TextWindow(error) as t:
      t.wait_for_exit()

    raise

  # manual exit because we are forked
  sys.exit(0)
