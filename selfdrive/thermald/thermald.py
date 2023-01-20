#!/usr/bin/env python3
import datetime
import os
import subprocess
import queue
import threading
import time
from collections import OrderedDict, namedtuple
from pathlib import Path
from typing import Dict, Optional, Tuple

import psutil

import cereal.messaging as messaging
from cereal import log
from common.dict_helpers import strip_deprecated_keys
from common.filter_simple import FirstOrderFilter

from common.numpy_fast import interp

from common.params import Params
from common.realtime import DT_TRML, sec_since_boot
from selfdrive.controls.lib.alertmanager import set_offroad_alert
from selfdrive.hardware import EON, TICI, PC, HARDWARE
from selfdrive.loggerd.config import get_available_percent
from selfdrive.swaglog import cloudlog
from selfdrive.thermald.power_monitoring import PowerMonitoring
from selfdrive.thermald.fan_controller import EonFanController, UnoFanController, TiciFanController
from selfdrive.version import terms_version, training_version


ThermalStatus = log.DeviceState.ThermalStatus
NetworkType = log.DeviceState.NetworkType
NetworkStrength = log.DeviceState.NetworkStrength
CURRENT_TAU = 15.   # 15s time constant
TEMP_TAU = 5.   # 5s time constant
DISCONNECT_TIMEOUT = 3.  # wait 5 seconds before going offroad after disconnect so you get an alert
PANDA_STATES_TIMEOUT = int(1000 * 1.5 * DT_TRML)  # 1.5x the expected pandaState frequency

ThermalBand = namedtuple("ThermalBand", ['min_temp', 'max_temp'])
HardwareState = namedtuple("HardwareState", ['network_type', 'network_metered', 'network_strength', 'network_info', 'nvme_temps', 'modem_temps', 'wifi_address', 'connect_name', 'rsrp'])

# List of thermal bands. We will stay within this region as long as we are within the bounds.
# When exiting the bounds, we'll jump to the lower or higher band. Bands are ordered in the dict.
THERMAL_BANDS = OrderedDict({
  ThermalStatus.green: ThermalBand(None, 80.0),
  ThermalStatus.yellow: ThermalBand(75.0, 96.0),
  ThermalStatus.red: ThermalBand(80.0, 107.),
  ThermalStatus.danger: ThermalBand(94.0, None),
})

# Override to highest thermal band when offroad and above this temp
OFFROAD_DANGER_TEMP = 79.5 if TICI else 70.0

prev_offroad_states: Dict[str, Tuple[bool, Optional[str]]] = {}

mediaplayer = '/data/openpilot/selfdrive/assets/addon/mediaplayer/'
prebuiltfile = '/data/openpilot/prebuilt'
sshkeyfile = '/data/public_key'
pandaflash_ongoing = '/data/openpilot/pandaflash_ongoing'

tz_by_type: Optional[Dict[str, int]] = None
def populate_tz_by_type():
  global tz_by_type
  tz_by_type = {}
  for n in os.listdir("/sys/devices/virtual/thermal"):
    if not n.startswith("thermal_zone"):
      continue
    with open(os.path.join("/sys/devices/virtual/thermal", n, "type")) as f:
      tz_by_type[f.read().strip()] = int(n.lstrip("thermal_zone"))

def read_tz(x):
  if x is None:
    return 0

  if isinstance(x, str):
    if tz_by_type is None:
      populate_tz_by_type()
    x = tz_by_type[x]

  try:
    with open(f"/sys/devices/virtual/thermal/thermal_zone{x}/temp") as f:
      return int(f.read())
  except FileNotFoundError:
    return 0


def read_thermal(thermal_config):
  dat = messaging.new_message('deviceState')
  dat.deviceState.cpuTempC = [read_tz(z) / thermal_config.cpu[1] for z in thermal_config.cpu[0]]
  dat.deviceState.gpuTempC = [read_tz(z) / thermal_config.gpu[1] for z in thermal_config.gpu[0]]
  dat.deviceState.memoryTempC = read_tz(thermal_config.mem[0]) / thermal_config.mem[1]
  dat.deviceState.ambientTempC = read_tz(thermal_config.ambient[0]) / thermal_config.ambient[1]
  dat.deviceState.pmicTempC = [read_tz(z) / thermal_config.pmic[1] for z in thermal_config.pmic[0]]
  dat.deviceState.batteryTempC = read_tz(thermal_config.bat[0]) / thermal_config.bat[1]
  return dat


def set_offroad_alert_if_changed(offroad_alert: str, show_alert: bool, extra_text: Optional[str]=None):
  if prev_offroad_states.get(offroad_alert, None) == (show_alert, extra_text):
    return
  prev_offroad_states[offroad_alert] = (show_alert, extra_text)
  set_offroad_alert(offroad_alert, show_alert, extra_text)


def hw_state_thread(end_event, hw_queue):
  """Handles non critical hardware state, and sends over queue"""
  count = 0
  registered_count = 0
  prev_hw_state = None

  modem_version = None
  modem_nv = None
  modem_configured = False

  while not end_event.is_set():
    # these are expensive calls. update every 10s
    if (count % int(10. / DT_TRML)) == 0:
      try:
        network_type = HARDWARE.get_network_type()
        modem_temps = HARDWARE.get_modem_temperatures()
        if len(modem_temps) == 0 and prev_hw_state is not None:
          modem_temps = prev_hw_state.modem_temps

        # Log modem version once
        if TICI and ((modem_version is None) or (modem_nv is None)):
          modem_version = HARDWARE.get_modem_version()  # pylint: disable=assignment-from-none
          modem_nv = HARDWARE.get_modem_nv()  # pylint: disable=assignment-from-none

          if (modem_version is not None) and (modem_nv is not None):
            cloudlog.event("modem version", version=modem_version, nv=modem_nv)

        network_strength_, connect_name_, rsrp_ = HARDWARE.get_network_strength(network_type)
        hw_state = HardwareState(
          network_type=network_type,
          network_metered=HARDWARE.get_network_metered(network_type),
          network_strength=network_strength_,
          network_info=HARDWARE.get_network_info(),
          nvme_temps=HARDWARE.get_nvme_temperatures(),
          modem_temps=modem_temps,
          wifi_address=HARDWARE.get_ip_address(),
          connect_name = connect_name_,
          rsrp = rsrp_,
        )

        try:
          hw_queue.put_nowait(hw_state)
        except queue.Full:
          pass

        if TICI and (hw_state.network_info is not None) and (hw_state.network_info.get('state', None) == "REGISTERED"):
          registered_count += 1
        else:
          registered_count = 0

        if registered_count > 10:
          cloudlog.warning(f"Modem stuck in registered state {hw_state.network_info}. nmcli conn up lte")
          os.system("nmcli conn up lte")
          registered_count = 0

        # TODO: remove this once the config is in AGNOS
        if not modem_configured and len(HARDWARE.get_sim_info().get('sim_id', '')) > 0:
          cloudlog.warning("configuring modem")
          HARDWARE.configure_modem()
          modem_configured = True

        prev_hw_state = hw_state
      except Exception:
        cloudlog.exception("Error getting hardware state")

    count += 1
    time.sleep(DT_TRML)


def thermald_thread(end_event, hw_queue):
  pm = messaging.PubMaster(['deviceState'])
  sm = messaging.SubMaster(["peripheralState", "gpsLocationExternal", "controlsState", "pandaStates"], poll=["pandaStates"])

  count = 0

  onroad_conditions: Dict[str, bool] = {
    "ignition": False,
  }
  startup_conditions: Dict[str, bool] = {}
  startup_conditions_prev: Dict[str, bool] = {}

  off_ts = None
  started_ts = None
  started_seen = False
  thermal_status = ThermalStatus.green
  usb_power = True

  last_hw_state = HardwareState(
    network_type=NetworkType.none,
    network_metered=False,
    network_strength=NetworkStrength.unknown,
    network_info=None,
    nvme_temps=[],
    modem_temps=[],
    wifi_address = "N/A",
    connect_name = "---",
    rsrp = "--",
  )

  current_filter = FirstOrderFilter(0., CURRENT_TAU, DT_TRML)
  temp_filter = FirstOrderFilter(0., TEMP_TAU, DT_TRML)
  should_start_prev = False
  in_car = False
  is_uno = False
  engaged_prev = False

  params = Params()
  power_monitor = PowerMonitoring()

  HARDWARE.initialize_hardware()
  thermal_config = HARDWARE.get_thermal_config()

  fan_controller = None

  # sound trigger
  sound_trigger = 1
  opkrAutoShutdown = 0

  shutdown_trigger = 1
  is_openpilot_view_enabled = 0

  env = dict(os.environ)
  env['LD_LIBRARY_PATH'] = mediaplayer

  getoff_alert = int(params.get("OpkrEnableGetoffAlert", encoding="utf8"))

  hotspot_on_boot = params.get_bool("OpkrHotspotOnBoot")
  hotspot_run = False

  opkrAutoShutdown = interp(int(params.get("OpkrAutoShutdown", encoding="utf8")), [0,1,2,3,4,5,6,7,8,9], [0,5,30,60,180,300,600,1800,3600,10800])
  battery_charging_control = params.get_bool("OpkrBatteryChargingControl")
  battery_charging_min = int(params.get("OpkrBatteryChargingMin", encoding="utf8"))
  battery_charging_max = int(params.get("OpkrBatteryChargingMax", encoding="utf8"))

  c2withCommaPower = params.get_bool("C2WithCommaPower")

  is_openpilot_dir = True
  wakeuprunning = False
  onroadrefresh = False

  ts = sec_since_boot()

  while not end_event.is_set():
    sm.update(PANDA_STATES_TIMEOUT)

    pandaStates = sm['pandaStates']
    peripheralState = sm['peripheralState']

    msg = read_thermal(thermal_config)

    if sm.updated['pandaStates'] and len(pandaStates) > 0:

      ts = sec_since_boot()

      # Set ignition based on any panda connected
      onroad_conditions["ignition"] = any(ps.ignitionLine or ps.ignitionCan for ps in pandaStates if ps.pandaType != log.PandaState.PandaType.unknown)

      pandaState = pandaStates[0]

      if pandaState.pandaType != log.PandaState.PandaType.unknown:
        shutdown_trigger = 1
      else:
        sound_trigger == 1

      in_car = pandaState.harnessStatus != log.PandaState.HarnessStatus.notConnected
      usb_power = peripheralState.usbPowerMode != log.PeripheralState.UsbPowerMode.client

      # Setup fan handler on first connect to panda
      if fan_controller is None and peripheralState.pandaType != log.PandaState.PandaType.unknown:
        is_uno = peripheralState.pandaType == log.PandaState.PandaType.uno
        if TICI:
          fan_controller = TiciFanController()
        elif is_uno or PC:
          fan_controller = UnoFanController()
        else:
          fan_controller = EonFanController()
    elif params.get_bool("IsOpenpilotViewEnabled") and not params.get_bool("IsDriverViewEnabled") and is_openpilot_view_enabled == 0:
      is_openpilot_view_enabled = 1
      onroad_conditions["ignition"] = True
    elif not params.get_bool("IsOpenpilotViewEnabled") and not params.get_bool("IsDriverViewEnabled") and is_openpilot_view_enabled == 1:
      shutdown_trigger = 0
      sound_trigger == 0
      is_openpilot_view_enabled = 0
      onroad_conditions["ignition"] = False
      fan_controller = None
    elif not is_openpilot_view_enabled:
      if sec_since_boot() - ts > DISCONNECT_TIMEOUT:
        if onroad_conditions["ignition"]:
          cloudlog.error("Lost panda connection while onroad")
        onroad_conditions["ignition"] = False

    try:
      last_hw_state = hw_queue.get_nowait()
    except queue.Empty:
      pass

    # these are expensive calls. update every 10s
    if (count % int(10. / DT_TRML)) == 0:
      try:
        ping_test = subprocess.check_output(["ping", "-c", "1", "-W", "1", "google.com"])
        Params().put("LastAthenaPingTime", str(int(sec_since_boot() * 1e9))) if ping_test else False
      except Exception:
        Params().delete("LastAthenaPingTime")

    msg.deviceState.freeSpacePercent = get_available_percent(default=100.0)
    msg.deviceState.memoryUsagePercent = int(round(psutil.virtual_memory().percent))
    msg.deviceState.cpuUsagePercent = [int(round(n)) for n in psutil.cpu_percent(percpu=True)]
    msg.deviceState.gpuUsagePercent = int(round(HARDWARE.get_gpu_usage_percent()))

    msg.deviceState.networkType = last_hw_state.network_type
    msg.deviceState.networkMetered = last_hw_state.network_metered
    msg.deviceState.networkStrength = last_hw_state.network_strength
    if last_hw_state.network_info is not None:
      msg.deviceState.networkInfo = last_hw_state.network_info

    msg.deviceState.nvmeTempC = last_hw_state.nvme_temps
    msg.deviceState.modemTempC = last_hw_state.modem_temps
    msg.deviceState.wifiIpAddress = last_hw_state.wifi_address
    msg.deviceState.connectName = last_hw_state.connect_name
    msg.deviceState.rSRP = last_hw_state.rsrp

    msg.deviceState.screenBrightnessPercent = HARDWARE.get_screen_brightness()
    msg.deviceState.batteryPercent = HARDWARE.get_battery_capacity()
    msg.deviceState.batteryStatus = HARDWARE.get_battery_status()
    msg.deviceState.batteryCurrent = HARDWARE.get_battery_current()
    msg.deviceState.batteryVoltage = HARDWARE.get_battery_voltage()
    msg.deviceState.usbOnline = HARDWARE.get_usb_present()
    current_filter.update(msg.deviceState.batteryCurrent / 1e6)

    max_comp_temp = temp_filter.update(
      max(max(msg.deviceState.cpuTempC), msg.deviceState.memoryTempC, max(msg.deviceState.gpuTempC))
    )

    bat_temp = msg.deviceState.batteryTempC

    if fan_controller is not None:
      msg.deviceState.fanSpeedPercentDesired = fan_controller.update(max_comp_temp, bat_temp, onroad_conditions["ignition"])

    is_offroad_for_5_min = (started_ts is None) and ((not started_seen) or (off_ts is None) or (sec_since_boot() - off_ts > 60 * 5))
    if is_offroad_for_5_min and max_comp_temp > OFFROAD_DANGER_TEMP:
      # If device is offroad we want to cool down before going onroad
      # since going onroad increases load and can make temps go over 107
      thermal_status = ThermalStatus.danger
    else:
      current_band = THERMAL_BANDS[thermal_status]
      band_idx = list(THERMAL_BANDS.keys()).index(thermal_status)
      if current_band.min_temp is not None and max_comp_temp < current_band.min_temp:
        thermal_status = list(THERMAL_BANDS.keys())[band_idx - 1]
      elif current_band.max_temp is not None and max_comp_temp > current_band.max_temp:
        thermal_status = list(THERMAL_BANDS.keys())[band_idx + 1]

    # **** starting logic ****

    # Ensure date/time are valid
    now = datetime.datetime.utcnow()
    startup_conditions["time_valid"] = True if ((now.year > 2020) or (now.year == 2020 and now.month >= 10)) else True # set True for battery less EON otherwise, set False.
    set_offroad_alert_if_changed("Offroad_InvalidTime", (not startup_conditions["time_valid"]))

    # Show update prompt
    # try:
    #   last_update = datetime.datetime.fromisoformat(params.get("LastUpdateTime", encoding='utf8'))
    # except (TypeError, ValueError):
    #   last_update = now
    # dt = now - last_update

    # update_failed_count = params.get("UpdateFailedCount")
    # update_failed_count = 0 if update_failed_count is None else int(update_failed_count)
    # last_update_exception = params.get("LastUpdateException", encoding='utf8')

    # if update_failed_count > 15 and last_update_exception is not None:
    #   if current_branch in ["release2", "dashcam"]:
    #     extra_text = "Ensure the software is correctly installed"
    #   else:
    #     extra_text = last_update_exception

    #   set_offroad_alert_if_changed("Offroad_ConnectivityNeeded", False)
    #   set_offroad_alert_if_changed("Offroad_ConnectivityNeededPrompt", False)
    #   set_offroad_alert_if_changed("Offroad_UpdateFailed", True, extra_text=extra_text)
    # elif dt.days > DAYS_NO_CONNECTIVITY_MAX and update_failed_count > 1:
    #   set_offroad_alert_if_changed("Offroad_UpdateFailed", False)
    #   set_offroad_alert_if_changed("Offroad_ConnectivityNeededPrompt", False)
    #   set_offroad_alert_if_changed("Offroad_ConnectivityNeeded", True)
    # elif dt.days > DAYS_NO_CONNECTIVITY_PROMPT:
    #   remaining_time = str(max(DAYS_NO_CONNECTIVITY_MAX - dt.days, 0))
    #   set_offroad_alert_if_changed("Offroad_UpdateFailed", False)
    #   set_offroad_alert_if_changed("Offroad_ConnectivityNeeded", False)
    #   set_offroad_alert_if_changed("Offroad_ConnectivityNeededPrompt", True, extra_text=f"{remaining_time} days.")
    # else:
    #   set_offroad_alert_if_changed("Offroad_UpdateFailed", False)
    #   set_offroad_alert_if_changed("Offroad_ConnectivityNeeded", False)
    #   set_offroad_alert_if_changed("Offroad_ConnectivityNeededPrompt", False)

    #startup_conditions["up_to_date"] = params.get("Offroad_ConnectivityNeeded") is None or params.get_bool("DisableUpdates")
    startup_conditions["not_uninstalling"] = not params.get_bool("DoUninstall")
    startup_conditions["accepted_terms"] = params.get("HasAcceptedTerms") == terms_version

    # with 2% left, we killall, otherwise the phone will take a long time to boot
    startup_conditions["free_space"] = msg.deviceState.freeSpacePercent > 2
    startup_conditions["completed_training"] = params.get("CompletedTrainingVersion") == training_version or \
                                               params.get_bool("Passive")
    startup_conditions["not_driver_view"] = not params.get_bool("IsDriverViewEnabled")
    startup_conditions["not_taking_snapshot"] = not params.get_bool("IsTakingSnapshot")
    # if any CPU gets above 107 or the battery gets above 63, kill all processes
    # controls will warn with CPU above 95 or battery above 60
    onroad_conditions["device_temp_good"] = thermal_status < ThermalStatus.danger
    set_offroad_alert_if_changed("Offroad_TemperatureTooHigh", (not onroad_conditions["device_temp_good"]))

    # TODO: this should move to TICI.initialize_hardware, but we currently can't import params there
    if TICI:
      if not os.path.isfile("/persist/comma/living-in-the-moment"):
        if not Path("/data/media").is_mount():
          set_offroad_alert_if_changed("Offroad_StorageMissing", True)
        else:
          # check for bad NVMe
          try:
            with open("/sys/block/nvme0n1/device/model") as f:
              model = f.read().strip()
            if not model.startswith("Samsung SSD 980") and params.get("Offroad_BadNvme") is None:
              set_offroad_alert_if_changed("Offroad_BadNvme", True)
              cloudlog.event("Unsupported NVMe", model=model, error=True)
          except Exception:
            pass

    if params.get_bool("OnRoadRefresh"):
      onroad_conditions["onroad_refresh"] = not params.get_bool("OnRoadRefresh")
      onroadrefresh = True
    elif onroadrefresh:
       onroadrefresh = False
       onroad_conditions["onroad_refresh"] = True

    # Handle offroad/onroad transition
    should_start = all(onroad_conditions.values())
    if started_ts is None:
      should_start = should_start and all(startup_conditions.values())

    if should_start != should_start_prev or (count == 0):
      params.put_bool("IsOnroad", should_start)
      params.put_bool("IsOffroad", not should_start)

      params.put_bool("IsEngaged", False)
      engaged_prev = False
      HARDWARE.set_power_save(not should_start)

    if sm.updated['controlsState']:
      engaged = sm['controlsState'].enabled
      if engaged != engaged_prev:
        params.put_bool("IsEngaged", engaged)
        engaged_prev = engaged

      try:
        with open('/dev/kmsg', 'w') as kmsg:
          kmsg.write(f"<3>[thermald] engaged: {engaged}\n")
      except Exception:
        pass

    if should_start:
      off_ts = None
      if started_ts is None:
        started_ts = sec_since_boot()
        started_seen = True
    else:
      if onroad_conditions["ignition"] and (startup_conditions != startup_conditions_prev):
        cloudlog.event("Startup blocked", startup_conditions=startup_conditions, onroad_conditions=onroad_conditions)

      started_ts = None
      if off_ts is None:
        off_ts = sec_since_boot()

      if shutdown_trigger == 1 and sound_trigger == 1 and msg.deviceState.batteryStatus == "Discharging" and started_seen and (sec_since_boot() - off_ts) > 1 and getoff_alert == 1:
        subprocess.Popen([mediaplayer + 'mediaplayer', '/data/openpilot/selfdrive/assets/addon/sound/eondetach_ko.wav'], shell = False, stdin=None, stdout=None, stderr=None, env = env, close_fds=True)
        sound_trigger = 0
      elif shutdown_trigger == 1 and sound_trigger == 1 and msg.deviceState.batteryStatus == "Discharging" and started_seen and (sec_since_boot() - off_ts) > 1 and getoff_alert == 2:
        subprocess.Popen([mediaplayer + 'mediaplayer', '/data/openpilot/selfdrive/assets/addon/sound/eondetach_en.wav'], shell = False, stdin=None, stdout=None, stderr=None, env = env, close_fds=True)
        sound_trigger = 0
      # shutdown if the battery gets lower than 3%, it's discharging, we aren't running for
      # more than a minute but we were running
      if shutdown_trigger == 1 and msg.deviceState.batteryStatus == "Discharging" and \
         started_seen and opkrAutoShutdown and (sec_since_boot() - off_ts) > opkrAutoShutdown and not os.path.isfile(pandaflash_ongoing):
        HARDWARE.shutdown()

      if (count % int(1. / DT_TRML)) == 0:
        if int(params.get("OpkrForceShutdown", encoding="utf8")) != 0 and not started_seen and msg.deviceState.batteryStatus == "Discharging":
          opkrForceShutdown = interp(int(params.get("OpkrForceShutdown", encoding="utf8")), [0,1,2,3,4,5], [0,60,180,300,600,1800])
          if (sec_since_boot() - off_ts) > opkrForceShutdown and opkrForceShutdown and params.get_bool("OpkrForceShutdownTrigger"):
            HARDWARE.shutdown()
          elif not params.get_bool("OpkrForceShutdownTrigger"):
            off_ts = sec_since_boot()
        elif msg.deviceState.batteryPercent < 10 and not started_seen and msg.deviceState.batteryStatus == "Discharging":
          HARDWARE.shutdown()

    # opkr
    prebuiltlet = params.get_bool("PutPrebuiltOn")
    if not os.path.isdir("/data/openpilot"):
      if is_openpilot_dir:
        os.system("cd /data/params/d; rm -f DongleId") # Delete DongleID if the Openpilot directory disappears, Seems you want to switch fork/branch.
      is_openpilot_dir = False
    elif not os.path.isfile(prebuiltfile) and prebuiltlet and is_openpilot_dir:
      os.system("cd /data/openpilot; touch prebuilt")
    elif os.path.isfile(prebuiltfile) and not prebuiltlet:
      os.system("cd /data/openpilot; rm -f prebuilt")

    # opkr
    sshkeylet = params.get_bool("OpkrSSHLegacy")
    if not os.path.isfile(sshkeyfile) and sshkeylet:
      os.system("cp -f /data/openpilot/selfdrive/assets/addon/key/GithubSshKeys_legacy /data/params/d/GithubSshKeys; chmod 600 /data/params/d/GithubSshKeys; touch /data/public_key")
    elif os.path.isfile(sshkeyfile) and not sshkeylet:
      os.system("cp -f /data/openpilot/selfdrive/assets/addon/key/GithubSshKeys_new /data/params/d/GithubSshKeys; chmod 600 /data/params/d/GithubSshKeys; rm -f /data/public_key")

    # opkr hotspot
    if hotspot_on_boot and not hotspot_run and sec_since_boot() > 80:
      os.system("service call wifi 37 i32 0 i32 1 &")
      hotspot_run = True

    opkrwakeup = params.get_bool("OpkrWakeUp")
    if opkrwakeup and not wakeuprunning:
      cmd1 = '/data/openpilot/selfdrive/assets/addon/sound/wakeup.wav'
      wakeuprunning = True
      wakeupstarted = sec_since_boot()
      subprocess.Popen([mediaplayer + 'mediaplayer', cmd1], shell = False, stdin=None, stdout=None, stderr=None, env = env, close_fds=True)
    elif wakeuprunning:
      if not opkrwakeup:
        wakeuprunning = False
        os.system("pkill -f mediaplayer")
      elif sec_since_boot() - wakeupstarted > 40:
        wakeuprunning = False
        Params().put_bool("OpkrWakeUp", False)
        os.system("pkill -f mediaplayer")

    # Offroad power monitoring
    power_monitor.calculate(peripheralState, onroad_conditions["ignition"])
    msg.deviceState.offroadPowerUsageUwh = power_monitor.get_power_used()
    msg.deviceState.carBatteryCapacityUwh = max(0, power_monitor.get_car_battery_capacity())

    if c2withCommaPower:
      current_power_draw = HARDWARE.get_current_power_draw()  # pylint: disable=assignment-from-none
      if current_power_draw is not None:
        msg.deviceState.powerDrawW = current_power_draw
      else:
        msg.deviceState.powerDrawW = 0
  
      # Check if we need to disable charging (handled by boardd)
      msg.deviceState.chargingDisabled = power_monitor.should_disable_charging(onroad_conditions["ignition"], in_car, off_ts)
  
      # Check if we need to shut down
      if power_monitor.should_shutdown(peripheralState, onroad_conditions["ignition"], in_car, off_ts, started_seen):
        cloudlog.warning(f"shutting device down, offroad since {off_ts}")
        params.put_bool("DoShutdown", True)
      
    msg.deviceState.chargingError = current_filter.x > 0. and msg.deviceState.batteryPercent < 90  # if current is positive, then battery is being discharged
    msg.deviceState.started = started_ts is not None
    msg.deviceState.startedMonoTime = int(1e9*(started_ts or 0))

    last_ping = params.get("LastAthenaPingTime")
    if last_ping is not None:
      msg.deviceState.lastAthenaPingTime = int(last_ping)

    msg.deviceState.thermalStatus = thermal_status
    pm.send("deviceState", msg)

    if EON and not is_uno:
      set_offroad_alert_if_changed("Offroad_ChargeDisabled", (not usb_power))

    should_start_prev = should_start
    startup_conditions_prev = startup_conditions.copy()

    # atom
    if usb_power and battery_charging_control:
      power_monitor.charging_ctrl(msg, sec_since_boot(), battery_charging_max, battery_charging_min)

    # report to server once every 10 minutes
    if (count % int(600. / DT_TRML)) == 0:
      if EON and started_ts is None and msg.deviceState.memoryUsagePercent > 40:
        cloudlog.event("High offroad memory usage", mem=msg.deviceState.memoryUsagePercent)

      cloudlog.event("STATUS_PACKET",
                     count=count,
                     pandaStates=[strip_deprecated_keys(p.to_dict()) for p in pandaStates],
                     peripheralState=strip_deprecated_keys(peripheralState.to_dict()),
                     location=(strip_deprecated_keys(sm["gpsLocationExternal"].to_dict()) if sm.alive["gpsLocationExternal"] else None),
                     deviceState=strip_deprecated_keys(msg.to_dict()))

    count += 1


def main():
  hw_queue = queue.Queue(maxsize=1)
  end_event = threading.Event()

  threads = [
    threading.Thread(target=hw_state_thread, args=(end_event, hw_queue)),
    threading.Thread(target=thermald_thread, args=(end_event, hw_queue)),
  ]

  for t in threads:
    t.start()

  try:
    while True:
      time.sleep(1)
      if not all(t.is_alive() for t in threads):
        break
  finally:
    end_event.set()

  for t in threads:
    t.join()


if __name__ == "__main__":
  main()
