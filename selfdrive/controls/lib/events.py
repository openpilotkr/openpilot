import os
from enum import IntEnum
from typing import Dict, Union, Callable, List, Optional

import linecache

from cereal import log, car
import cereal.messaging as messaging
from common.realtime import DT_CTRL
from common.conversions import Conversions as CV
from selfdrive.locationd.calibrationd import MIN_SPEED_FILTER

from common.params import Params

AlertSize = log.ControlsState.AlertSize
AlertStatus = log.ControlsState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
EventName = car.CarEvent.EventName


# Alert priorities
class Priority(IntEnum):
  LOWEST = 0
  LOWER = 1
  LOW = 2
  MID = 3
  HIGH = 4
  HIGHEST = 5


# Event types
class ET:
  ENABLE = 'enable'
  PRE_ENABLE = 'preEnable'
  NO_ENTRY = 'noEntry'
  WARNING = 'warning'
  USER_DISABLE = 'userDisable'
  SOFT_DISABLE = 'softDisable'
  IMMEDIATE_DISABLE = 'immediateDisable'
  PERMANENT = 'permanent'


# get event name from enum
EVENT_NAME = {v: k for k, v in EventName.schema.enumerants.items()}

try:
  LANG_FILE='/data/openpilot/selfdrive/assets/addon/lang/events/' + Params().get("LanguageSetting", encoding="utf8") + '.txt'
except:
  LANG_FILE='/data/openpilot/selfdrive/assets/addon/lang/events/main_en.txt'
  pass

# opkr
def tr(line_num: int):
  return linecache.getline(LANG_FILE, line_num)

class Events:
  def __init__(self):
    self.events: List[int] = []
    self.static_events: List[int] = []
    self.events_prev = dict.fromkeys(EVENTS.keys(), 0)

  @property
  def names(self) -> List[int]:
    return self.events

  def __len__(self) -> int:
    return len(self.events)

  def add(self, event_name: int, static: bool=False) -> None:
    if static:
      self.static_events.append(event_name)
    self.events.append(event_name)

  def clear(self) -> None:
    self.events_prev = {k: (v + 1 if k in self.events else 0) for k, v in self.events_prev.items()}
    self.events = self.static_events.copy()

  def any(self, event_type: str) -> bool:
    return any(event_type in EVENTS.get(e, {}) for e in self.events)

  def create_alerts(self, event_types: List[str], callback_args=None):
    if callback_args is None:
      callback_args = []

    ret = []
    for e in self.events:
      types = EVENTS[e].keys()
      for et in event_types:
        if et in types:
          alert = EVENTS[e][et]
          if not isinstance(alert, Alert):
            alert = alert(*callback_args)

          if DT_CTRL * (self.events_prev[e] + 1) >= alert.creation_delay:
            alert.alert_type = f"{EVENT_NAME[e]}/{et}"
            alert.event_type = et
            ret.append(alert)
    return ret

  def add_from_msg(self, events):
    for e in events:
      self.events.append(e.name.raw)

  def to_msg(self):
    ret = []
    for event_name in self.events:
      event = car.CarEvent.new_message()
      event.name = event_name
      for event_type in EVENTS.get(event_name, {}):
        setattr(event, event_type, True)
      ret.append(event)
    return ret


class Alert:
  def __init__(self,
               alert_text_1: str,
               alert_text_2: str,
               alert_status: log.ControlsState.AlertStatus,
               alert_size: log.ControlsState.AlertSize,
               priority: Priority,
               visual_alert: car.CarControl.HUDControl.VisualAlert,
               audible_alert: car.CarControl.HUDControl.AudibleAlert,
               duration: float,
               alert_rate: float = 0.,
               creation_delay: float = 0.):

    self.alert_text_1 = alert_text_1
    self.alert_text_2 = alert_text_2
    self.alert_status = alert_status
    self.alert_size = alert_size
    self.priority = priority
    self.visual_alert = visual_alert
    self.audible_alert = audible_alert

    self.duration = int(duration / DT_CTRL)

    self.alert_rate = alert_rate
    self.creation_delay = creation_delay

    self.alert_type = ""
    self.event_type: Optional[str] = None

  def __str__(self) -> str:
    return f"{self.alert_text_1}/{self.alert_text_2} {self.priority} {self.visual_alert} {self.audible_alert}"

  def __gt__(self, alert2) -> bool:
    return self.priority > alert2.priority


class NoEntryAlert(Alert):
  def __init__(self, alert_text_2: str, visual_alert: car.CarControl.HUDControl.VisualAlert=VisualAlert.none):
    super().__init__(tr(1), alert_text_2, AlertStatus.normal,
                     AlertSize.mid, Priority.LOW, visual_alert,
                     AudibleAlert.refuse, 3.)


class SoftDisableAlert(Alert):
  def __init__(self, alert_text_2: str):
    super().__init__(tr(2), alert_text_2,
                     AlertStatus.userPrompt, AlertSize.full,
                     Priority.MID, VisualAlert.steerRequired,
                     AudibleAlert.warningSoft, 2.),


# less harsh version of SoftDisable, where the condition is user-triggered
class UserSoftDisableAlert(SoftDisableAlert):
  def __init__(self, alert_text_2: str):
    super().__init__(alert_text_2),
    self.alert_text_1 = tr(3)


class ImmediateDisableAlert(Alert):
  def __init__(self, alert_text_2: str):
    super().__init__(tr(4), alert_text_2,
                     AlertStatus.critical, AlertSize.full,
                     Priority.HIGHEST, VisualAlert.steerRequired,
                     AudibleAlert.warningImmediate, 4.),


class EngagementAlert(Alert):
  def __init__(self, audible_alert: car.CarControl.HUDControl.AudibleAlert):
    super().__init__("", "",
                     AlertStatus.normal, AlertSize.none,
                     Priority.MID, VisualAlert.none,
                     audible_alert, .2),


class NormalPermanentAlert(Alert):
  def __init__(self, alert_text_1: str, alert_text_2: str = "", duration: float = 0.2, priority: Priority = Priority.LOWER, creation_delay: float = 0.):
    super().__init__(alert_text_1, alert_text_2,
                     AlertStatus.normal, AlertSize.mid if len(alert_text_2) else AlertSize.small,
                     priority, VisualAlert.none, AudibleAlert.none, duration, creation_delay=creation_delay),


class StartupAlert(Alert):
  def __init__(self, alert_text_1: str, alert_text_2: str = tr(5), alert_status=AlertStatus.normal):
    super().__init__(alert_text_1, alert_text_2,
                     alert_status, AlertSize.mid,
                     Priority.LOWER, VisualAlert.none, AudibleAlert.none, 10.),


# ********** helper functions **********
def get_display_speed(speed_ms: float, metric: bool) -> str:
  speed = int(round(speed_ms * (CV.MS_TO_KPH if metric else CV.MS_TO_MPH)))
  unit = 'km/h' if metric else 'mph'
  return f"{speed} {unit}"


# ********** alert callback functions **********

AlertCallbackType = Callable[[car.CarParams, messaging.SubMaster, bool, int], Alert]


def soft_disable_alert(alert_text_2: str) -> AlertCallbackType:
  def func(CP: car.CarParams, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
    if soft_disable_time < int(0.5 / DT_CTRL):
      return ImmediateDisableAlert(alert_text_2)
    return SoftDisableAlert(alert_text_2)
  return func

def user_soft_disable_alert(alert_text_2: str) -> AlertCallbackType:
  def func(CP: car.CarParams, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
    if soft_disable_time < int(0.5 / DT_CTRL):
      return ImmediateDisableAlert(alert_text_2)
    return UserSoftDisableAlert(alert_text_2)
  return func


def below_engage_speed_alert(CP: car.CarParams, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return NoEntryAlert(f"Speed Below {get_display_speed(CP.minEnableSpeed, metric)}")


def below_steer_speed_alert(CP: car.CarParams, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return Alert(
    f"Steer Unavailable Below {get_display_speed(CP.minSteerSpeed, metric)}",
    "",
    AlertStatus.userPrompt, AlertSize.small,
    Priority.MID, VisualAlert.none, AudibleAlert.prompt, 0.4)


def calibration_incomplete_alert(CP: car.CarParams, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return Alert(
    "Calibration in Progress: %d%%" % sm['liveCalibration'].calPerc,
    f"Drive Above {get_display_speed(MIN_SPEED_FILTER, metric)}",
    AlertStatus.normal, AlertSize.mid,
    Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .2)


def no_gps_alert(CP: car.CarParams, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  gps_integrated = sm['peripheralState'].pandaType in (log.PandaState.PandaType.uno, log.PandaState.PandaType.dos)
  return Alert(
    tr(10),
    tr(11) if gps_integrated else tr(12),
    AlertStatus.normal, AlertSize.mid,
    Priority.LOWER, VisualAlert.none, AudibleAlert.none, .2, creation_delay=300.)


def wrong_car_mode_alert(CP: car.CarParams, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  text = tr(13)
  if CP.carName == "honda":
    text = tr(14)
  return NoEntryAlert(text)


def joystick_alert(CP: car.CarParams, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  axes = sm['testJoystick'].axes
  gb, steer = list(axes)[:2] if len(axes) else (0., 0.)
  vals = f"Gas: {round(gb * 100.)}%, Steer: {round(steer * 100.)}%"
  return NormalPermanentAlert("Joystick Mode", vals)

# opkr
def can_error_alert(CP: car.CarParams, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  if os.path.isfile('/data/log/can_missing.txt'):
    f = open('/data/log/can_missing.txt', 'r')
    add = f.readline()
    add_int = int(add, 0)
    f.close()
    return Alert(
      "CAN Error: %s is missing\n Decimal Value : %d" % (add, add_int),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .2, creation_delay=1.)
  elif os.path.isfile('/data/log/can_timeout.txt'):
    f = open('/data/log/can_timeout.txt', 'r')
    add = f.readline()
    add_int = int(add, 0)
    f.close()
    return Alert(
      "CAN Error: %s is timeout\n Decimal Value : %d" % (add, add_int),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .2, creation_delay=1.)
  else:
    return Alert(
      "CAN Error: Check Harness Connections",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .2, creation_delay=1.)


EVENTS: Dict[int, Dict[str, Union[Alert, AlertCallbackType]]] = {
  # ********** events with no alerts **********

  EventName.stockFcw: {},

  # ********** events only containing alerts displayed in all states **********

  EventName.joystickDebug: {
    ET.WARNING: joystick_alert,
    ET.PERMANENT: NormalPermanentAlert(tr(21)),
  },

  EventName.controlsInitializing: {
    ET.NO_ENTRY: NoEntryAlert(tr(22)),
  },

  EventName.startup: {
    ET.PERMANENT: StartupAlert(tr(23))
  },

  EventName.startupMaster: {
    ET.PERMANENT: StartupAlert(tr(24),
                               alert_status=AlertStatus.userPrompt),
  },

  # Car is recognized, but marked as dashcam only
  EventName.startupNoControl: {
    ET.PERMANENT: StartupAlert(tr(25)),
  },

  # Car is not recognized
  EventName.startupNoCar: {
    ET.PERMANENT: StartupAlert(tr(26)),
  },

  EventName.startupNoFw: {
    ET.PERMANENT: StartupAlert(tr(27),
                               tr(28),
                               alert_status=AlertStatus.userPrompt),
  },

  EventName.dashcamMode: {
    ET.PERMANENT: NormalPermanentAlert(tr(29),
                                       priority=Priority.LOWEST),
  },

  EventName.invalidLkasSetting: {
    ET.PERMANENT: NormalPermanentAlert(tr(30),
                                       tr(31)),
  },

  EventName.cruiseMismatch: {
    #ET.PERMANENT: ImmediateDisableAlert("openpilot failed to cancel cruise"),
  },

  # openpilot doesn't recognize the car. This switches openpilot into a
  # read-only mode. This can be solved by adding your fingerprint.
  # See https://github.com/commaai/openpilot/wiki/Fingerprinting for more information
  EventName.carUnrecognized: {
    ET.PERMANENT: NormalPermanentAlert(tr(32),
                                       tr(33),
                                       priority=Priority.LOWEST),
  },

  EventName.stockAeb: {
    ET.PERMANENT: Alert(
      tr(34),
      tr(35),
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.fcw, AudibleAlert.none, 2.),
    ET.NO_ENTRY: NoEntryAlert(tr(36)),
  },

  EventName.fcw: {
    ET.PERMANENT: Alert(
      tr(37),
      tr(38),
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.fcw, AudibleAlert.warningSoft, 2.),
  },

  EventName.ldw: {
    ET.PERMANENT: Alert(
      tr(39),
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.ldw, AudibleAlert.prompt, 3.),
  },

  # ********** events only containing alerts that display while engaged **********

  EventName.gasPressed: {
    ET.PRE_ENABLE: Alert(
      tr(40),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .1, creation_delay=1.),
  },

  # openpilot tries to learn certain parameters about your car by observing
  # how the car behaves to steering inputs from both human and openpilot driving.
  # This includes:
  # - steer ratio: gear ratio of the steering rack. Steering angle divided by tire angle
  # - tire stiffness: how much grip your tires have
  # - angle offset: most steering angle sensors are offset and measure a non zero angle when driving straight
  # This alert is thrown when any of these values exceed a sanity check. This can be caused by
  # bad alignment or bad sensor data. If this happens consistently consider creating an issue on GitHub
  EventName.vehicleModelInvalid: {
    ET.NO_ENTRY: NoEntryAlert(tr(41)),
    ET.SOFT_DISABLE: soft_disable_alert(tr(42)),
  },

  EventName.steerTempUnavailableSilent: {
    ET.WARNING: Alert(
      tr(43),
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.prompt, 1.),
  },

  EventName.preDriverDistracted: {
    ET.WARNING: Alert(
      tr(44),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1),
  },

  EventName.promptDriverDistracted: {
    ET.WARNING: Alert(
      tr(45),
      tr(46),
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.promptDistracted, .1),
  },

  EventName.driverDistracted: {
    ET.WARNING: Alert(
      tr(47),
      tr(48),
      AlertStatus.critical, AlertSize.full,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.warningImmediate, .1),
  },

  EventName.preDriverUnresponsive: {
    ET.WARNING: Alert(
      tr(49),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.none, .1, alert_rate=0.75),
  },

  EventName.promptDriverUnresponsive: {
    ET.WARNING: Alert(
      tr(50),
      tr(51),
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.promptDistracted, .1),
  },

  EventName.driverUnresponsive: {
    ET.WARNING: Alert(
      tr(52),
      tr(53),
      AlertStatus.critical, AlertSize.full,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.warningImmediate, .1),
  },

  EventName.manualRestart: {
    ET.WARNING: Alert(
      tr(54),
      tr(55),
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .2),
  },

  EventName.resumeRequired: {
    ET.WARNING: Alert(
      tr(56),
      tr(57),
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .2),
  },

  EventName.belowSteerSpeed: {
    ET.WARNING: below_steer_speed_alert,
  },

  EventName.preLaneChangeLeft: {
    ET.WARNING: Alert(
      tr(58),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1, alert_rate=0.75),
  },

  EventName.preLaneChangeRight: {
    ET.WARNING: Alert(
      tr(59),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1, alert_rate=0.75),
  },

  EventName.laneChangeBlocked: {
    ET.WARNING: Alert(
      tr(60),
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.prompt, .1),
  },

  EventName.laneChange: {
    ET.WARNING: Alert(
      tr(61),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1),
  },

  EventName.e2eLongAlert: {
    ET.WARNING: Alert(
      tr(62),
      tr(63),
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.warning, 2.),
  },

  EventName.laneChangeManual: {
    ET.WARNING: Alert(
      tr(64),
      tr(65),
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1, alert_rate=0.75),
  },

  EventName.emgButtonManual: {
    ET.WARNING: Alert(
      tr(66),
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1, alert_rate=0.75),
  },

  EventName.driverSteering: {
    ET.WARNING: Alert(
      tr(67),
      tr(68),
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1, alert_rate=0.75),
  },

  EventName.steerSaturated: {
    ET.WARNING: Alert(
      tr(69),
      tr(70),
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.prompt, 1.),
  },

  # Thrown when the fan is driven at >50% but is not rotating
  EventName.fanMalfunction: {
    ET.PERMANENT: NormalPermanentAlert(tr(71), tr(72)),
  },

  # Camera is not outputting frames at a constant framerate
  EventName.cameraMalfunction: {
    ET.PERMANENT: NormalPermanentAlert(tr(73), tr(74)),
  },

  # Unused
  EventName.gpsMalfunction: {
    ET.PERMANENT: NormalPermanentAlert(tr(75), tr(76)),
  },

  # When the GPS position and localizer diverge the localizer is reset to the
  # current GPS position. This alert is thrown when the localizer is reset
  # more often than expected.
  EventName.localizerMalfunction: {
    ET.PERMANENT: NormalPermanentAlert(tr(77), tr(78)),
  },

  EventName.modeChangeOpenpilot: {
    ET.WARNING: Alert(
      tr(79),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.warning, 1.),
  },
  
  EventName.modeChangeDistcurv: {
    ET.WARNING: Alert(
      tr(80),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.warning, 1.),
  },
  EventName.modeChangeDistance: {
    ET.WARNING: Alert(
      tr(81),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.warning, 1.),
  },
  EventName.modeChangeCurv: {
    ET.WARNING: Alert(
      tr(82),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.warning, 1.),
  },
  EventName.modeChangeOneway: {
    ET.WARNING: Alert(
      tr(83),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.warning, 1.),
  },
  EventName.modeChangeMaponly: {
    ET.WARNING: Alert(
      tr(84),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.warning, 1.),
  },
  EventName.needBrake: {
    ET.WARNING: Alert(
      tr(85),
      tr(86),
      AlertStatus.normal, AlertSize.full,
      Priority.LOW, VisualAlert.none, AudibleAlert.promptRepeat, .1),
  },
  EventName.routineDriveOn: {
    ET.WARNING: Alert(
      tr(87),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1.),
  },

  # ********** events that affect controls state transitions **********

  EventName.pcmEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.engage),
  },

  EventName.buttonEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.engage),
  },

  EventName.pcmDisable: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
  },

  EventName.buttonCancel: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
  },

  EventName.brakeHold: {
    ET.WARNING: Alert(
      tr(88),
      "",
      AlertStatus.normal, AlertSize.full,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1.),
  },

  EventName.parkBrake: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.none),
    ET.NO_ENTRY: NoEntryAlert(tr(89)),
  },

  EventName.pedalPressed: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.none),
    ET.NO_ENTRY: NoEntryAlert(tr(90),
                              visual_alert=VisualAlert.brakePressed),
  },

  EventName.wrongCarMode: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: wrong_car_mode_alert,
  },

  EventName.wrongCruiseMode: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.none),
    ET.NO_ENTRY: NoEntryAlert(tr(91)),
  },

  EventName.steerTempUnavailable: {
    ET.SOFT_DISABLE: soft_disable_alert(tr(92)),
    ET.NO_ENTRY: NoEntryAlert(tr(93)),
  },
  
  EventName.isgActive: {
    ET.WARNING: Alert(
      tr(94),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1, alert_rate=0.75),
  },

  EventName.camSpeedDown: {
    ET.WARNING: Alert(
      tr(95),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .5, alert_rate=0.75),
  },

  EventName.standstillResButton: {
    ET.WARNING: Alert(
      tr(96),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .5, alert_rate=0.75),
  },

  EventName.gapAdjusting: {
    ET.WARNING: Alert(
      tr(97),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .5, alert_rate=0.75),
  },

  EventName.resCruise: {
    ET.WARNING: Alert(
      tr(98),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .5),
  },

  EventName.curvSpeedDown: {
    ET.WARNING: Alert(
      tr(99),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .5),
  },

  EventName.cutinDetection: {
    ET.WARNING: Alert(
      tr(100),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .5),
  },

  EventName.outOfSpace: {
    ET.PERMANENT: NormalPermanentAlert(tr(101)),
    ET.NO_ENTRY: NoEntryAlert(tr(102)),
  },

  EventName.belowEngageSpeed: {
    ET.NO_ENTRY: below_engage_speed_alert,
  },

  EventName.sensorDataInvalid: {
    ET.PERMANENT: Alert(
      tr(103),
      tr(104),
      AlertStatus.normal, AlertSize.mid,
      Priority.LOWER, VisualAlert.none, AudibleAlert.none, .2, creation_delay=1.),
    ET.NO_ENTRY: NoEntryAlert(tr(105)),
  },

  EventName.noGps: {
    ET.PERMANENT: no_gps_alert,
  },

  EventName.soundsUnavailable: {
    ET.PERMANENT: NormalPermanentAlert(tr(106), tr(107)),
    ET.NO_ENTRY: NoEntryAlert(tr(108)),
  },

  EventName.tooDistracted: {
    ET.NO_ENTRY: NoEntryAlert(tr(109)),
  },

  EventName.overheat: {
    ET.PERMANENT: NormalPermanentAlert(tr(110)),
    ET.SOFT_DISABLE: soft_disable_alert(tr(111)),
    ET.NO_ENTRY: NoEntryAlert(tr(112)),
  },

  EventName.wrongGear: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert(tr(113)),
  },

  # This alert is thrown when the calibration angles are outside of the acceptable range.
  # For example if the device is pointed too much to the left or the right.
  # Usually this can only be solved by removing the mount from the windshield completely,
  # and attaching while making sure the device is pointed straight forward and is level.
  # See https://comma.ai/setup for more information
  EventName.calibrationInvalid: {
    ET.PERMANENT: NormalPermanentAlert(tr(114), tr(115)),
    ET.SOFT_DISABLE: soft_disable_alert(tr(116)),
    ET.NO_ENTRY: NoEntryAlert(tr(117)),
  },

  EventName.calibrationIncomplete: {
    ET.PERMANENT: calibration_incomplete_alert,
    ET.SOFT_DISABLE: soft_disable_alert(tr(118)),
    ET.NO_ENTRY: NoEntryAlert(tr(119)),
  },

  EventName.doorOpen: {
    ET.SOFT_DISABLE: user_soft_disable_alert(tr(120)),
    ET.NO_ENTRY: NoEntryAlert(tr(121)),
  },

  EventName.seatbeltNotLatched: {
    ET.SOFT_DISABLE: user_soft_disable_alert(tr(122)),
    ET.NO_ENTRY: NoEntryAlert(tr(123)),
  },

  EventName.espDisabled: {
    ET.SOFT_DISABLE: soft_disable_alert(tr(124)),
    ET.NO_ENTRY: NoEntryAlert(tr(125)),
  },

  EventName.lowBattery: {
    ET.SOFT_DISABLE: soft_disable_alert(tr(126)),
    ET.NO_ENTRY: NoEntryAlert(tr(127)),
  },

  # Different openpilot services communicate between each other at a certain
  # interval. If communication does not follow the regular schedule this alert
  # is thrown. This can mean a service crashed, did not broadcast a message for
  # ten times the regular interval, or the average interval is more than 10% too high.
  EventName.commIssue: {
    ET.WARNING: Alert(
      tr(128),
      tr(129),
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1.),
  },
  
  EventName.commIssueAvgFreq: {
    ET.SOFT_DISABLE: soft_disable_alert(tr(130)),
    ET.NO_ENTRY: NoEntryAlert(tr(131)),
  },  

  # Thrown when manager detects a service exited unexpectedly while driving
  EventName.processNotRunning: {
    ET.NO_ENTRY: NoEntryAlert(tr(132)),
  },

  EventName.radarFault: {
    ET.SOFT_DISABLE: soft_disable_alert(tr(133)),
    ET.NO_ENTRY: NoEntryAlert(tr(134)),
  },

  # Every frame from the camera should be processed by the model. If modeld
  # is not processing frames fast enough they have to be dropped. This alert is
  # thrown when over 20% of frames are dropped.
  EventName.modeldLagging: {
    ET.SOFT_DISABLE: soft_disable_alert(tr(135)),
    ET.NO_ENTRY: NoEntryAlert(tr(136)),
  },

  # Besides predicting the path, lane lines and lead car data the model also
  # predicts the current velocity and rotation speed of the car. If the model is
  # very uncertain about the current velocity while the car is moving, this
  # usually means the model has trouble understanding the scene. This is used
  # as a heuristic to warn the driver.
  EventName.posenetInvalid: {
    ET.SOFT_DISABLE: soft_disable_alert(tr(137)),
    ET.NO_ENTRY: NoEntryAlert(tr(138)),
  },

  # When the localizer detects an acceleration of more than 40 m/s^2 (~4G) we
  # alert the driver the device might have fallen from the windshield.
  EventName.deviceFalling: {
    ET.SOFT_DISABLE: soft_disable_alert(tr(139)),
    ET.NO_ENTRY: NoEntryAlert(tr(140)),
  },

  EventName.lowMemory: {
    ET.SOFT_DISABLE: soft_disable_alert(tr(141)),
    ET.PERMANENT: NormalPermanentAlert(tr(142), tr(143)),
    ET.NO_ENTRY: NoEntryAlert(tr(144)),
  },

  EventName.highCpuUsage: {
    #ET.SOFT_DISABLE: soft_disable_alert("System Malfunction: Reboot Your Device"),
    #ET.PERMANENT: NormalPermanentAlert("System Malfunction", "Reboot your Device"),
    ET.NO_ENTRY: NoEntryAlert(tr(145)),
  },

  EventName.accFaulted: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert(tr(146)),
    ET.PERMANENT: NormalPermanentAlert(tr(147), ""),
    ET.NO_ENTRY: NoEntryAlert(tr(148)),
  },

  EventName.controlsMismatch: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert(tr(149)),
  },

  EventName.roadCameraError: {
    ET.PERMANENT: NormalPermanentAlert(tr(150),
                                       duration=1.,
                                       creation_delay=30.),
  },

  EventName.driverCameraError: {
    ET.PERMANENT: NormalPermanentAlert(tr(151),
                                       duration=1.,
                                       creation_delay=30.),
  },

  EventName.wideRoadCameraError: {
    ET.PERMANENT: NormalPermanentAlert(tr(152),
                                       duration=1.,
                                       creation_delay=30.),
  },

  # Sometimes the USB stack on the device can get into a bad state
  # causing the connection to the panda to be lost
  EventName.usbError: {
    ET.SOFT_DISABLE: soft_disable_alert(tr(153)),
    ET.PERMANENT: NormalPermanentAlert(tr(154), ""),
    ET.NO_ENTRY: NoEntryAlert(tr(155)),
  },

  # This alert can be thrown for the following reasons:
  # - No CAN data received at all
  # - CAN data is received, but some message are not received at the right frequency
  # If you're not writing a new car port, this is usually cause by faulty wiring
  EventName.canError: {
    ET.PERMANENT: can_error_alert,
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert(tr(156)),
    # ET.PERMANENT: Alert(
    #   "CAN Error: Check Connections",
    #   "",
    #   AlertStatus.normal, AlertSize.small,
    #   Priority.LOW, VisualAlert.none, AudibleAlert.none, 1., creation_delay=1.),
    ET.NO_ENTRY: NoEntryAlert(tr(157)),
  },

  EventName.steerUnavailable: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert(tr(158)),
    ET.PERMANENT: NormalPermanentAlert(tr(159)),
    ET.NO_ENTRY: NoEntryAlert(tr(160)),
  },

  EventName.brakeUnavailable: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert(tr(161)),
    ET.PERMANENT: NormalPermanentAlert(tr(162)),
    ET.NO_ENTRY: NoEntryAlert(tr(163)),
  },

  EventName.reverseGear: {
    ET.PERMANENT: Alert(
      tr(164),
      "",
      AlertStatus.userPrompt, AlertSize.full,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .2, creation_delay=0.5),
    # ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Reverse Gear"),
    ET.NO_ENTRY: NoEntryAlert(tr(165)),
  },

  EventName.gearNotD: {
    ET.WARNING: Alert(
      tr(166),
      "",
      AlertStatus.userPrompt, AlertSize.full,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .2, creation_delay=0.5),
    # ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Reverse Gear"),
    ET.NO_ENTRY: NoEntryAlert(tr(167)),
  },

  # On cars that use stock ACC the car can decide to cancel ACC for various reasons.
  # When this happens we can no long control the car so the user needs to be warned immediately.
  EventName.cruiseDisabled: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert(tr(168)),
  },

  # For planning the trajectory Model Predictive Control (MPC) is used. This is
  # an optimization algorithm that is not guaranteed to find a feasible solution.
  # If no solution is found or the solution has a very high cost this alert is thrown.
  EventName.plannerError: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert(tr(169)),
    ET.NO_ENTRY: NoEntryAlert(tr(170)),
  },

  # When the relay in the harness box opens the CAN bus between the LKAS camera
  # and the rest of the car is separated. When messages from the LKAS camera
  # are received on the car side this usually means the relay hasn't opened correctly
  # and this alert is thrown.
  EventName.relayMalfunction: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert(tr(171)),
    ET.PERMANENT: NormalPermanentAlert(tr(172), tr(173)),
    ET.NO_ENTRY: NoEntryAlert(tr(174)),
  },

  EventName.noTarget: {
    ET.IMMEDIATE_DISABLE: Alert(
      tr(175),
      tr(176),
      AlertStatus.normal, AlertSize.mid,
      Priority.HIGH, VisualAlert.none, AudibleAlert.none, 3.),
    ET.NO_ENTRY: NoEntryAlert(tr(177)),
  },

  EventName.speedTooLow: {
    ET.IMMEDIATE_DISABLE: Alert(
      tr(178),
      tr(179),
      AlertStatus.normal, AlertSize.mid,
      Priority.HIGH, VisualAlert.none, AudibleAlert.none, 3.),
  },

  # When the car is driving faster than most cars in the training data, the model outputs can be unpredictable.
  EventName.speedTooHigh: {
    ET.WARNING: Alert(
      tr(180),
      tr(181),
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.promptRepeat, 4.),
    ET.NO_ENTRY: NoEntryAlert(tr(182)),
  },

  EventName.lowSpeedLockout: {
    ET.PERMANENT: NormalPermanentAlert(tr(183)),
    ET.NO_ENTRY: NoEntryAlert(tr(184)),
  },

  EventName.lkasDisabled: {
    # ET.PERMANENT: NormalPermanentAlert("LKAS Disabled: Enable LKAS to engage"),
    # ET.NO_ENTRY: NoEntryAlert("LKAS Disabled"),
    ET.WARNING: Alert(
      tr(185),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.disengage, 1., alert_rate=0.5),
  },

  EventName.lkasEnabled: {
    # ET.PERMANENT: NormalPermanentAlert("LKAS Disabled: Enable LKAS to engage"),
    # ET.NO_ENTRY: NoEntryAlert("LKAS Disabled"),
    ET.WARNING: Alert(
      tr(186),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.engage, 1.),
  },
 
  EventName.unSleepMode: {
    ET.WARNING: Alert(
      tr(187),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 2.),
  },

  EventName.speedBump: {
    ET.WARNING: Alert(
      tr(188),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .5),
  },

  EventName.sccDriverOverride: {
    ET.WARNING: Alert(
      tr(189),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .5),
  },

  EventName.doNotDisturb: {
    ET.WARNING: Alert(
      tr(190),
      tr(191),
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 5.),
  },

  EventName.chimeAtResume: {
    ET.WARNING: Alert(
      tr(192),
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.dingdong, 3.),
  },

}
