#!/usr/bin/env python3
from cereal import car
from panda import Panda
from common.conversions import Conversions as CV
from selfdrive.car.hyundai.tunes import LatTunes, LongTunes, set_long_tune, set_lat_tune
from selfdrive.car.hyundai.values import CAR, EV_CAR, HYBRID_CAR, Buttons, CarControllerParams
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.disable_ecu import disable_ecu

from common.params import Params
from decimal import Decimal

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    self.cp2 = self.CS.get_can2_parser(CP)
    self.lkas_button_alert = False

    self.blinker_status = 0
    self.blinker_timer = 0
    self.ufc_mode_enabled = Params().get_bool('UFCModeEnabled')
    self.no_mdps_mods = Params().get_bool('NoSmartMDPS')

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[], disable_radar=False):  # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.carName = "hyundai"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiLegacy, 0)]

    ret.mdpsBus = 1 if 593 in fingerprint[1] and 1296 not in fingerprint[1] else 0
    ret.sasBus = 1 if 688 in fingerprint[1] and 1296 not in fingerprint[1] else 0
    ret.sccBus = 0 if 1056 in fingerprint[0] else 1 if 1056 in fingerprint[1] and 1296 not in fingerprint[1] \
                                                                     else 2 if 1056 in fingerprint[2] else -1
    ret.fcaBus = 0 if 909 in fingerprint[0] else 2 if 909 in fingerprint[2] else -1
    ret.bsmAvailable = True if 1419 in fingerprint[0] else False
    ret.lfaAvailable = True if 1157 in fingerprint[2] else False
    ret.lvrAvailable = True if 871 in fingerprint[0] else False
    ret.evgearAvailable = True if 882 in fingerprint[0] else False
    ret.emsAvailable = True if 608 and 809 in fingerprint[0] else False

    ret.radarOffCan = ret.sccBus == -1
    ret.standStill = False
    ret.openpilotLongitudinalControl = Params().get_bool("RadarDisable") or ret.sccBus == 2



    ret.smoothSteer.method = int( Params().get("OpkrSteerMethod", encoding="utf8") )   # 1
    ret.smoothSteer.maxSteeringAngle = float( Params().get("OpkrMaxSteeringAngle", encoding="utf8") )   # 90
    ret.smoothSteer.maxDriverAngleWait = float( Params().get("OpkrMaxDriverAngleWait", encoding="utf8") )  # 0.002
    ret.smoothSteer.maxSteerAngleWait = float( Params().get("OpkrMaxSteerAngleWait", encoding="utf8") )   # 0.001  # 10 sec
    ret.smoothSteer.driverAngleWait = float( Params().get("OpkrDriverAngleWait", encoding="utf8") )  #0.001
    #ret.steeringPressed
    #ret.maxSteeringAngleDeg = 90
    ret.minSteerSpeed = 16.67 # m/s

    # Most Hyundai car ports are community features for now
    ret.pcmCruise = not ret.radarOffCan

    ret.steerActuatorDelay = 0.25  # Default delay
    ret.steerLimitTimer = 0.8
    tire_stiffness_factor = 1.

    set_long_tune(ret.longitudinalTuning, LongTunes.OPKR)

    ret.stoppingControl = False
    ret.vEgoStopping = 0.8  # 1.0, 0.5
    ret.vEgoStarting = 0.8  # needs to be >= vEgoStopping to avoid state transition oscillation
    ret.stopAccel = -2.0 # 0.0, -0.5    
    ret.stoppingDecelRate = 1.0 # 0.8, 0.2  # brake_travel/s while trying to stop
    
    ret.longitudinalActuatorDelayLowerBound = 1.0
    ret.longitudinalActuatorDelayUpperBound = 1.0

    ret.vCruisekph = 0
    ret.resSpeed = 0
    ret.vFuture = 0
    ret.vFutureA = 0
    ret.aqValue = 0
    ret.aqValueRaw = 0

    params = Params()
    tire_stiffness_factor = float(Decimal(params.get("TireStiffnessFactorAdj", encoding="utf8")) * Decimal('0.01'))
    ret.steerActuatorDelay = float(Decimal(params.get("SteerActuatorDelayAdj", encoding="utf8")) * Decimal('0.01'))
    ret.steerLimitTimer = float(Decimal(params.get("SteerLimitTimerAdj", encoding="utf8")) * Decimal('0.01'))
    ret.steerRatio = float(Decimal(params.get("SteerRatioAdj", encoding="utf8")) * Decimal('0.01'))

    lat_control_method = int(params.get("LateralControlMethod", encoding="utf8"))
    if lat_control_method == 0:
      set_lat_tune(ret.lateralTuning, LatTunes.PID)
    elif lat_control_method == 1:
      set_lat_tune(ret.lateralTuning, LatTunes.INDI)
    elif lat_control_method == 2:
      set_lat_tune(ret.lateralTuning, LatTunes.LQR)
    elif lat_control_method == 3:
      set_lat_tune(ret.lateralTuning, LatTunes.TORQUE)
    elif lat_control_method == 4:
      set_lat_tune(ret.lateralTuning, LatTunes.ATOM)    # Hybrid tune  

    # genesis
    if candidate == CAR.GENESIS_DH:
      ret.mass = 1930. + STD_CARGO_KG
      ret.wheelbase = 3.01
    elif candidate == CAR.GENESIS_G70_IK:
      ret.mass = 1595. + STD_CARGO_KG
      ret.wheelbase = 2.835
    elif candidate == CAR.GENESIS_G80_DH:
      ret.mass = 1855. + STD_CARGO_KG
      ret.wheelbase = 3.01
    elif candidate == CAR.GENESIS_G90_HI:
      ret.mass = 2120. + STD_CARGO_KG
      ret.wheelbase = 3.16
    elif candidate == CAR.GENESIS_EQ900_HI:
      ret.mass = 2130. + STD_CARGO_KG
      ret.wheelbase = 3.16
    # hyundai
    elif candidate == CAR.SANTAFE_TM:
      ret.mass = 1694. + STD_CARGO_KG
      ret.wheelbase = 2.765
    elif candidate == CAR.SANTAFE_HEV_TM:
      ret.mass = 1907. + STD_CARGO_KG
      ret.wheelbase = 2.765
    elif candidate == CAR.SONATA_DN8:
      ret.mass = 1465. + STD_CARGO_KG
      ret.wheelbase = 2.84
    elif candidate == CAR.SONATA_HEV_DN8:
      ret.mass = 1505. + STD_CARGO_KG
      ret.wheelbase = 2.84
    elif candidate == CAR.SONATA_LF:
      ret.mass = 1465. + STD_CARGO_KG
      ret.wheelbase = 2.805
    elif candidate == CAR.SONATA_TURBO_LF:
      ret.mass = 1470. + STD_CARGO_KG
      ret.wheelbase = 2.805
    elif candidate == CAR.SONATA_HEV_LF:
      ret.mass = 1595. + STD_CARGO_KG
      ret.wheelbase = 2.805
    elif candidate == CAR.PALISADE_LX2:
      ret.mass = 1885. + STD_CARGO_KG
      ret.wheelbase = 2.90
    elif candidate == CAR.AVANTE_AD:
      ret.mass = 1250. + STD_CARGO_KG
      ret.wheelbase = 2.7
    elif candidate == CAR.AVANTE_CN7:
      ret.mass = 1225. + STD_CARGO_KG
      ret.wheelbase = 2.72
    elif candidate == CAR.AVANTE_HEV_CN7:
      ret.mass = 1335. + STD_CARGO_KG
      ret.wheelbase = 2.72
    elif candidate == CAR.I30_PD:
      ret.mass = 1380. + STD_CARGO_KG
      ret.wheelbase = 2.65
    elif candidate == CAR.KONA_OS:
      ret.mass = 1325. + STD_CARGO_KG
      ret.wheelbase = 2.6
    elif candidate == CAR.KONA_HEV_OS:
      ret.mass = 1395. + STD_CARGO_KG
      ret.wheelbase = 2.6
    elif candidate == CAR.KONA_EV_OS:
      ret.mass = 1685. + STD_CARGO_KG
      ret.wheelbase = 2.6
    elif candidate == CAR.IONIQ_HEV_AE:
      ret.mass = 1380. + STD_CARGO_KG
      ret.wheelbase = 2.7
    elif candidate == CAR.IONIQ_EV_AE:
      ret.mass = 1445. + STD_CARGO_KG
      ret.wheelbase = 2.7
    elif candidate == CAR.GRANDEUR_IG:
      ret.mass = 1560. + STD_CARGO_KG
      ret.wheelbase = 2.845
    elif candidate == CAR.GRANDEUR_HEV_IG:
      ret.mass = 1675. + STD_CARGO_KG
      ret.wheelbase = 2.845
    elif candidate == CAR.GRANDEUR_FL_IG:
      ret.mass = 1625. + STD_CARGO_KG
      ret.wheelbase = 2.885
    elif candidate == CAR.GRANDEUR_HEV_FL_IG:
      ret.mass = 1675. + STD_CARGO_KG
      ret.wheelbase = 2.885
    elif candidate == CAR.VELOSTER_JS:
      ret.mass = 1285. + STD_CARGO_KG
      ret.wheelbase = 2.65
    elif candidate == CAR.TUCSON_TL:
      ret.mass = 1550. + STD_CARGO_KG
      ret.wheelbase = 2.67
    elif candidate == CAR.NEXO_FE:
      ret.mass = 1885. + STD_CARGO_KG
      ret.wheelbase = 2.79
    # kia
    elif candidate == CAR.KIA_FORTE:
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
    elif candidate == CAR.SORENTO_UM:
      ret.mass = 1910. + STD_CARGO_KG
      ret.wheelbase = 2.78
    elif candidate == CAR.K5_JF:
      ret.wheelbase = 2.805
      ret.mass = 1475. + STD_CARGO_KG
    elif candidate == CAR.K5_HEV_JF:
      ret.wheelbase = 2.805
      ret.mass = 1600. + STD_CARGO_KG
    elif candidate == CAR.K5_DL3:
      ret.wheelbase = 2.85
      ret.mass = 1450. + STD_CARGO_KG
    elif candidate == CAR.STINGER_CK:
      ret.mass = 1650. + STD_CARGO_KG
      ret.wheelbase = 2.905
    elif candidate == CAR.K3_BD:
      ret.mass = 1260. + STD_CARGO_KG
      ret.wheelbase = 2.70
    elif candidate == CAR.SPORTAGE_QL:
      ret.mass = 1510. + STD_CARGO_KG
      ret.wheelbase = 2.67
    elif candidate == CAR.NIRO_HEV_DE:
      ret.mass = 1425. + STD_CARGO_KG
      ret.wheelbase = 2.7
    elif candidate == CAR.NIRO_EV_DE:
      ret.mass = 1755. + STD_CARGO_KG
      ret.wheelbase = 2.7
    elif candidate == CAR.K7_YG:
      ret.mass = 1565. + STD_CARGO_KG
      ret.wheelbase = 2.855
    elif candidate == CAR.K7_HEV_YG:
      ret.mass = 1680. + STD_CARGO_KG
      ret.wheelbase = 2.855
    elif candidate == CAR.SELTOS_SP2:
      ret.mass = 1425. + STD_CARGO_KG
      ret.wheelbase = 2.63
    elif candidate == CAR.SOUL_EV_SK3:
      ret.mass = 1695. + STD_CARGO_KG
      ret.wheelbase = 2.6
    elif candidate == CAR.MOHAVE_HM:
      ret.mass = 2285. + STD_CARGO_KG
      ret.wheelbase = 2.895

    # set appropriate safety param for gas signal
    if candidate in HYBRID_CAR:
      ret.safetyConfigs[0].safetyParam = 2
    elif candidate in EV_CAR:
      ret.safetyConfigs[0].safetyParam = 1

    ret.centerToFront = ret.wheelbase * 0.4

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.enableBsm = 0x58b in fingerprint[0]

    #if ret.openpilotLongitudinalControl:
    #  ret.safetyParam |= Panda.FLAG_HYUNDAI_LONG

    # set safety_hyundai_community only for non-SCC, MDPS harrness or SCC harrness cars or cars that have unknown issue
    if ret.radarOffCan or ret.mdpsBus == 1 or ret.openpilotLongitudinalControl or params.get_bool("UFCModeEnabled"):
      ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiCommunity, 0)]
    return ret

  # @staticmethod
  # def init(CP, logcan, sendcan):
  #   if CP.openpilotLongitudinalControl and CP.sccBus in (0, -1):
  #     disable_ecu(logcan, sendcan, addr=0x7d0, com_cont_req=b'\x28\x83\x01')

  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)
    self.cp2.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp2, self.cp_cam)
    ret.canValid = self.cp.can_valid and self.cp2.can_valid and self.cp_cam.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    if not self.cp.can_valid or not self.cp2.can_valid or not self.cp_cam.can_valid:
      print('cp={}  cp2={}  cp_cam={}'.format(bool(self.cp.can_valid), bool(self.cp2.can_valid), bool(self.cp_cam.can_valid)))


    if self.CP.pcmCruise and not self.CC.scc_live:
      self.CP.pcmCruise = False
    elif self.CC.scc_live and not self.CP.pcmCruise:
      self.CP.pcmCruise = True

    # most HKG cars has no long control, it is safer and easier to engage by main on
    if self.ufc_mode_enabled:
      ret.cruiseState.enabled = ret.cruiseState.available

    buttonEvents = []
    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
      be = car.CarState.ButtonEvent.new_message()
      be.pressed = self.CS.cruise_buttons != 0
      but = self.CS.cruise_buttons if be.pressed else self.CS.prev_cruise_buttons
      if but == Buttons.RES_ACCEL:
        be.type = ButtonType.accelCruise
      elif but == Buttons.SET_DECEL:
        be.type = ButtonType.decelCruise
      elif but == Buttons.GAP_DIST:
        be.type = ButtonType.gapAdjustCruise
      #elif but == Buttons.CANCEL:
      #  be.type = ButtonType.cancel
      else:
        be.type = ButtonType.unknown
      buttonEvents.append(be)
    if self.CS.cruise_main_button != self.CS.prev_cruise_main_button:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.altButton3
      be.pressed = bool(self.CS.cruise_main_button)
      buttonEvents.append(be)
    ret.buttonEvents = buttonEvents

    events = self.create_common_events(ret)

    if self.CC.longcontrol and self.CS.brake_error:
      events.add(EventName.brakeUnavailable)
    #if abs(ret.steeringAngle) > 90. and EventName.steerTempUnavailable not in events.events:
    #  events.add(EventName.steerTempUnavailable)
    # if self.ufc_mode_enabled and EventName.pedalPressed in events.events:
    #   events.events.remove(EventName.pedalPressed)
    if ret.vEgo < self.CP.minSteerSpeed and self.no_mdps_mods:
      events.add(car.CarEvent.EventName.belowSteerSpeed)
    if self.CC.need_brake and not self.CC.longcontrol:
      events.add(EventName.needBrake)
    if not self.CC.lkas_temp_disabled:
      if self.CC.lanechange_manual_timer and ret.vEgo > 0.3:
        events.add(EventName.laneChangeManual)
      if self.CC.emergency_manual_timer:
        events.add(EventName.emgButtonManual)
      #if self.CC.driver_steering_torque_above_timer:
      #  events.add(EventName.driverSteering)
      if self.CC.standstill_res_button:
        events.add(EventName.standstillResButton)
      if self.CC.cruise_gap_adjusting:
        events.add(EventName.gapAdjusting)
      if self.CC.on_speed_bump_control and ret.vEgo > 8.3:
        events.add(EventName.speedBump)
      if self.CC.on_speed_control and ret.vEgo > 0.3:
        events.add(EventName.camSpeedDown)
      if self.CC.curv_speed_control and ret.vEgo > 8.3:
        events.add(EventName.curvSpeedDown)
      if self.CC.cut_in_control and ret.vEgo > 8.3:
        events.add(EventName.cutinDetection)
      if self.CC.driver_scc_set_control:
        events.add(EventName.sccDriverOverride)        
      if self.CC.autohold_popup_timer:
        events.add(EventName.brakeHold)
      if self.CC.auto_res_starting:
        events.add(EventName.resCruise)
      if self.CC.e2e_standstill:
        events.add(EventName.chimeAtResume)
    if self.CS.cruiseState_standstill or self.CC.standstill_status == 1:
      #events.add(EventName.standStill)
      self.CP.standStill = True
    else:
      self.CP.standStill = False
    if self.CC.v_cruise_kph_auto_res > (20 if self.CS.is_set_speed_in_mph else 30):
      self.CP.vCruisekph = self.CC.v_cruise_kph_auto_res
    else:
      self.CP.vCruisekph = 0
    if self.CC.res_speed != 0:
      self.CP.resSpeed = self.CC.res_speed
    else:
      self.CP.resSpeed = 0
    if self.CC.vFuture >= 1:
      self.CP.vFuture = self.CC.vFuture
    else:
      self.CP.vFuture = 0
    if self.CC.vFutureA >= 1:
      self.CP.vFutureA = self.CC.vFutureA
    else:
      self.CP.vFutureA = 0
    self.CP.aqValue = self.CC.aq_value
    self.CP.aqValueRaw = self.CC.aq_value_raw

    if self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 0:
      events.add(EventName.modeChangeOpenpilot)
    elif self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 1:
      events.add(EventName.modeChangeDistcurv)
    elif self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 2:
      events.add(EventName.modeChangeDistance)
    elif self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 3:
      events.add(EventName.modeChangeCurv)
    elif self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 4:
      events.add(EventName.modeChangeOneway)
    elif self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 5:
      events.add(EventName.modeChangeMaponly)

    if self.CC.lkas_temp_disabled:
      events.add(EventName.lkasDisabled)
    elif self.CC.lkas_temp_disabled_timer:
      events.add(EventName.lkasEnabled)

  # handle button presses
    for b in ret.buttonEvents:
      # do disable on button down
      if b.type == ButtonType.cancel and b.pressed:
        events.add(EventName.buttonCancel)
      if self.CC.longcontrol and not self.CC.scc_live:
        # do enable on both accel and decel buttons
        if b.type in (ButtonType.accelCruise, ButtonType.decelCruise) and not b.pressed:
          events.add(EventName.buttonEnable)
        if EventName.wrongCarMode in events.events:
          events.events.remove(EventName.wrongCarMode)
        if EventName.pcmDisable in events.events:
          events.events.remove(EventName.pcmDisable)
      elif not self.CC.longcontrol and ret.cruiseState.enabled:
        # do enable on decel button only
        if b.type == ButtonType.decelCruise and not b.pressed:
          events.add(EventName.buttonEnable)

    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c):
    hud_control = c.hudControl
    ret = self.CC.update(c, c.enabled, self.CS, self.frame, c.actuators,
                         c.cruiseControl.cancel, hud_control.visualAlert, hud_control.leftLaneVisible,
                         hud_control.rightLaneVisible, hud_control.leftLaneDepart, hud_control.rightLaneDepart,
                         hud_control.setSpeed, hud_control.leadVisible, hud_control.vFuture, hud_control.vFutureA)
    self.frame += 1
    return ret
