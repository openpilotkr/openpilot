from cereal import car, log, messaging
from common.realtime import DT_CTRL
from common.numpy_fast import clip, interp
from common.conversions import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, create_lfahda_mfc, create_hda_mfc, \
                                             create_scc11, create_scc12, create_scc13, create_scc14, \
                                             create_scc42a, create_scc7d0, create_mdps12, create_fca11, create_fca12
from selfdrive.car.hyundai.values import Buttons, CarControllerParams, CAR, FEATURES
from opendbc.can.packer import CANPacker
from selfdrive.controls.lib.longcontrol import LongCtrlState
from selfdrive.car.hyundai.carstate import GearShifter
from selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN

from selfdrive.car.hyundai.navicontrol  import NaviControl

from common.params import Params
import common.log as trace1
import common.CTime1000 as tm
from random import randint
from decimal import Decimal

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState
LongitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource
LaneChangeState = log.LateralPlan.LaneChangeState


def process_hud_alert(enabled, fingerprint, visual_alert, left_lane,
                      right_lane, left_lane_depart, right_lane_depart):
  sys_warning = (visual_alert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  sys_state = 1
  if left_lane and right_lane or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif left_lane:
    sys_state = 5
  elif right_lane:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if left_lane_depart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_DH, CAR.GENESIS_G90_HI, CAR.GENESIS_G80_DH, CAR.GENESIS_G70_IK) else 2
  if right_lane_depart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_DH, CAR.GENESIS_G90_HI, CAR.GENESIS_G80_DH, CAR.GENESIS_G70_IK) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.p = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)
    self.angle_limit_counter = 0
    self.cut_steer_frames = 0
    self.cut_steer = False

    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.steer_rate_limited = False
    self.lkas11_cnt = 0
    self.scc12_cnt = 0
    self.counter_init = False
    self.aq_value = 0
    self.aq_value_raw = 0

    self.resume_cnt = 0
    self.last_lead_distance = 0
    self.resume_wait_timer = 0

    self.last_resume_frame = 0
    self.accel = 0

    self.lanechange_manual_timer = 0
    self.emergency_manual_timer = 0
    self.driver_steering_torque_above = False
    self.driver_steering_torque_above_timer = 100
    
    self.mode_change_timer = 0

    self.acc_standstill_timer = 0
    self.acc_standstill = False

    self.need_brake = False
    self.need_brake_timer = 0

    self.cancel_counter = 0

    self.v_cruise_kph_auto_res = 0

    self.params = Params()
    self.mode_change_switch = int(self.params.get("CruiseStatemodeSelInit", encoding="utf8"))
    self.opkr_variablecruise = self.params.get_bool("OpkrVariableCruise")
    self.opkr_autoresume = self.params.get_bool("OpkrAutoResume")
    self.opkr_cruisegap_auto_adj = self.params.get_bool("CruiseGapAdjust")
    self.opkr_cruise_auto_res = self.params.get_bool("CruiseAutoRes")
    self.opkr_cruise_auto_res_option = int(self.params.get("AutoResOption", encoding="utf8"))
    self.opkr_cruise_auto_res_condition = int(self.params.get("AutoResCondition", encoding="utf8"))

    self.opkr_turnsteeringdisable = self.params.get_bool("OpkrTurnSteeringDisable")
    self.opkr_maxanglelimit = float(int(self.params.get("OpkrMaxAngleLimit", encoding="utf8")))
    self.ufc_mode_enabled = self.params.get_bool("UFCModeEnabled")
    self.ldws_fix = self.params.get_bool("LdwsCarFix")
    self.radar_helper_option = int(self.params.get("RadarLongHelper", encoding="utf8"))
    self.stopping_dist_adj_enabled = self.params.get_bool("StoppingDistAdj")
    self.standstill_resume_alt = self.params.get_bool("StandstillResumeAlt")
    self.auto_res_delay = int(self.params.get("AutoRESDelay", encoding="utf8")) * 100
    self.auto_res_delay_timer = 0
    self.stopped = False
    self.stoppingdist = float(Decimal(self.params.get("StoppingDist", encoding="utf8"))*Decimal('0.1'))

    self.longcontrol = CP.openpilotLongitudinalControl
    #self.scc_live is true because CP.radarOffCan is False
    self.scc_live = not CP.radarOffCan

    self.timer1 = tm.CTime1000("time")

    self.NC = NaviControl()

    self.dRel = 0
    self.vRel = 0
    self.yRel = 0

    self.cruise_gap_prev = 0
    self.cruise_gap_set_init = False
    self.cruise_gap_adjusting = False
    self.standstill_fault_reduce_timer = 0
    self.standstill_res_button = False
    self.standstill_res_count = int(self.params.get("RESCountatStandstill", encoding="utf8"))

    self.standstill_status = 0
    self.standstill_status_timer = 0
    self.switch_timer = 0
    self.switch_timer2 = 0
    self.auto_res_timer = 0
    self.auto_res_limit_timer = 0
    self.auto_res_limit_sec = int(self.params.get("AutoResLimitTime", encoding="utf8")) * 100
    self.auto_res_starting = False
    self.res_speed = 0
    self.res_speed_timer = 0
    self.autohold_popup_timer = 0
    self.autohold_popup_switch = False

    self.steerMax_base = int(self.params.get("SteerMaxBaseAdj", encoding="utf8"))
    self.steerDeltaUp_base = int(self.params.get("SteerDeltaUpBaseAdj", encoding="utf8"))
    self.steerDeltaDown_base = int(self.params.get("SteerDeltaDownBaseAdj", encoding="utf8"))
    self.steerMax_Max = int(self.params.get("SteerMaxAdj", encoding="utf8"))
    self.steerDeltaUp_Max = int(self.params.get("SteerDeltaUpAdj", encoding="utf8"))
    self.steerDeltaDown_Max = int(self.params.get("SteerDeltaDownAdj", encoding="utf8"))
    self.model_speed_range = [30, 100, 255]
    self.steerMax_range = [self.steerMax_Max, self.steerMax_base, self.steerMax_base]
    self.steerDeltaUp_range = [self.steerDeltaUp_Max, self.steerDeltaUp_base, self.steerDeltaUp_base]
    self.steerDeltaDown_range = [self.steerDeltaDown_Max, self.steerDeltaDown_base, self.steerDeltaDown_base]
    self.steerMax = 0
    self.steerDeltaUp = 0
    self.steerDeltaDown = 0

    self.variable_steer_max = self.params.get_bool("OpkrVariableSteerMax")
    self.variable_steer_delta = self.params.get_bool("OpkrVariableSteerDelta")
    self.osm_spdlimit_enabled = self.params.get_bool("OSMSpeedLimitEnable")
    self.stock_safety_decel_enabled = self.params.get_bool("UseStockDecelOnSS")
    self.joystick_debug_mode = self.params.get_bool("JoystickDebugMode")
    self.stopsign_enabled = self.params.get_bool("StopAtStopSign")

    self.smooth_start = False

    self.cc_timer = 0
    self.on_speed_control = False
    self.on_speed_bump_control = False
    self.curv_speed_control = False
    self.cut_in_control = False
    self.driver_scc_set_control = False
    self.vFuture = 0
    self.vFutureA = 0
    self.cruise_init = False
    self.change_accel_fast = False

    self.to_avoid_lkas_fault_enabled = self.params.get_bool("AvoidLKASFaultEnabled")
    self.to_avoid_lkas_fault_max_angle = int(self.params.get("AvoidLKASFaultMaxAngle", encoding="utf8"))
    self.to_avoid_lkas_fault_max_frame = int(self.params.get("AvoidLKASFaultMaxFrame", encoding="utf8"))
    self.enable_steer_more = self.params.get_bool("AvoidLKASFaultBeyond")
    self.no_mdps_mods = self.params.get_bool("NoSmartMDPS")

    #self.user_specific_feature = int(self.params.get("UserSpecificFeature", encoding="utf8"))

    self.gap_by_spd_on = self.params.get_bool("CruiseGapBySpdOn")
    self.gap_by_spd_spd = list(map(int, Params().get("CruiseGapBySpdSpd", encoding="utf8").split(',')))
    self.gap_by_spd_gap = list(map(int, Params().get("CruiseGapBySpdGap", encoding="utf8").split(',')))
    self.gap_by_spd_on_buffer1 = 0
    self.gap_by_spd_on_buffer2 = 0
    self.gap_by_spd_on_buffer3 = 0
    self.gap_by_spd_gap1 = False
    self.gap_by_spd_gap2 = False
    self.gap_by_spd_gap3 = False
    self.gap_by_spd_gap4 = False
    self.gap_by_spd_on_sw = False
    self.gap_by_spd_on_sw_trg = True
    self.gap_by_spd_on_sw_cnt = 0
    self.gap_by_spd_on_sw_cnt2 = 0

    self.radar_disabled_conf = self.params.get_bool("RadarDisable")
    self.prev_cruiseButton = 0
    self.gapsettingdance = 4
    self.lead_visible = False
    self.lead_debounce = 0
    self.radarDisableOverlapTimer = 0
    self.radarDisableActivated = False
    self.objdiststat = 0
    self.fca11supcnt = self.fca11inc = self.fca11alivecnt = self.fca11cnt13 = 0
    self.fca11maxcnt = 0xD

    self.steer_timer_apply_torque = 1.0
    self.DT_STEER = 0.005             # 0.01 1sec, 0.005  2sec

    self.lkas_onoff_counter = 0
    self.lkas_temp_disabled = False
    self.lkas_temp_disabled_timer = 0

    self.try_early_stop = self.params.get_bool("OPKREarlyStop")
    self.try_early_stop_retrieve = False
    self.try_early_stop_org_gap = 4.0

    self.ed_rd_diff_on = False
    self.ed_rd_diff_on_timer = 0
    self.ed_rd_diff_on_timer2 = 0

    self.vrel_delta = 0
    self.vrel_delta_prev = 0
    self.vrel_delta_timer = 0
    self.vrel_delta_timer2 = 0
    self.vrel_delta_timer3 = 0

    self.e2e_standstill_enable = self.params.get_bool("DepartChimeAtResume")
    self.e2e_standstill = False
    self.e2e_standstill_stat = False
    self.e2e_standstill_timer = 0
    self.e2e_standstill_timer_buf = 0

    self.str_log2 = 'MultiLateral'
    if CP.lateralTuning.which() == 'pid':
      self.str_log2 = 'T={:0.2f}/{:0.3f}/{:0.2f}/{:0.5f}'.format(CP.lateralTuning.pid.kpV[1], CP.lateralTuning.pid.kiV[1], CP.lateralTuning.pid.kdV[0], CP.lateralTuning.pid.kf)
    elif CP.lateralTuning.which() == 'indi':
      self.str_log2 = 'T={:03.1f}/{:03.1f}/{:03.1f}/{:03.1f}'.format(CP.lateralTuning.indi.innerLoopGainV[0], CP.lateralTuning.indi.outerLoopGainV[0], \
       CP.lateralTuning.indi.timeConstantV[0], CP.lateralTuning.indi.actuatorEffectivenessV[0])
    elif CP.lateralTuning.which() == 'lqr':
      self.str_log2 = 'T={:04.0f}/{:05.3f}/{:07.5f}'.format(CP.lateralTuning.lqr.scale, CP.lateralTuning.lqr.ki, CP.lateralTuning.lqr.dcGain)
    elif CP.lateralTuning.which() == 'torque':
      self.str_log2 = 'T={:0.2f}/{:0.2f}/{:0.2f}/{:0.3f}'.format(CP.lateralTuning.torque.kp, CP.lateralTuning.torque.kf, CP.lateralTuning.torque.ki, CP.lateralTuning.torque.friction)

    self.sm = messaging.SubMaster(['controlsState', 'radarState', 'longitudinalPlan'])


  def smooth_steer( self, apply_torque, CS ):
    if self.CP.smoothSteer.maxSteeringAngle and abs(CS.out.steeringAngleDeg) > self.CP.smoothSteer.maxSteeringAngle:
      if self.CP.smoothSteer.maxDriverAngleWait and CS.out.steeringPressed:
        self.steer_timer_apply_torque -= self.CP.smoothSteer.maxDriverAngleWait # 0.002 #self.DT_STEER   # 0.01 1sec, 0.005  2sec   0.002  5sec
      elif self.CP.smoothSteer.maxSteerAngleWait:
        self.steer_timer_apply_torque -= self.CP.smoothSteer.maxSteerAngleWait # 0.001  # 10 sec
    elif self.CP.smoothSteer.driverAngleWait and CS.out.steeringPressed:
      self.steer_timer_apply_torque -= self.CP.smoothSteer.driverAngleWait #0.001
    else:
      if self.steer_timer_apply_torque >= 1:
          return int(round(float(apply_torque)))
      self.steer_timer_apply_torque += self.DT_STEER

    if self.steer_timer_apply_torque < 0:
      self.steer_timer_apply_torque = 0
    elif self.steer_timer_apply_torque > 1:
      self.steer_timer_apply_torque = 1

    apply_torque *= self.steer_timer_apply_torque

    return  int(round(float(apply_torque)))

  def update(self, c, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert,
             left_lane, right_lane, left_lane_depart, right_lane_depart, set_speed, lead_visible, v_future, v_future_a):

    self.vFuture = v_future
    self.vFutureA = v_future_a
    path_plan = self.NC.update_lateralPlan()
    if frame % 10 == 0:
      self.model_speed = path_plan.modelSpeed

    self.sm.update(0)
    self.dRel = self.sm['radarState'].leadOne.dRel #EON Lead
    self.vRel = self.sm['radarState'].leadOne.vRel #EON Lead
    self.yRel = self.sm['radarState'].leadOne.yRel #EON Lead

    if self.enable_steer_more and self.to_avoid_lkas_fault_enabled and abs(CS.out.steeringAngleDeg) > self.to_avoid_lkas_fault_max_angle*0.5 and \
     CS.out.vEgo <= 12.5 and not (0 <= self.driver_steering_torque_above_timer < 100):
      self.steerMax = self.steerMax_Max
      self.steerDeltaUp = self.steerDeltaUp_Max
      self.steerDeltaDown = self.steerDeltaDown_Max
    elif CS.out.vEgo > 8.3:
      if self.variable_steer_max:
        self.steerMax = interp(int(abs(self.model_speed)), self.model_speed_range, self.steerMax_range)
      else:
        self.steerMax = self.steerMax_base
      if self.variable_steer_delta:
        self.steerDeltaUp = interp(int(abs(self.model_speed)), self.model_speed_range, self.steerDeltaUp_range)
        self.steerDeltaDown = interp(int(abs(self.model_speed)), self.model_speed_range, self.steerDeltaDown_range)
      else:
        self.steerDeltaUp = self.steerDeltaUp_base
        self.steerDeltaDown = self.steerDeltaDown_base
    else:
      self.steerMax = self.steerMax_base
      self.steerDeltaUp = self.steerDeltaUp_base
      self.steerDeltaDown = self.steerDeltaDown_base

    self.p.STEER_MAX = self.steerMax
    self.p.STEER_DELTA_UP = self.steerDeltaUp
    self.p.STEER_DELTA_DOWN = self.steerDeltaDown

    # Steering Torque
    if self.CP.smoothSteer.method == 1:
      new_steer = actuators.steer * self.steerMax
      new_steer = self.smooth_steer( new_steer, CS )
    elif 0 <= self.driver_steering_torque_above_timer < 100:
      new_steer = int(round(actuators.steer * self.steerMax * (self.driver_steering_torque_above_timer / 100)))
    else:
      new_steer = int(round(actuators.steer * self.steerMax))
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.p)
    self.steer_rate_limited = new_steer != apply_steer

    if self.to_avoid_lkas_fault_enabled: # Shane and Greg's idea
      lkas_active = c.active
      if lkas_active and abs(CS.out.steeringAngleDeg) > self.to_avoid_lkas_fault_max_angle:
        self.angle_limit_counter += 1
      else:
        self.angle_limit_counter = 0

      # stop requesting torque to avoid 90 degree fault and hold torque with induced temporary fault
      # two cycles avoids race conditions every few minutes
      if self.angle_limit_counter > self.to_avoid_lkas_fault_max_frame:
        self.cut_steer = True
      elif self.cut_steer_frames > 1:
        self.cut_steer_frames = 0
        self.cut_steer = False

      cut_steer_temp = False
      if self.cut_steer:
        cut_steer_temp = True
        self.angle_limit_counter = 0
        self.cut_steer_frames += 1
    else:
      if self.joystick_debug_mode:
        lkas_active = c.active
      # disable when temp fault is active, or below LKA minimum speed
      elif self.opkr_maxanglelimit == 90:
        lkas_active = c.active and abs(CS.out.steeringAngleDeg) < self.opkr_maxanglelimit and CS.out.gearShifter == GearShifter.drive
      elif self.opkr_maxanglelimit > 90:
        str_angle_limit = interp(CS.out.vEgo * CV.MS_TO_KPH, [0, 20], [self.opkr_maxanglelimit+60, self.opkr_maxanglelimit])
        lkas_active = c.active and abs(CS.out.steeringAngleDeg) < str_angle_limit and CS.out.gearShifter == GearShifter.drive
      else:
        lkas_active = c.active and CS.out.gearShifter == GearShifter.drive
      if CS.mdps_error_cnt > self.to_avoid_lkas_fault_max_frame:
        self.cut_steer = True
      elif self.cut_steer_frames > 1:
        self.cut_steer_frames = 0
        self.cut_steer = False

      cut_steer_temp = False
      if self.cut_steer:
        cut_steer_temp = True
        self.cut_steer_frames += 1

    if (( CS.out.leftBlinker and not CS.out.rightBlinker) or ( CS.out.rightBlinker and not CS.out.leftBlinker)) and CS.out.vEgo < LANE_CHANGE_SPEED_MIN and self.opkr_turnsteeringdisable:
      self.lanechange_manual_timer = 50
    if CS.out.leftBlinker and CS.out.rightBlinker:
      self.emergency_manual_timer = 50
    if self.lanechange_manual_timer:
      lkas_active = False
    if self.lanechange_manual_timer > 0:
      self.lanechange_manual_timer -= 1
    if self.emergency_manual_timer > 0:
      self.emergency_manual_timer -= 1

    if abs(CS.out.steeringTorque) > 170 and CS.out.vEgo < LANE_CHANGE_SPEED_MIN:
      self.driver_steering_torque_above = True
    else:
      self.driver_steering_torque_above = False

    if self.driver_steering_torque_above == True:
      self.driver_steering_torque_above_timer -= 1
      if self.driver_steering_torque_above_timer <= 0:
        self.driver_steering_torque_above_timer = 0
    elif self.driver_steering_torque_above == False:
      self.driver_steering_torque_above_timer += 5
      if self.driver_steering_torque_above_timer >= 100:
        self.driver_steering_torque_above_timer = 100

    if self.no_mdps_mods and CS.out.vEgo < CS.CP.minSteerSpeed:
      lkas_active = False
    if not lkas_active:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    if CS.cruise_active and CS.lead_distance > 149 and self.dRel < ((CS.out.vEgo * CV.MS_TO_KPH)+5) < 100 and \
     self.vRel*3.6 < -(CS.out.vEgo * CV.MS_TO_KPH * 0.16) and CS.out.vEgo > 7 and abs(CS.out.steeringAngleDeg) < 10 and not self.longcontrol:
      self.need_brake_timer += 1
      if self.need_brake_timer > 50:
        self.need_brake = True
    elif not CS.cruise_active and 1 < self.dRel < (CS.out.vEgo * CV.MS_TO_KPH * 0.5) < 13 and self.vRel*3.6 < -(CS.out.vEgo * CV.MS_TO_KPH * 0.6) and \
     5 < (CS.out.vEgo * CV.MS_TO_KPH) < 20 and not (CS.out.brakeLights or CS.out.brakePressed or CS.out.gasPressed): # generate an event to avoid collision when SCC is not activated at low speed.
      self.need_brake_timer += 1
      if self.need_brake_timer > 20:
        self.need_brake = True
    else:
      self.need_brake = False
      self.need_brake_timer = 0

    sys_warning, sys_state, left_lane_warning, right_lane_warning =\
      process_hud_alert(lkas_active, self.car_fingerprint, visual_alert,
                        left_lane, right_lane, left_lane_depart, right_lane_depart)

    clu11_speed = CS.clu11["CF_Clu_Vanz"]
    enabled_speed = 38 if CS.is_set_speed_in_mph else 60
    if clu11_speed > enabled_speed or not lkas_active or CS.out.gearShifter != GearShifter.drive:
      enabled_speed = clu11_speed

    if CS.cruise_active: # to toggle lkas, hold gap button for 1 sec
      if CS.cruise_buttons == 3:
        self.lkas_onoff_counter += 1
        self.gap_by_spd_on_sw = True
        self.gap_by_spd_on_sw_cnt2 = 0
        if self.lkas_onoff_counter > 100:
          self.lkas_onoff_counter = 0
          self.lkas_temp_disabled = not self.lkas_temp_disabled
          if self.lkas_temp_disabled:
            self.lkas_temp_disabled_timer = 0
          else:
            self.lkas_temp_disabled_timer = 15
      else:
        if self.lkas_temp_disabled_timer:
          self.lkas_temp_disabled_timer -= 1
        self.lkas_onoff_counter = 0
        if self.gap_by_spd_on_sw:
          self.gap_by_spd_on_sw = False
          self.gap_by_spd_on_sw_cnt += 1
          if self.gap_by_spd_on_sw_cnt > 4: #temporary disable of auto gap if you press gap button 5 times quickly.
            self.gap_by_spd_on_sw_trg = not self.gap_by_spd_on_sw_trg
            self.gap_by_spd_on_sw_cnt = 0
            self.gap_by_spd_on_sw_cnt2 = 0
        elif self.gap_by_spd_on_sw_cnt:
          self.gap_by_spd_on_sw_cnt2 += 1
          if self.gap_by_spd_on_sw_cnt2 > 20:
            self.gap_by_spd_on_sw_cnt = 0
            self.gap_by_spd_on_sw_cnt2 = 0
    else:
      self.lkas_onoff_counter = 0
      if self.lkas_temp_disabled_timer:
        self.lkas_temp_disabled_timer -= 1
      self.gap_by_spd_on_sw_cnt = 0
      self.gap_by_spd_on_sw_cnt2 = 0
      self.gap_by_spd_on_sw = False
      self.gap_by_spd_on_sw_trg = True

    can_sends = []

    if frame == 0: # initialize counts from last received count signals
      self.lkas11_cnt = CS.lkas11["CF_Lkas_MsgCount"] + 1
      self.scc12_cnt = CS.scc12["CR_VSM_Alive"] + 1 if not CS.no_radar else 0
    self.lkas11_cnt %= 0x10
    self.scc12_cnt %= 0xF

    can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active and not self.lkas_temp_disabled,
                                   cut_steer_temp, CS.lkas11, sys_warning, sys_state, enabled, left_lane, right_lane,
                                   left_lane_warning, right_lane_warning, 0, self.ldws_fix, self.lkas11_cnt))

    if CS.CP.sccBus: # send lkas11 bus 1 or 2 if scc bus is
      can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active and not self.lkas_temp_disabled,
                                   cut_steer_temp, CS.lkas11, sys_warning, sys_state, enabled, left_lane, right_lane,
                                   left_lane_warning, right_lane_warning, CS.CP.sccBus, self.ldws_fix, self.lkas11_cnt))
    if CS.CP.mdpsBus: # send lkas11 bus 1 if mdps is bus 1
      can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active and not self.lkas_temp_disabled,
                                   cut_steer_temp, CS.lkas11, sys_warning, sys_state, enabled, left_lane, right_lane,
                                   left_lane_warning, right_lane_warning, 1, self.ldws_fix, self.lkas11_cnt))
      if frame % 2: # send clu11 to mdps if it is not on bus 0
        can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.NONE, enabled_speed, CS.CP.mdpsBus))

    if CS.out.cruiseState.modeSel == 0 and self.mode_change_switch == 5:
      self.mode_change_timer = 50
      self.mode_change_switch = 0
    elif CS.out.cruiseState.modeSel == 1 and self.mode_change_switch == 0:
      self.mode_change_timer = 50
      self.mode_change_switch = 1
    elif CS.out.cruiseState.modeSel == 2 and self.mode_change_switch == 1:
      self.mode_change_timer = 50
      self.mode_change_switch = 2
    elif CS.out.cruiseState.modeSel == 3 and self.mode_change_switch == 2:
      self.mode_change_timer = 50
      self.mode_change_switch = 3
    elif CS.out.cruiseState.modeSel == 4 and self.mode_change_switch == 3:
      self.mode_change_timer = 50
      self.mode_change_switch = 4
    elif CS.out.cruiseState.modeSel == 5 and self.mode_change_switch == 4:
      self.mode_change_timer = 50
      self.mode_change_switch = 5
    if self.mode_change_timer > 0:
      self.mode_change_timer -= 1

    if pcm_cancel_cmd and self.longcontrol:
      can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.CANCEL, clu11_speed, CS.CP.sccBus))

    if CS.out.cruiseState.standstill:
      self.standstill_status = 1
      if self.opkr_autoresume:
        # run only first time when the car stopped
        if self.last_lead_distance == 0:
          # get the lead distance from the Radar
          self.last_lead_distance = CS.lead_distance
          self.resume_cnt = 0
          self.switch_timer = 0
          self.standstill_fault_reduce_timer += 1
        elif self.switch_timer > 0:
          self.switch_timer -= 1
          self.standstill_fault_reduce_timer += 1
        # at least 0.1 sec delay after entering the standstill
        elif 10 < self.standstill_fault_reduce_timer and CS.lead_distance != self.last_lead_distance and abs(CS.lead_distance - self.last_lead_distance) > 0.1:
          self.acc_standstill_timer = 0
          self.acc_standstill = False
          if self.standstill_resume_alt: # for D.Fyffe, code from neokii
            self.standstill_res_button = True
            can_sends.append(create_clu11(self.packer, self.resume_cnt, CS.clu11, Buttons.RES_ACCEL, clu11_speed, CS.CP.sccBus))
            self.resume_cnt += 1
            if self.resume_cnt >= randint(6, 8):
              self.resume_cnt = 0
              self.switch_timer = randint(30, 36)
          else:
            if (frame - self.last_resume_frame) * DT_CTRL > 0.1:
              self.standstill_res_button = True
              # send 25 messages at a time to increases the likelihood of resume being accepted, value 25 is not acceptable at some cars.
              can_sends.extend([create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL)] * self.standstill_res_count) if not self.longcontrol \
              else can_sends.extend([create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL, clu11_speed, CS.CP.sccBus)] * self.standstill_res_count)
              self.last_resume_frame = frame
          self.standstill_fault_reduce_timer += 1
        # gap save after 1sec
        elif 100 < self.standstill_fault_reduce_timer and self.cruise_gap_prev == 0 and CS.cruiseGapSet != 1.0 and self.opkr_autoresume and self.opkr_cruisegap_auto_adj and not self.gap_by_spd_on: 
          self.cruise_gap_prev = CS.cruiseGapSet
          self.cruise_gap_set_init = True
        # gap adjust to 1 for fast start
        elif 110 < self.standstill_fault_reduce_timer and CS.cruiseGapSet != 1.0 and self.opkr_autoresume and self.opkr_cruisegap_auto_adj and (not self.gap_by_spd_on or not self.gap_by_spd_on_sw_trg):
          can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
            else can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, CS.CP.sccBus))
          self.resume_cnt += 1
          if self.resume_cnt >= randint(6, 8):
            self.resume_cnt = 0
            self.switch_timer = randint(30, 36)
          self.cruise_gap_adjusting = True
        elif self.opkr_autoresume:
          self.cruise_gap_adjusting = False
          self.standstill_res_button = False
          self.standstill_fault_reduce_timer += 1
    # reset lead distnce after the car starts moving
    elif self.last_lead_distance != 0:
      self.last_lead_distance = 0
      self.standstill_res_button = False
    elif self.opkr_variablecruise and CS.acc_active and CS.out.cruiseState.modeSel > 0:
      self.on_speed_control = self.NC.onSpeedControl
      self.on_speed_bump_control = self.NC.onSpeedBumpControl
      self.curv_speed_control = self.NC.curvSpeedControl
      self.cut_in_control = self.NC.cutInControl
      self.driver_scc_set_control = self.NC.driverSccSetControl
      btn_signal = self.NC.update(CS, path_plan)
      if self.opkr_cruisegap_auto_adj and (not self.gap_by_spd_on or not self.gap_by_spd_on_sw_trg):
        # gap restore
        if self.switch_timer > 0:
          self.switch_timer -= 1
        elif self.dRel > 15 and self.vRel*3.6 < 5 and self.cruise_gap_prev != CS.cruiseGapSet and self.cruise_gap_set_init and self.opkr_autoresume:
          can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
            else can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, CS.CP.sccBus))
          self.cruise_gap_adjusting = True
          self.resume_cnt += 1
          if self.resume_cnt >= randint(6, 8):
            self.resume_cnt = 0
            self.switch_timer = randint(30, 36)
        elif self.cruise_gap_prev == CS.cruiseGapSet and CS.cruiseGapSet != 1.0 and self.opkr_autoresume:
          self.cruise_gap_set_init = False
          self.cruise_gap_prev = 0
          self.cruise_gap_adjusting = False
        else:
          self.cruise_gap_adjusting = False
      if not self.cruise_gap_adjusting:
        if not self.gap_by_spd_on or not self.gap_by_spd_on_sw_trg:
          if 0 < CS.lead_distance <= 149 and CS.lead_objspd < 0 and self.try_early_stop and CS.cruiseGapSet != 4.0 and CS.clu_Vanz > 30 and \
           0 < self.sm['longitudinalPlan'].e2eX[12] < 120 and (self.sm['longitudinalPlan'].stopLine[12] < 100 or CS.lead_objspd < -4):
            if not self.try_early_stop_retrieve:
              self.try_early_stop_org_gap = CS.cruiseGapSet
            self.try_early_stop_retrieve = True
            if self.switch_timer > 0:
              self.switch_timer -= 1
            else:
              can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
                else can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, CS.CP.sccBus))
              self.resume_cnt += 1
              if self.resume_cnt >= randint(6, 8):
                self.resume_cnt = 0
                self.switch_timer = randint(30, 36)
          elif btn_signal != None:
            can_sends.append(create_clu11(self.packer, self.resume_cnt, CS.clu11, btn_signal)) if not self.longcontrol \
            else can_sends.append(create_clu11(self.packer, frame, CS.clu11, btn_signal, clu11_speed, CS.CP.sccBus))
            self.resume_cnt += 1
          elif 0 < CS.lead_distance <= 149 and not self.cruise_gap_set_init and self.try_early_stop and self.try_early_stop_retrieve and \
           CS.cruiseGapSet != self.try_early_stop_org_gap and \
           (CS.clu_Vanz <= 20 or (CS.lead_objspd >= 0 and self.sm['longitudinalPlan'].e2eX[12] > 50 and self.sm['longitudinalPlan'].stopLine[12] > 100 and CS.clu_Vanz > 20)):
            if self.switch_timer > 0:
              self.switch_timer -= 1
            else:
              can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
                else can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, CS.CP.sccBus))
              self.resume_cnt += 1
              if self.resume_cnt >= randint(6, 8):
                self.resume_cnt = 0
                self.switch_timer = randint(30, 36)
            if CS.cruiseGapSet == self.try_early_stop_org_gap:
              self.try_early_stop_retrieve = False
          else:
            self.resume_cnt = 0
        elif self.gap_by_spd_on and self.gap_by_spd_on_sw_trg:
          if 0 < CS.lead_distance <= 149 and CS.lead_objspd < 0 and self.try_early_stop and CS.cruiseGapSet != 4.0 and CS.clu_Vanz > 30 and \
           0 < self.sm['longitudinalPlan'].e2eX[12] < 120 and (self.sm['longitudinalPlan'].stopLine[12] < 100 or CS.lead_objspd < -4):
            if not self.try_early_stop_retrieve:
              self.try_early_stop_org_gap = CS.cruiseGapSet
            self.try_early_stop_retrieve = True
            if self.switch_timer > 0:
              self.switch_timer -= 1
            else:
              can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
                else can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, CS.CP.sccBus))
              self.resume_cnt += 1
              if self.resume_cnt >= randint(6, 8):
                self.resume_cnt = 0
                self.switch_timer = randint(30, 36)
              self.switch_timer2 = randint(30, 36)
          elif self.switch_timer > 0 and not self.try_early_stop_retrieve:
            self.switch_timer -= 1
          elif CS.cruiseGapSet != self.gap_by_spd_gap[0] and ((CS.clu_Vanz < self.gap_by_spd_spd[0]+self.gap_by_spd_on_buffer1) or self.gap_by_spd_gap1) and not self.try_early_stop_retrieve:
            self.gap_by_spd_gap1 = True
            self.gap_by_spd_gap2 = False
            self.gap_by_spd_gap3 = False
            self.gap_by_spd_gap4 = False
            self.gap_by_spd_on_buffer1 = 0
            self.gap_by_spd_on_buffer2 = 0
            can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
              else can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, CS.CP.sccBus))
            self.resume_cnt += 1
            if self.resume_cnt >= randint(6, 8):
              self.resume_cnt = 0
              self.switch_timer = randint(30, 36)
          elif CS.cruiseGapSet != self.gap_by_spd_gap[1] and ((self.gap_by_spd_spd[0] <= CS.clu_Vanz < self.gap_by_spd_spd[1]+self.gap_by_spd_on_buffer2) or self.gap_by_spd_gap2) and not self.try_early_stop_retrieve:
            self.gap_by_spd_gap1 = False
            self.gap_by_spd_gap2 = True
            self.gap_by_spd_gap3 = False
            self.gap_by_spd_gap4 = False
            self.gap_by_spd_on_buffer1 = -5
            self.gap_by_spd_on_buffer3 = 0
            can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
              else can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, CS.CP.sccBus))
            self.resume_cnt += 1
            if self.resume_cnt >= randint(6, 8):
              self.resume_cnt = 0
              self.switch_timer = randint(30, 36)
          elif CS.cruiseGapSet != self.gap_by_spd_gap[2] and ((self.gap_by_spd_spd[1] <= CS.clu_Vanz < self.gap_by_spd_spd[2]+self.gap_by_spd_on_buffer3) or self.gap_by_spd_gap3) and not self.try_early_stop_retrieve:
            self.gap_by_spd_gap1 = False
            self.gap_by_spd_gap2 = False
            self.gap_by_spd_gap3 = True
            self.gap_by_spd_gap4 = False
            self.gap_by_spd_on_buffer2 = -5
            can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
              else can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, CS.CP.sccBus))
            self.resume_cnt += 1
            if self.resume_cnt >= randint(6, 8):
              self.resume_cnt = 0
              self.switch_timer = randint(30, 36)
          elif CS.cruiseGapSet != self.gap_by_spd_gap[3] and ((self.gap_by_spd_spd[2] <= CS.clu_Vanz) or self.gap_by_spd_gap4) and not self.try_early_stop_retrieve:
            self.gap_by_spd_gap1 = False
            self.gap_by_spd_gap2 = False
            self.gap_by_spd_gap3 = False
            self.gap_by_spd_gap4 = True
            self.gap_by_spd_on_buffer3 = -5
            can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
              else can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, CS.CP.sccBus))
            self.resume_cnt += 1
            if self.resume_cnt >= randint(6, 8):
              self.resume_cnt = 0
              self.switch_timer = randint(30, 36)
          elif btn_signal != None:
            if self.switch_timer2 > 0 and self.try_early_stop_retrieve:
              self.switch_timer2 -= 1
            else:
              can_sends.append(create_clu11(self.packer, self.resume_cnt, CS.clu11, btn_signal)) if not self.longcontrol \
              else can_sends.append(create_clu11(self.packer, frame, CS.clu11, btn_signal, clu11_speed, CS.CP.sccBus))
            self.resume_cnt += 1
            self.gap_by_spd_gap1 = False
            self.gap_by_spd_gap2 = False
            self.gap_by_spd_gap3 = False
            self.gap_by_spd_gap4 = False
          elif 0 < CS.lead_distance <= 149 and not self.cruise_gap_set_init and self.try_early_stop and self.try_early_stop_retrieve and \
           CS.cruiseGapSet != self.try_early_stop_org_gap and \
           (CS.clu_Vanz <= 20 or (CS.lead_objspd >= 0 and self.sm['longitudinalPlan'].e2eX[12] > 50 and self.sm['longitudinalPlan'].stopLine[12] > 100 and CS.clu_Vanz > 20)):
            if self.switch_timer > 0:
              self.switch_timer -= 1
            else:
              can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
                else can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, CS.CP.sccBus))
              self.resume_cnt += 1
              if self.resume_cnt >= randint(6, 8):
                self.resume_cnt = 0
                self.switch_timer = randint(30, 36)
              self.switch_timer2 = randint(30, 36)
            if CS.cruiseGapSet == self.try_early_stop_org_gap:
              self.try_early_stop_retrieve = False
            self.gap_by_spd_gap1 = False
            self.gap_by_spd_gap2 = False
            self.gap_by_spd_gap3 = False
            self.gap_by_spd_gap4 = False
          elif 0 < CS.lead_distance <= 149 and not self.cruise_gap_set_init and self.try_early_stop and self.try_early_stop_retrieve and \
           CS.cruiseGapSet == self.try_early_stop_org_gap and \
           (CS.clu_Vanz <= 20 or (CS.lead_objspd >= 0 and self.sm['longitudinalPlan'].e2eX[12] > 50 and self.sm['longitudinalPlan'].stopLine[12] > 100 and CS.clu_Vanz > 20)):
            self.try_early_stop_retrieve = False
            self.gap_by_spd_gap1 = False
            self.gap_by_spd_gap2 = False
            self.gap_by_spd_gap3 = False
            self.gap_by_spd_gap4 = False
          else:
            self.resume_cnt = 0
            self.gap_by_spd_gap1 = False
            self.gap_by_spd_gap2 = False
            self.gap_by_spd_gap3 = False
            self.gap_by_spd_gap4 = False

    else:
      self.on_speed_control = False
      self.on_speed_bump_control = False
      self.curv_speed_control = False
      self.cut_in_control = False
      self.driver_scc_set_control = False
      self.cruise_gap_adjusting = False
      self.standstill_res_button = False
      self.auto_res_starting = False
      self.gap_by_spd_gap1 = False
      self.gap_by_spd_gap2 = False
      self.gap_by_spd_gap3 = False
      self.gap_by_spd_gap4 = False

    if not enabled:
      self.cruise_init = False
      self.lkas_temp_disabled = False
      self.e2e_standstill = False
      self.e2e_standstill_stat = False
      self.e2e_standstill_timer = 0
      self.e2e_standstill_timer_buf = 0
    if CS.cruise_buttons == 4:
      self.cancel_counter += 1
      self.auto_res_starting = False
      self.standstill_res_button = False
    elif CS.cruise_buttons == 3:
      self.try_early_stop_retrieve = False
      self.try_early_stop_org_gap = CS.cruiseGapSet
      self.gap_by_spd_gap1 = False
      self.gap_by_spd_gap2 = False
      self.gap_by_spd_gap3 = False
      self.gap_by_spd_gap4 = False
    elif CS.cruise_active:
      self.cruise_init = True
      self.cancel_counter = 0
      self.auto_res_limit_timer = 0
      self.auto_res_delay_timer = 0          
      self.e2e_standstill = False
      self.e2e_standstill_stat = False
      self.e2e_standstill_timer = 0
      self.e2e_standstill_timer_buf = 0
      if self.res_speed_timer > 0:
        self.res_speed_timer -= 1
        self.auto_res_starting = False
      else:
        self.auto_res_starting = False
        self.v_cruise_kph_auto_res = 0
        self.res_speed = 0
    else:
      if CS.out.brakeLights:
        self.auto_res_limit_timer = 0
        self.auto_res_delay_timer = 0
      else:
        if self.auto_res_limit_timer < self.auto_res_limit_sec:
          self.auto_res_limit_timer += 1
        if self.auto_res_delay_timer < self.auto_res_delay:
          self.auto_res_delay_timer += 1

      if self.e2e_standstill_enable:
        try:
          if self.e2e_standstill:
            self.e2e_standstill_timer += 1
            if self.e2e_standstill_timer > 100:
              self.e2e_standstill = False
              self.e2e_standstill_timer = 0
          elif CS.clu_Vanz > 0:
            self.e2e_standstill = False
            self.e2e_standstill_stat = False
            self.e2e_standstill_timer = 0
            self.e2e_standstill_timer_buf = 0
          elif self.e2e_standstill_stat and self.sm['longitudinalPlan'].e2eX[12] > 30 and self.sm['longitudinalPlan'].stopLine[12] < 10 and CS.clu_Vanz == 0:
            self.e2e_standstill = True
            self.e2e_standstill_stat = False
            self.e2e_standstill_timer = 0
            self.e2e_standstill_timer_buf += 300
          elif 0 < self.sm['longitudinalPlan'].e2eX[12] < 10 and self.sm['longitudinalPlan'].stopLine[12] < 10 and CS.clu_Vanz == 0:
            self.e2e_standstill_timer += 1
            if self.e2e_standstill_timer > (300 + self.e2e_standstill_timer_buf):
              self.e2e_standstill_timer = 101
              self.e2e_standstill_stat = True
          else:
            self.e2e_standstill_timer = 0
            self.e2e_standstill_timer_buf = 0
        except:
          pass

    if CS.brakeHold and not self.autohold_popup_switch:
      self.autohold_popup_timer = 100
      self.autohold_popup_switch = True
    elif CS.brakeHold and self.autohold_popup_switch and self.autohold_popup_timer:
      self.autohold_popup_timer -= 1
    elif not CS.brakeHold and self.autohold_popup_switch:
      self.autohold_popup_switch = False
      self.autohold_popup_timer = 0

    opkr_cruise_auto_res_condition = False
    opkr_cruise_auto_res_condition = not self.opkr_cruise_auto_res_condition or CS.out.gasPressed
    t_speed = 20 if CS.is_set_speed_in_mph else 30
    if self.auto_res_timer > 0:
      self.auto_res_timer -= 1
    elif self.model_speed > 95 and self.cancel_counter == 0 and not CS.cruise_active and not CS.out.brakeLights and round(CS.VSetDis) >= t_speed and \
     (1 < CS.lead_distance < 149 or round(CS.clu_Vanz) > t_speed) and round(CS.clu_Vanz) >= 3 and self.cruise_init and \
     self.opkr_cruise_auto_res and opkr_cruise_auto_res_condition and (self.auto_res_limit_sec == 0 or self.auto_res_limit_timer < self.auto_res_limit_sec) and \
     (self.auto_res_delay == 0 or self.auto_res_delay_timer >= self.auto_res_delay):
      if self.opkr_cruise_auto_res_option == 0:
        can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL)) if not self.longcontrol \
         else can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL, clu11_speed, CS.CP.sccBus))  # auto res
        self.auto_res_starting = True
        self.res_speed = round(CS.VSetDis) if CS.is_set_speed_in_mph or self.osm_spdlimit_enabled else round(CS.clu_Vanz*1.1)
        self.res_speed_timer = 300
        self.resume_cnt += 1
        if self.resume_cnt >= randint(6, 8):
          self.resume_cnt = 0
          self.auto_res_timer = randint(30, 36)
      elif self.opkr_cruise_auto_res_option == 1:
        can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.SET_DECEL)) if not self.longcontrol \
         else can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.SET_DECEL, clu11_speed, CS.CP.sccBus)) # auto res but set_decel to set current speed
        self.auto_res_starting = True
        self.v_cruise_kph_auto_res = round(CS.clu_Vanz)
        self.res_speed_timer = 50
        self.resume_cnt += 1
        if self.resume_cnt >= randint(6, 8):
          self.resume_cnt = 0
          self.auto_res_timer = randint(30, 36)
      elif self.opkr_cruise_auto_res_option == 2:
        if not self.longcontrol:
          can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL)) if 1 < CS.lead_distance < 149 \
           else can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.SET_DECEL))
        else:
          can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL, clu11_speed, CS.CP.sccBus)) if 1 < CS.lead_distance < 149 \
           else can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.SET_DECEL, clu11_speed, CS.CP.sccBus))
        self.auto_res_starting = True
        self.v_cruise_kph_auto_res = round(CS.clu_Vanz)
        self.res_speed_timer = 50
        self.resume_cnt += 1
        if self.resume_cnt >= randint(6, 8):
          self.resume_cnt = 0
          self.auto_res_timer = randint(30, 36)

    if CS.out.brakeLights and CS.out.vEgo == 0 and not CS.out.cruiseState.standstill:
      self.standstill_status_timer += 1
      if self.standstill_status_timer > 200:
        self.standstill_status = 1
        self.standstill_status_timer = 0
    if self.standstill_status == 1 and CS.out.vEgo > 1:
      self.standstill_status = 0
      self.standstill_fault_reduce_timer = 0
      self.last_resume_frame = frame
      self.res_switch_timer = 0
      self.resume_cnt = 0

    if CS.out.vEgo <= 1:
      long_control_state = self.sm['controlsState'].longControlState
      if long_control_state == LongCtrlState.stopping and CS.out.vEgo < 0.1 and not CS.out.gasPressed:
        self.acc_standstill_timer += 1
        if self.acc_standstill_timer >= 200:
          self.acc_standstill_timer = 200
          self.acc_standstill = True
      else:
        self.acc_standstill_timer = 0
        self.acc_standstill = False
    elif CS.out.gasPressed or CS.out.vEgo > 1:
      self.acc_standstill = False
      self.acc_standstill_timer = 0      
    else:
      self.acc_standstill = False
      self.acc_standstill_timer = 0

    if CS.CP.mdpsBus: # send mdps12 to LKAS to prevent LKAS error
      can_sends.append(create_mdps12(self.packer, frame, CS.mdps12))

    # # tester present - w/ no response (keeps radar disabled)
    # if CS.CP.openpilotLongitudinalControl:
    #   if (frame % 100) == 0:
    #     can_sends.append([0x7D0, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", 0])

    # if frame % 2 == 0 and CS.CP.openpilotLongitudinalControl:
    #   lead_visible = False
    #   accel = actuators.accel if enabled else 0
    #   jerk = clip(2.0 * (accel - CS.out.aEgo), -12.7, 12.7)
    #   if accel < 0:
    #     accel = interp(accel - CS.out.aEgo, [-1.0, -0.5], [2 * accel, accel])
    #   accel = clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
    #   stopping = (actuators.longControlState == LongCtrlState.stopping)
    #   set_speed_in_units = hud_speed * (CV.MS_TO_MPH if CS.clu11["CF_Clu_SPEED_UNIT"] == 1 else CV.MS_TO_KPH)
    #   can_sends.extend(create_acc_commands(self.packer, enabled, accel, jerk, int(frame / 2), lead_visible, set_speed_in_units, stopping))

    if self.radar_disabled_conf: #xps-genesis's way
      if self.prev_cruiseButton != CS.cruise_buttons:  # gap change for RadarDisable
        if CS.cruise_buttons == 3:
          self.gapsettingdance -= 1
        if self.gapsettingdance < 1:
          self.gapsettingdance = 4
        self.prev_cruiseButton = CS.cruise_buttons
      if lead_visible:
        self.lead_visible = True
        self.lead_debounce = 50
      elif self.lead_debounce > 0:
        self.lead_debounce -= 1
      else:
        self.lead_visible = lead_visible
      self.radarDisableOverlapTimer += 1
      if self.radarDisableOverlapTimer >= 30:
        self.radarDisableActivated = True
        if 200 > self.radarDisableOverlapTimer > 36:
          if frame % 41 == 0 or self.radarDisableOverlapTimer == 37:
            can_sends.append(create_scc7d0(b'\x02\x10\x03\x00\x00\x00\x00\x00'))
          elif frame % 43 == 0 or self.radarDisableOverlapTimer == 37:
            can_sends.append(create_scc7d0(b'\x03\x28\x03\x01\x00\x00\x00\x00'))
          elif frame % 19 == 0 or self.radarDisableOverlapTimer == 37:
            can_sends.append(create_scc7d0(b'\x02\x10\x85\x00\x00\x00\x00\x00')) # off
      else:
        self.counter_init = False
        can_sends.append(create_scc7d0(b'\x02\x10\x90\x00\x00\x00\x00\x00')) # on
        can_sends.append(create_scc7d0(b'\x03\x29\x03\x01\x00\x00\x00\x00'))
      if (frame % 50 == 0 or self.radarDisableOverlapTimer == 37) and self.radarDisableOverlapTimer >= 30:
        can_sends.append(create_scc7d0(b'\x02\x3E\x00\x00\x00\x00\x00\x00'))
      if self.radarDisableOverlapTimer > 200:
        self.radarDisableOverlapTimer = 200
      if self.lead_visible:
        self.objdiststat = 1 if self.dRel < 25 else 2 if self.dRel < 40 else 3 if self.dRel < 60 else 4 if self.dRel < 80 else 5
      else:
        self.objdiststat = 0

    setSpeed = round(set_speed * CV.MS_TO_KPH)

    if (CS.CP.sccBus != 0 or self.radarDisableActivated) and self.counter_init and self.longcontrol:
      if frame % 2 == 0:
        if self.radar_disabled_conf:
          self.fca11supcnt += 1
          self.fca11supcnt %= 0xF
          if self.fca11alivecnt == 1:
            self.fca11inc = 0
            if self.fca11cnt13 == 3:
              self.fca11maxcnt = 0x9
              self.fca11cnt13 = 0
            else:
              self.fca11maxcnt = 0xD
              self.fca11cnt13 += 1
          else:
            self.fca11inc += 4
          self.fca11alivecnt = self.fca11maxcnt - self.fca11inc
          if CS.CP.fcaBus == -1:
            can_sends.append(create_fca11(self.packer, CS.fca11, self.fca11alivecnt, self.fca11supcnt))

        self.scc12cnt += 1
        self.scc12cnt %= 0xF
        self.scc11cnt += 1
        self.scc11cnt %= 0x10
        lead_objspd = CS.lead_objspd  # vRel (km/h)
        aReqValue = CS.scc12["aReqValue"]
        faccel = actuators.accel if c.active and not CS.out.gasPressed else 0
        accel = actuators.oaccel if c.active and not CS.out.gasPressed else 0
        stopping = (actuators.longControlState == LongCtrlState.stopping)
        radar_recog = (0 < CS.lead_distance <= 149)
        if self.joystick_debug_mode:
          accel = actuators.accel
        elif self.radar_helper_option == 0: # Vision Only
          if 0 < CS.lead_distance <= 4.0: # use radar by force to stop anyway below 4.0m if lead car is detected.
            stock_weight = interp(CS.lead_distance, [2.5, 4.0], [1., 0.])
            accel = accel * (1. - stock_weight) + aReqValue * stock_weight
          elif 0.1 < self.dRel < 6.0 and self.vRel < 0:
            accel = self.accel - (DT_CTRL * interp(CS.out.vEgo, [0.9, 3.0], [1.0, 3.0]))
            self.stopped = False
          elif 0.1 < self.dRel < 6.0:
            accel = min(-0.5, faccel*0.3)
            if stopping:
              self.stopped = True
            else:
              self.stopped = False
          elif 0.1 < self.dRel:
            self.stopped = False
            pass
          else:
            self.stopped = False
            accel = aReqValue
        elif self.radar_helper_option == 1: # Radar Only
          accel = aReqValue
        elif self.radar_helper_option >= 2: # OPKR Custom(Radar+Vision), more smooth slowdown for cut-in or encountering being decellerated car.
          if 0 < CS.lead_distance <= 149:
            stock_weight = 0.0
            self.smooth_start = False
            self.vrel_delta_timer2 += 1
            if self.vrel_delta_timer2 > 10:
              self.vrel_delta_timer2 = 0
              self.vrel_delta = (self.vRel*3.6) - self.vrel_delta_prev
              self.vrel_delta_prev = self.vRel*3.6
            if accel > 0 and self.change_accel_fast and CS.out.vEgo < 11.:
              if aReqValue >= accel:
                self.change_accel_fast = False
              else:
                accel = (aReqValue + accel) / 2
            elif aReqValue < 0 and accel > 0 and accel - aReqValue > 0.3 and lead_objspd > 0 and CS.out.vEgo < 11.:
              self.change_accel_fast = True
            elif 0.1 < self.dRel < 6 and CS.lead_distance < 30.0 and lead_objspd > 0 and aReqValue - accel > 0.8: # in case radar detection works during vision breaking at stop.
              accel = interp(aReqValue, [0.0, 1.8], [0.0, -0.7])
              self.change_accel_fast = False
            elif 0.1 < self.dRel <= 10.0 and CS.lead_distance - self.dRel >= 5.0 and aReqValue >= 0:
              self.change_accel_fast = False
              pass
            elif aReqValue >= 0.0:
              # accel = interp(CS.lead_distance, [14.0, 15.0], [max(accel, aReqValue, faccel), aReqValue])
              dRel1 = self.dRel if self.dRel > 0 else CS.lead_distance
              if ((CS.lead_distance - dRel1 > 3.0) or self.NC.cutInControl) and accel < 0:
                if aReqValue < accel:
                  accel = interp(lead_objspd, [-1, 0, 5], [aReqValue, aReqValue, accel])
                else:
                  accel = accel*0.8
              else:
                accel = aReqValue
            elif aReqValue < 0.0 and CS.lead_distance < self.stoppingdist and accel >= aReqValue and lead_objspd <= 0 and self.stopping_dist_adj_enabled:
              if CS.lead_distance < 1.8:
                accel = self.accel - (DT_CTRL * 4.0)
              elif CS.lead_distance < self.stoppingdist:
                accel = self.accel - (DT_CTRL * interp(CS.out.vEgo, [0.0, 1.0, 2.0], [0.05, 1.0, 5.0]))
            elif aReqValue < 0.0:
              dRel2 = self.dRel if self.dRel > 0 else CS.lead_distance
              if ((CS.lead_distance - dRel2 > 3.0) or self.NC.cutInControl) and accel < 0:
                stock_weight = 0.3
                if aReqValue < accel:
                  stock_weight = interp(lead_objspd, [-1, 0, 5], [1.0, 1.0, 0.0])
              elif ((dRel2 - CS.lead_distance > 3.0) or self.NC.cutInControl) and not self.ed_rd_diff_on:
                self.ed_rd_diff_on = True
                self.ed_rd_diff_on_timer = min(400, int(self.dRel * 10))
                self.ed_rd_diff_on_timer2 = min(400, int(self.dRel * 10))
                stock_weight = 1.0
              elif self.ed_rd_diff_on_timer: # damping btw ED and RD for few secs.
                stock_weight = interp(self.ed_rd_diff_on_timer, [0, self.ed_rd_diff_on_timer2], [0.1, 1.0])
                self.ed_rd_diff_on_timer -= 1
                if aReqValue <= accel:
                  stock_weight = 1.0
              else:
                if not self.NC.cutInControl:
                  self.ed_rd_diff_on = False
                self.ed_rd_diff_on_timer = 0
                self.ed_rd_diff_on_timer2 = 0
                stock_weight = interp(abs(lead_objspd), [1.0, 4.0, 8.0, 20.0, 50.0], [0.2, 0.3, 1.0, 0.9, 0.2])
                if aReqValue <= accel:
                  self.vrel_delta_timer = 0
                  self.vrel_delta_timer3 = 0
                  stock_weight = min(1.0, interp(CS.out.vEgo, [7.0, 30.0], [stock_weight, stock_weight*5.0]))
                  if not self.stopping_dist_adj_enabled:
                    stock_weight = min(1.0, interp(CS.lead_distance, [0.0, 10.0], [stock_weight*5.0, stock_weight]))
                elif aReqValue > accel:
                  if self.vrel_delta < -5 and self.vrel_delta_timer == 0:
                    self.vrel_delta_timer = min(400, int(self.dRel*10))
                    self.vrel_delta_timer3 = min(400, int(self.dRel*10))
                    stock_weight = 1.0
                  elif self.vrel_delta_timer > 0:
                    self.vrel_delta_timer -= 1
                    stock_weight = interp(self.vrel_delta_timer, [0, self.vrel_delta_timer3], [0.1, 1.0])
                  else:
                    self.vrel_delta_timer = 0
                    self.vrel_delta_timer3 = 0
                    stock_weight = interp(abs(lead_objspd), [1.0, 10.0], [1.0, 0.0])
              accel = accel * (1.0 - stock_weight) + aReqValue * stock_weight
              accel = min(accel, -0.5) if CS.lead_distance <= 4.5 and not CS.out.standstill else accel
            # elif aReqValue < 0.0:
            #   stock_weight = interp(CS.lead_distance, [6.0, 10.0, 18.0, 25.0, 32.0], [1.0, 0.85, 1.0, 0.4, 1.0])
            #   accel = accel * (1.0 - stock_weight) + aReqValue * stock_weight
            else:
              stock_weight = 0.0
              self.change_accel_fast = False
              accel = accel * (1.0 - stock_weight) + aReqValue * stock_weight
          elif 0.1 < self.dRel < 6.0 and int(self.vRel*3.6) < 0:
            accel = self.accel - (DT_CTRL * interp(CS.out.vEgo, [0.9, 3.0], [1.0, 3.0]))
            self.stopped = False
          elif 0.1 < self.dRel < 6.0:
            accel = min(-0.5, faccel*0.3)
            if stopping:
              self.stopped = True
            else:
              self.stopped = False
          elif 0.1 < self.dRel < 80:
            self.stopped = False
            pass
          else:
            self.stopped = False
            if self.stopsign_enabled:
              if self.sm['longitudinalPlan'].longitudinalPlanSource == LongitudinalPlanSource.stop:
                self.smooth_start = True
                accel = faccel if faccel <= 0 else faccel*0.5
              elif self.smooth_start and CS.clu_Vanz < setSpeed:
                accel = interp(CS.clu_Vanz, [0, setSpeed], [faccel, aReqValue])
              else:
                self.smooth_start = False
                accel = aReqValue
            else:
              accel = aReqValue
        else:
          self.stopped = False
          stock_weight = 0.

        if self.stock_safety_decel_enabled:
          if CS.scc11["Navi_SCC_Camera_Act"] == 2 and accel > aReqValue:
            accel = aReqValue
        accel = clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
        self.aq_value = accel
        self.aq_value_raw = aReqValue
        can_sends.append(create_scc11(self.packer, frame, set_speed, lead_visible, self.scc_live, self.dRel, self.vRel, self.yRel, 
         self.car_fingerprint, CS.out.vEgo * CV.MS_TO_KPH, self.acc_standstill, self.gapsettingdance, self.stopped, radar_recog, CS.scc11))
        if (CS.brake_check or CS.cancel_check) and self.car_fingerprint != CAR.NIRO_EV_DE:
          can_sends.append(create_scc12(self.packer, accel, enabled, self.scc_live, CS.out.gasPressed, 1, 
           CS.out.stockAeb, self.car_fingerprint, CS.out.vEgo * CV.MS_TO_KPH, self.stopped, self.acc_standstill, radar_recog, self.scc12_cnt, CS.scc12))
        else:
          can_sends.append(create_scc12(self.packer, accel, enabled, self.scc_live, CS.out.gasPressed, CS.out.brakePressed, 
           CS.out.stockAeb, self.car_fingerprint, CS.out.vEgo * CV.MS_TO_KPH, self.stopped, self.acc_standstill, radar_recog, self.scc12_cnt, CS.scc12))
        self.scc12_cnt += 1
        can_sends.append(create_scc14(self.packer, enabled, CS.scc14, CS.out.stockAeb, lead_visible, self.dRel, 
         CS.out.vEgo, self.acc_standstill, self.car_fingerprint))
        self.accel = accel
      if frame % 20 == 0:
        if self.radar_disabled_conf:
          if CS.CP.fcaBus == -1:
            can_sends.append(create_fca12(self.packer))
        can_sends.append(create_scc13(self.packer, CS.scc13))
      if frame % 50 == 0:
        can_sends.append(create_scc42a(self.packer))
    elif (CS.CP.sccBus != 0 or self.radarDisableActivated) and self.longcontrol:
      if self.radar_disabled_conf:
        self.fca11alivecnt = CS.fca11init["CR_FCA_Alive"]
        self.fca11supcnt = CS.fca11init["Supplemental_Counter"]
      self.counter_init = True
      self.scc12cnt = CS.scc12init["CR_VSM_Alive"]
      self.scc11cnt = CS.scc11init["AliveCounterACC"]

    str_log1 = 'MD={}  BS={:1.0f}/{:1.0f}  CV={:03.0f}/{:0.4f}  TQ={:03.0f}/{:03.0f}  VF={:03.0f}  ST={:03.0f}/{:01.0f}/{:01.0f}  FR={:03.0f}'.format(
      CS.out.cruiseState.modeSel, CS.CP.mdpsBus, CS.CP.sccBus, self.model_speed, abs(self.sm['controlsState'].curvature), abs(new_steer), abs(CS.out.steeringTorque), v_future, self.p.STEER_MAX, self.p.STEER_DELTA_UP, self.p.STEER_DELTA_DOWN, self.timer1.sampleTime())
    if CS.out.cruiseState.accActive:
      str_log2 = 'AQ={:+04.2f}  VF={:03.0f}/{:03.0f}  TS={:03.0f}  SS/VS={:03.0f}/{:03.0f}  RD/LD={:04.1f}/{:03.1f}  CG={:1.0f}  FR={:03.0f}'.format(
       self.aq_value if self.longcontrol else CS.scc12["aReqValue"], v_future, v_future_a, self.NC.ctrl_speed , setSpeed, round(CS.VSetDis), CS.lead_distance, self.last_lead_distance, CS.cruiseGapSet, self.timer1.sampleTime())
    else:
      str_log2 = 'MDPS={}  LKAS={}  LEAD={}  AQ={:+04.2f}  VF={:03.0f}/{:03.0f}  CG={:1.0f}  FR={:03.0f}'.format(
       CS.out.steerFaultTemporary, CS.lkas_button_on, 0 < CS.lead_distance < 149, self.aq_value if self.longcontrol else CS.scc12["aReqValue"], v_future, v_future_a, CS.cruiseGapSet, self.timer1.sampleTime())
    trace1.printf2( '{}'.format( str_log2 ) )

    # str_log3 = 'V/D/R/A/M/G={:.1f}/{:.1f}/{:.1f}/{:.2f}/{:.1f}/{:1.0f}'.format(CS.clu_Vanz, CS.lead_distance, CS.lead_objspd, CS.scc12["aReqValue"], self.stoppingdist, CS.cruiseGapSet)
    # trace1.printf3('{}'.format(str_log3))

    self.cc_timer += 1
    if self.cc_timer > 100:
      self.cc_timer = 0
      # self.radar_helper_option = int(self.params.get("RadarLongHelper", encoding="utf8"))
      # self.stopping_dist_adj_enabled = self.params.get_bool("StoppingDistAdj")
      # self.standstill_res_count = int(self.params.get("RESCountatStandstill", encoding="utf8"))
      # self.opkr_cruisegap_auto_adj = self.params.get_bool("CruiseGapAdjust")
      # self.to_avoid_lkas_fault_enabled = self.params.get_bool("AvoidLKASFaultEnabled")
      # self.to_avoid_lkas_fault_max_angle = int(self.params.get("AvoidLKASFaultMaxAngle", encoding="utf8"))
      # self.to_avoid_lkas_fault_max_frame = int(self.params.get("AvoidLKASFaultMaxFrame", encoding="utf8"))
      # self.e2e_long_enabled = self.params.get_bool("E2ELong")
      # self.stopsign_enabled = self.params.get_bool("StopAtStopSign")
      # self.gap_by_spd_on = self.params.get_bool("CruiseGapBySpdOn")
      if self.params.get_bool("OpkrLiveTunePanelEnable"):
        if CS.CP.lateralTuning.which() == 'pid':
          self.str_log2 = 'T={:0.2f}/{:0.3f}/{:0.1f}/{:0.5f}'.format(float(Decimal(self.params.get("PidKp", encoding="utf8"))*Decimal('0.01')), \
           float(Decimal(self.params.get("PidKi", encoding="utf8"))*Decimal('0.001')), float(Decimal(self.params.get("PidKd", encoding="utf8"))*Decimal('0.01')), \
           float(Decimal(self.params.get("PidKf", encoding="utf8"))*Decimal('0.00001')))
        elif CS.CP.lateralTuning.which() == 'indi':
          self.str_log2 = 'T={:03.1f}/{:03.1f}/{:03.1f}/{:03.1f}'.format(float(Decimal(self.params.get("InnerLoopGain", encoding="utf8"))*Decimal('0.1')), \
           float(Decimal(self.params.get("OuterLoopGain", encoding="utf8"))*Decimal('0.1')), float(Decimal(self.params.get("TimeConstant", encoding="utf8"))*Decimal('0.1')), \
           float(Decimal(self.params.get("ActuatorEffectiveness", encoding="utf8"))*Decimal('0.1')))
        elif CS.CP.lateralTuning.which() == 'lqr':
          self.str_log2 = 'T={:04.0f}/{:05.3f}/{:07.5f}'.format(float(Decimal(self.params.get("Scale", encoding="utf8"))*Decimal('1.0')), \
           float(Decimal(self.params.get("LqrKi", encoding="utf8"))*Decimal('0.001')), float(Decimal(self.params.get("DcGain", encoding="utf8"))*Decimal('0.00001')))
        elif CS.CP.lateralTuning.which() == 'torque':
          max_lat_accel = float(Decimal(self.params.get("TorqueMaxLatAccel", encoding="utf8"))*Decimal('0.1'))
          self.str_log2 = 'T={:0.2f}/{:0.2f}/{:0.2f}/{:0.3f}'.format(float(Decimal(self.params.get("TorqueKp", encoding="utf8"))*Decimal('0.1'))/max_lat_accel, \
           float(Decimal(self.params.get("TorqueKf", encoding="utf8"))*Decimal('0.1'))/max_lat_accel, float(Decimal(self.params.get("TorqueKi", encoding="utf8"))*Decimal('0.1'))/max_lat_accel, \
           float(Decimal(self.params.get("TorqueFriction", encoding="utf8")) * Decimal('0.001')))

    trace1.printf1('{}  {}'.format(str_log1, self.str_log2))

    # 20 Hz LFA MFA message
    if frame % 5 == 0 and self.car_fingerprint in FEATURES["send_lfahda_mfa"]:
      can_sends.append(create_lfahda_mfc(self.packer, lkas_active))

    elif frame % 5 == 0 and self.car_fingerprint in FEATURES["send_hda_mfa"]:
      can_sends.append(create_hda_mfc(self.packer, CS, enabled, left_lane, right_lane))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.p.STEER_MAX
    new_actuators.accel = self.accel
    safetycam_speed = self.NC.safetycam_speed

    self.lkas11_cnt += 1

    return new_actuators, can_sends, safetycam_speed, self.lkas_temp_disabled, (self.gap_by_spd_on_sw_trg and self.gap_by_spd_on)
