from cereal import car, log
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
from selfdrive.controls.lib.pid import LongPIDController
from selfdrive.controls.lib.drive_helpers import CONTROL_N
from selfdrive.modeld.constants import T_IDXS

from selfdrive.car.hyundai.values import CAR
from common.conversions import Conversions as CV
from common.params import Params
from decimal import Decimal
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc

import common.log as trace1
LongitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource

LongCtrlState = car.CarControl.Actuators.LongControlState

# As per ISO 15622:2018 for all speeds
ACCEL_MIN_ISO = -4.0  # m/s^2
ACCEL_MAX_ISO = 2.0  # m/s^2


def long_control_state_trans(CP, active, long_control_state, v_ego, v_target,
                             v_target_future, brake_pressed, cruise_standstill, stop, gas_pressed):
  """Update longitudinal control state machine"""
  accelerating = v_target_future > v_target
  stopping_condition = stop or (v_ego < 2.0 and cruise_standstill) or \
                       (v_ego < CP.vEgoStopping and
                        ((v_target_future < CP.vEgoStopping and not accelerating) or brake_pressed))

  starting_condition = v_target_future > CP.vEgoStarting and accelerating and not cruise_standstill or gas_pressed

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition:
        long_control_state = LongCtrlState.pid

  return long_control_state


class LongControl():
  def __init__(self, CP, candidate):
    self.long_control_state = LongCtrlState.off  # initialized to off

    self.pid = LongPIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                                 (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                                 (CP.longitudinalTuning.kdBP, CP.longitudinalTuning.kdV),
                                 (CP.longitudinalTuning.kfBP, CP.longitudinalTuning.kfV),
                                 rate=1 / DT_CTRL)
    self.v_pid = 0.0
    self.last_output_accel = 0.0
    self.long_stat = ""
    self.long_plan_source = ""

    self.candidate = candidate
    self.long_log = Params().get_bool("LongLogDisplay")
    self.stopping_dist = float(Decimal(Params().get("StoppingDist", encoding="utf8"))*Decimal('0.1'))

    self.vRel_prev = 0
    self.decel_damping = 1.0
    self.decel_damping2 = 1.0
    self.damping_timer3 = 1.0
    self.damping_timer = 0
    self.loc_timer = 0


  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid

  def update(self, active, CS, CP, long_plan, accel_limits, t_since_plan, radarState):
    self.loc_timer += 1
    if self.loc_timer > 100:
      self.loc_timer = 0
      self.long_log = Params().get_bool("LongLogDisplay")
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # Interp control trajectory
    speeds = long_plan.speeds
    if len(speeds) == CONTROL_N:
      v_target = interp(t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target = interp(t_since_plan, T_IDXS[:CONTROL_N], long_plan.accels)

      v_target_lower = interp(CP.longitudinalActuatorDelayLowerBound + t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_lower = 2 * (v_target_lower - v_target) / CP.longitudinalActuatorDelayLowerBound - a_target

      v_target_upper = interp(CP.longitudinalActuatorDelayUpperBound + t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_upper = 2 * (v_target_upper - v_target) / CP.longitudinalActuatorDelayUpperBound - a_target
      a_target = min(a_target_lower, a_target_upper)

      v_target_future = speeds[-1]
    else:
      v_target = 0.0
      v_target_future = 0.0
      a_target = 0.0

    # TODO: This check is not complete and needs to be enforced by MPC
    a_target = clip(a_target, ACCEL_MIN_ISO, ACCEL_MAX_ISO)

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    # Update state machine
    output_accel = self.last_output_accel

    if radarState is None:
      dRel = 150
      vRel = 0
    else:
      dRel = radarState.leadOne.dRel
      vRel = radarState.leadOne.vRel
    if long_plan.hasLead:
      if 1 < CS.radarDistance <= 149:
        stop = True if (dRel <= self.stopping_dist and radarState.leadOne.status) else False
      else:
        stop = True if (dRel < 6.0 and radarState.leadOne.status) else False
    else:
      stop = False
    self.long_control_state = long_control_state_trans(CP, active, self.long_control_state, CS.vEgo,
                                                       v_target, v_target_future, CS.brakePressed,
                                                       CS.cruiseState.standstill, stop, CS.gasPressed)

    if (self.long_control_state == LongCtrlState.off or (CS.brakePressed or CS.gasPressed)) and self.candidate != CAR.NIRO_EV_DE:
      self.pid.reset()
      output_accel = 0.
    elif self.long_control_state == LongCtrlState.off or CS.gasPressed:
      self.reset(CS.vEgo)
      output_accel = 0.

    # tracking objects and driving
    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target

      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      prevent_overshoot = not CP.stoppingControl and CS.vEgo < 1.5 and v_target_future < 0.7 and v_target_future < self.v_pid
      deadzone = interp(CS.vEgo, CP.longitudinalTuning.deadzoneBP, CP.longitudinalTuning.deadzoneV)
      freeze_integrator = prevent_overshoot

      # opkr
      # if self.vRel_prev != vRel and vRel <= 0 and CS.vEgo > 13. and self.damping_timer <= 0: # decel mitigation for a while
      #   if (vRel - self.vRel_prev)*3.6 <= -5:
      #     self.damping_timer = 2.5*CS.vEgo
      #     self.damping_timer3 = self.damping_timer
      #     self.decel_damping2 = interp(abs((vRel - self.vRel_prev)*3.6), [0., 5.], [1., 0.])
      #   self.vRel_prev = vRel
      # elif self.damping_timer > 0:
      #   self.damping_timer -= 1
      #   self.decel_damping = interp(self.damping_timer, [0., self.damping_timer3], [1., self.decel_damping2])

      output_accel = self.pid.update(self.v_pid, CS.vEgo, speed=CS.vEgo, deadzone=deadzone, feedforward=a_target, freeze_integrator=freeze_integrator)
      # output_accel *= self.decel_damping

      if prevent_overshoot or CS.brakeHold:
        output_accel = min(output_accel, 0.0)

    # Intention is to stop, switch to a different brake control until we stop
    elif self.long_control_state == LongCtrlState.stopping:
      # Keep applying brakes until the car is stopped
      if not CS.standstill or output_accel > CP.stopAccel:
        output_accel -= CP.stoppingDecelRate * DT_CTRL
      elif CS.standstill and CS.cruiseState.standstill and output_accel <= -0.7:
        output_accel = -0.7
      elif CS.standstill and output_accel < -0.7: # loosen brake at standstill, to mitigate load of brake
        output_accel += CP.stoppingDecelRate * DT_CTRL
      output_accel = clip(output_accel, accel_limits[0], accel_limits[1])
      self.reset(CS.vEgo)

    self.last_output_accel = output_accel
    final_accel = clip(output_accel, accel_limits[0], accel_limits[1])

    if self.long_control_state == LongCtrlState.stopping:
      self.long_stat = "STP"
    elif self.long_control_state == LongCtrlState.pid:
      self.long_stat = "PID"
    elif self.long_control_state == LongCtrlState.off:
      self.long_stat = "OFF"
    else:
      self.long_stat = "---"

    if long_plan.longitudinalPlanSource == LongitudinalPlanSource.lead0:
      self.long_plan_source = "lead0"
    elif long_plan.longitudinalPlanSource == LongitudinalPlanSource.lead1:
      self.long_plan_source = "lead1"
    elif long_plan.longitudinalPlanSource == LongitudinalPlanSource.cruise:
      self.long_plan_source = "cruise"
    elif long_plan.longitudinalPlanSource == LongitudinalPlanSource.stop:
      self.long_plan_source = "stop"
    else:
      self.long_plan_source = "---"

    if CP.sccBus != 0 and self.long_log:
      str_log3 = 'LS={:s}  LP={:s}  AQ/AR/AT/FA={:+04.2f}/{:+04.2f}/{:+04.2f}/{:+04.2f}  GB={}  ED/RD={:04.1f}/{:04.1f}  TG={:03.0f}/{:03.0f}'.format(self.long_stat, \
       self.long_plan_source, CP.aqValue, CP.aqValueRaw, a_target, final_accel, int(CS.gasPressed or CS.brakePressed), dRel, CS.radarDistance, \
       (v_target*CV.MS_TO_MPH+10.0) if CS.isMph else (v_target*CV.MS_TO_KPH), (v_target_future*CV.MS_TO_MPH+10.0) if CS.isMph else (v_target_future*CV.MS_TO_KPH))
      trace1.printf2('{}'.format(str_log3))

    return final_accel, a_target
