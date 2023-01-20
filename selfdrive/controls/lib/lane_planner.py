import numpy as np
from cereal import log, messaging
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp
from common.realtime import DT_MDL
from selfdrive.hardware import EON, TICI
from selfdrive.swaglog import cloudlog
from common.params import Params
from decimal import Decimal

TRAJECTORY_SIZE = 33
# camera offset is meters from center car to camera
# model path is in the frame of the camera. Empirically 
# the model knows the difference between TICI and EON
# so a path offset is not needed
PATH_OFFSET = -(float(Decimal(Params().get("PathOffsetAdj", encoding="utf8")) * Decimal('0.001')))  # default 0.0
if EON:
  CAMERA_OFFSET = -(float(Decimal(Params().get("CameraOffsetAdj", encoding="utf8")) * Decimal('0.001')))  # m from center car to camera
  CAMERA_OFFSET_A = CAMERA_OFFSET + 0.15
elif TICI:
  CAMERA_OFFSET = 0.04
  CAMERA_OFFSET_A = CAMERA_OFFSET + 0.15
else:
  CAMERA_OFFSET = 0.0
  CAMERA_OFFSET_A = CAMERA_OFFSET + 0.15


class LanePlanner:
  def __init__(self, wide_camera=False):
    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))

    self.params = Params()
    self.lane_width_estimate = FirstOrderFilter(float(Decimal(self.params.get("LaneWidth", encoding="utf8")) * Decimal('0.1')), 9.95, DT_MDL)
    self.lane_width_certainty = FirstOrderFilter(1.0, 0.95, DT_MDL)
    self.lane_width = float(Decimal(self.params.get("LaneWidth", encoding="utf8")) * Decimal('0.1'))
    self.spd_lane_width_spd = list(map(float, self.params.get("SpdLaneWidthSpd", encoding="utf8").split(',')))
    self.spd_lane_width_set = list(map(float, self.params.get("SpdLaneWidthSet", encoding="utf8").split(',')))

    self.lll_prob = 0.
    self.rll_prob = 0.
    self.d_prob = 0.

    self.lll_std = 0.
    self.rll_std = 0.

    self.l_lane_change_prob = 0.
    self.r_lane_change_prob = 0.

    self.camera_offset = -CAMERA_OFFSET if wide_camera else CAMERA_OFFSET
    self.path_offset = -PATH_OFFSET if wide_camera else PATH_OFFSET

    self.left_curv_offset = int(self.params.get("LeftCurvOffsetAdj", encoding="utf8"))
    self.right_curv_offset = int(self.params.get("RightCurvOffsetAdj", encoding="utf8"))

    self.drive_routine_on_co = self.params.get_bool("RoutineDriveOn")
    if self.drive_routine_on_co:
      option_list = list(self.params.get("RoutineDriveOption", encoding="utf8"))
      if '0' in option_list:
        self.drive_routine_on_co = True
      else:
        self.drive_routine_on_co = False

    self.drive_close_to_edge = self.params.get_bool("CloseToRoadEdge")
    self.left_edge_offset = float(Decimal(self.params.get("LeftEdgeOffset", encoding="utf8")) * Decimal('0.01'))
    self.right_edge_offset = float(Decimal(self.params.get("RightEdgeOffset", encoding="utf8")) * Decimal('0.01'))

    self.speed_offset = self.params.get_bool("SpeedCameraOffset")

    self.road_edge_offset = 0.0

    self.lp_timer = 0
    self.lp_timer2 = 0
    self.lp_timer3 = 0
    
    self.sm = messaging.SubMaster(['liveMapData'])

    self.total_camera_offset = self.camera_offset

  def parse_model(self, md, sm, v_ego):
    curvature = sm['controlsState'].curvature
    mode_select = sm['carState'].cruiseState.modeSel
    if self.drive_routine_on_co:
      self.sm.update(0)
      current_road_offset = -self.sm['liveMapData'].roadCameraOffset
    else:
      current_road_offset = 0.0

    Curv = round(curvature, 4)
    # right lane is minus
    lane_differ = round(abs(self.lll_y[0] + self.rll_y[0]), 2)
    lean_offset = 0
    if int(mode_select) == 4:
      lean_offset = 0.15
    else:
      lean_offset = 0

    if (self.left_curv_offset != 0 or self.right_curv_offset != 0) and v_ego > 8:
      if curvature > 0.0008 and self.left_curv_offset < 0 and lane_differ >= 0: # left curve
        if lane_differ > 0.6:
          lane_differ = 0.6          
        lean_offset = -round(abs(self.left_curv_offset) * lane_differ * 0.05, 3) # move to left
      elif curvature > 0.0008 and self.left_curv_offset > 0 and lane_differ <= 0:
        if lane_differ > 0.6:
          lane_differ = 0.6
        lean_offset = +round(abs(self.left_curv_offset) * lane_differ * 0.05, 3) # move to right
      elif curvature < -0.0008 and self.right_curv_offset < 0 and lane_differ >= 0: # right curve
        if lane_differ > 0.6:
          lane_differ = 0.6    
        lean_offset = -round(abs(self.right_curv_offset) * lane_differ * 0.05, 3) # move to left
      elif curvature < -0.0008 and self.right_curv_offset > 0 and lane_differ <= 0:
        if lane_differ > 0.6:
          lane_differ = 0.6    
        lean_offset = +round(abs(self.right_curv_offset) * lane_differ * 0.05, 3) # move to right
      else:
        lean_offset = 0

    self.lp_timer += DT_MDL
    if self.lp_timer > 1.0:
      self.lp_timer = 0.0
      self.speed_offset = self.params.get_bool("SpeedCameraOffset")
      if self.params.get_bool("OpkrLiveTunePanelEnable"):
        self.camera_offset = -(float(Decimal(self.params.get("CameraOffsetAdj", encoding="utf8")) * Decimal('0.001')))

    if self.drive_close_to_edge: # opkr
      left_edge_prob = np.clip(1.0 - md.roadEdgeStds[0], 0.0, 1.0)
      left_nearside_prob = md.laneLineProbs[0]
      left_close_prob = md.laneLineProbs[1]
      right_close_prob = md.laneLineProbs[2]
      right_nearside_prob = md.laneLineProbs[3]
      right_edge_prob = np.clip(1.0 - md.roadEdgeStds[1], 0.0, 1.0)

      self.lp_timer3 += DT_MDL
      if self.lp_timer3 > 3.0:
        self.lp_timer3 = 0.0
        if right_nearside_prob < 0.1 and left_nearside_prob < 0.1:
          self.road_edge_offset = 0.0
        elif right_edge_prob > 0.35 and right_nearside_prob < 0.2 and right_close_prob > 0.5 and left_nearside_prob >= right_nearside_prob:
          self.road_edge_offset = -self.right_edge_offset
        elif left_edge_prob > 0.35 and left_nearside_prob < 0.2 and left_close_prob > 0.5 and right_nearside_prob >= left_nearside_prob:
          self.road_edge_offset = -self.left_edge_offset
        else:
          self.road_edge_offset = 0.0
    else:
      self.road_edge_offset = 0.0
    if self.speed_offset:
      speed_offset = -interp(v_ego, [0, 11.1, 16.6, 22.2, 31], [0.10, 0.05, 0.02, 0.01, 0.0])
    else:
      speed_offset = 0.0
    self.total_camera_offset = self.camera_offset + lean_offset + current_road_offset + self.road_edge_offset + speed_offset

    lane_lines = md.laneLines
    if len(lane_lines) == 4 and len(lane_lines[0].t) == TRAJECTORY_SIZE:
      self.ll_t = (np.array(lane_lines[1].t) + np.array(lane_lines[2].t))/2
      # left and right ll x is the same
      self.ll_x = lane_lines[1].x
      self.lll_y = np.array(lane_lines[1].y) + self.total_camera_offset
      self.rll_y = np.array(lane_lines[2].y) + self.total_camera_offset
      self.lll_prob = md.laneLineProbs[1]
      self.rll_prob = md.laneLineProbs[2]
      self.lll_std = md.laneLineStds[1]
      self.rll_std = md.laneLineStds[2]

    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]

  def get_d_path(self, v_ego, path_t, path_xyz):
    self.lp_timer2 += DT_MDL
    if self.lp_timer2 > 1.0:
      self.lp_timer2 = 0.0
      if self.params.get_bool("OpkrLiveTunePanelEnable"):
        self.path_offset = -(float(Decimal(self.params.get("PathOffsetAdj", encoding="utf8")) * Decimal('0.001')))
    # Reduce reliance on lanelines that are too far apart or
    # will be in a few seconds
    path_xyz[:, 1] += self.path_offset
    l_prob, r_prob = self.lll_prob, self.rll_prob
    width_pts = self.rll_y - self.lll_y
    prob_mods = []
    for t_check in (0.0, 1.5, 3.0):
      width_at_t = interp(t_check * (v_ego + 7), self.ll_x, width_pts)
      prob_mods.append(interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
    mod = min(prob_mods)
    l_prob *= mod
    r_prob *= mod

    # Reduce reliance on uncertain lanelines
    l_std_mod = interp(self.lll_std, [.15, .3], [1.0, 0.0])
    r_std_mod = interp(self.rll_std, [.15, .3], [1.0, 0.0])
    l_prob *= l_std_mod
    r_prob *= r_std_mod

    # Find current lanewidth
    self.lane_width_certainty.update(l_prob * r_prob)
    current_lane_width = abs(self.rll_y[0] - self.lll_y[0])
    self.lane_width_estimate.update(current_lane_width)
    speed_lane_width = interp(v_ego, self.spd_lane_width_spd, self.spd_lane_width_set)
    self.lane_width = self.lane_width_certainty.x * self.lane_width_estimate.x + \
                      (1 - self.lane_width_certainty.x) * speed_lane_width

    clipped_lane_width = min(4.0, self.lane_width)
    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

    self.d_prob = l_prob + r_prob - l_prob * r_prob
    lane_path_y = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)
    safe_idxs = np.isfinite(self.ll_t)
    if safe_idxs[0]:
      lane_path_y_interp = np.interp(path_t, self.ll_t[safe_idxs], lane_path_y[safe_idxs])
      path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]
    else:
      cloudlog.warning("Lateral mpc - NaNs in laneline times, ignoring")
    return path_xyz
