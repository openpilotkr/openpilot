#include "selfdrive/ui/ui.h"

#include <unistd.h>

#include <cassert>
#include <cmath>
#include <cstdio>

#include "selfdrive/common/util.h"
#include "selfdrive/common/watchdog.h"
#include "selfdrive/hardware/hw.h"

#define BACKLIGHT_DT 0.05
#define BACKLIGHT_TS 10.00
#define BACKLIGHT_OFFROAD 50


// Projects a point in car to space to the corresponding point in full frame
// image space.
static bool calib_frame_to_full_frame(const UIState *s, float in_x, float in_y, float in_z, vertex_data *out) {
  const float margin = 500.0f;
  const QRectF clip_region{-margin, -margin, s->fb_w + 2 * margin, s->fb_h + 2 * margin};

  const vec3 pt = (vec3){{in_x, in_y, in_z}};
  const vec3 Ep = matvecmul3(s->scene.view_from_calib, pt);
  const vec3 KEp = matvecmul3(s->wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix, Ep);

  // Project.
  QPointF point = s->car_space_transform.map(QPointF{KEp.v[0] / KEp.v[2], KEp.v[1] / KEp.v[2]});
  if (clip_region.contains(point)) {
    out->x = point.x();
    out->y = point.y();
    return true;
  }
  return false;
}

static int get_path_length_idx(const cereal::ModelDataV2::XYZTData::Reader &line, const float path_height) {
  const auto line_x = line.getX();
  int max_idx = 0;
  for (int i = 0; i < TRAJECTORY_SIZE && line_x[i] < path_height; ++i) {
    max_idx = i;
  }
  return max_idx;
}

static void update_leads(UIState *s, const cereal::RadarState::Reader &radar_state, std::optional<cereal::ModelDataV2::XYZTData::Reader> line) {
  for (int i = 0; i < 2; ++i) {
    auto lead_data = (i == 0) ? radar_state.getLeadOne() : radar_state.getLeadTwo();
    if (lead_data.getStatus()) {
      float z = line ? (*line).getZ()[get_path_length_idx(*line, lead_data.getDRel())] : 0.0;
      calib_frame_to_full_frame(s, lead_data.getDRel(), -lead_data.getYRel(), z + 1.22, &s->scene.lead_vertices[i]);
    }
  }
}

static void update_line_data(const UIState *s, const cereal::ModelDataV2::XYZTData::Reader &line,
                             float y_off, float z_off, line_vertices_data *pvd, int max_idx) {
  const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
  vertex_data *v = &pvd->v[0];
  for (int i = 0; i <= max_idx; i++) {
    v += calib_frame_to_full_frame(s, line_x[i], line_y[i] - y_off, line_z[i] + z_off, v);
  }
  for (int i = max_idx; i >= 0; i--) {
    v += calib_frame_to_full_frame(s, line_x[i], line_y[i] + y_off, line_z[i] + z_off, v);
  }
  pvd->cnt = v - pvd->v;
  assert(pvd->cnt <= std::size(pvd->v));
}


static void update_blindspot_data(const UIState *s, int lr, const cereal::ModelDataV2::XYZTData::Reader &line,
                             float y_off,  float z_off, line_vertices_data *pvd, int max_idx ) {

  float  y_off1, y_off2;

  if( lr == 0 )
  {
    y_off1 = y_off;
    y_off2 = -0.01;
  }
  else
  {
      y_off1 = 0.01;
      y_off2 = y_off;  
  }
     

  const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
  vertex_data *v = &pvd->v[0];
  for (int i = 0; i <= max_idx; i++) {
    v += calib_frame_to_full_frame(s, line_x[i], line_y[i] - y_off1, line_z[i] + z_off, v);
  }
  for (int i = max_idx; i >= 0; i--) {
    v += calib_frame_to_full_frame(s, line_x[i], line_y[i] + y_off2, line_z[i] + z_off, v);
  }
  pvd->cnt = v - pvd->v;
  assert(pvd->cnt <= std::size(pvd->v));
}


static void update_stop_line_data(const UIState *s, const cereal::ModelDataV2::StopLineData::Reader &line,
                                  float x_off, float y_off, float z_off, line_vertices_data *pvd) {
  const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
  vertex_data *v = &pvd->v[0];
  v += calib_frame_to_full_frame(s, line_x + x_off, line_y - y_off, line_z + z_off, v);
  v += calib_frame_to_full_frame(s, line_x + x_off, line_y + y_off, line_z + z_off, v);
  v += calib_frame_to_full_frame(s, line_x - x_off, line_y + y_off, line_z + z_off, v);
  v += calib_frame_to_full_frame(s, line_x - x_off, line_y - y_off, line_z + z_off, v);
  pvd->cnt = v - pvd->v;
  assert(pvd->cnt <= std::size(pvd->v));
}

static void update_model(UIState *s, const cereal::ModelDataV2::Reader &model) {
  UIScene &scene = s->scene;
  auto model_position = model.getPosition();
  float max_distance = std::clamp(model_position.getX()[TRAJECTORY_SIZE - 1],
                                  MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);

  // update lane lines
  const auto lane_lines = model.getLaneLines();
  const auto lane_line_probs = model.getLaneLineProbs();
  int max_idx = get_path_length_idx(lane_lines[0], max_distance);
  for (int i = 0; i < std::size(scene.lane_line_vertices); i++) {
    scene.lane_line_probs[i] = lane_line_probs[i];
    update_line_data(s, lane_lines[i], 0.025 * scene.lane_line_probs[i], 0, &scene.lane_line_vertices[i], max_idx);
  }

  // update road edges
  const auto road_edges = model.getRoadEdges();
  const auto road_edge_stds = model.getRoadEdgeStds();
  for (int i = 0; i < std::size(scene.road_edge_vertices); i++) {
    scene.road_edge_stds[i] = road_edge_stds[i];
    update_line_data(s, road_edges[i], 0.025, 0, &scene.road_edge_vertices[i], max_idx);
  }

  // update path
  auto lead_one = (*s->sm)["radarState"].getRadarState().getLeadOne();
  if (lead_one.getStatus()) {
    const float lead_d = lead_one.getDRel() * 2.;
    max_distance = std::clamp((float)(lead_d - fmin(lead_d * 0.35, 10.)), 0.0f, max_distance);
  }
  max_idx = get_path_length_idx(model_position, max_distance);
  update_line_data(s, model_position, 0.9, 1.22, &scene.track_vertices, max_idx);

   // update blindspot line
  for (int i = 0; i < std::size(scene.lane_blindspot_vertices); i++) {
    scene.lane_blindspot_probs[i] = lane_line_probs[i];
    update_blindspot_data(s, i, lane_lines[i+1], 2.8, 0, &scene.lane_blindspot_vertices[i], max_idx);
  }   


  // update stop lines
  if (scene.stop_line) {
    const auto stop_line = model.getStopLine();
    if (stop_line.getProb() > .5) {
      update_stop_line_data(s, stop_line, .5, 2, 1.22, &scene.stop_line_vertices);
    }
  }
}

static void update_sockets(UIState *s) {
  s->sm->update(0);
}

static void update_state(UIState *s) {
  SubMaster &sm = *(s->sm);
  UIScene &scene = s->scene;
  s->running_time = 1e-9 * (nanos_since_boot() - sm["deviceState"].getDeviceState().getStartedMonoTime());

  // update engageability and DM icons at 2Hz
  if (sm.frame % (UI_FREQ / 2) == 0) {
    auto cs = sm["controlsState"].getControlsState();
    scene.engageable = cs.getEngageable() || cs.getEnabled();
    scene.dm_active = sm["driverMonitoringState"].getDriverMonitoringState().getIsActiveMode();
  }

  if (sm.updated("controlsState")) {
    scene.controls_state = sm["controlsState"].getControlsState();
    scene.lateralControlMethod = scene.controls_state.getLateralControlMethod();
    if (scene.lateralControlMethod == 0) {
      scene.output_scale = scene.controls_state.getLateralControlState().getPidState().getOutput();
    } else if (scene.lateralControlMethod == 1) {
      scene.output_scale = scene.controls_state.getLateralControlState().getIndiState().getOutput();
    } else if (scene.lateralControlMethod == 2) {
      scene.output_scale = scene.controls_state.getLateralControlState().getLqrState().getOutput();
    } else if (scene.lateralControlMethod == 3) {
      scene.output_scale = scene.controls_state.getLateralControlState().getTorqueState().getOutput();
    } else if (scene.lateralControlMethod == 4) {
      scene.output_scale = scene.controls_state.getLateralControlState().getAtomState().getOutput();
      scene.multi_lat_selected = scene.controls_state.getLateralControlState().getAtomState().getSelected();
    }

    scene.alertTextMsg1 = scene.controls_state.getAlertTextMsg1(); //debug1
    scene.alertTextMsg2 = scene.controls_state.getAlertTextMsg2(); //debug2
    scene.alertTextMsg3 = scene.controls_state.getAlertTextMsg3(); //debug3

    scene.limitSpeedCamera = scene.controls_state.getLimitSpeedCamera();
    scene.limitSpeedCameraDist = scene.controls_state.getLimitSpeedCameraDist();
    scene.mapSign = scene.controls_state.getMapSign();
    scene.mapSignCam = scene.controls_state.getMapSignCam();
    scene.steerRatio = scene.controls_state.getSteerRatio();
    scene.dynamic_tr_mode = scene.controls_state.getDynamicTRMode();
    scene.dynamic_tr_value = scene.controls_state.getDynamicTRValue();
    scene.osm_off_spdlimit = scene.controls_state.getOsmOffSpdLimit();
    scene.accel = scene.controls_state.getAccel();
    scene.ctrl_speed = scene.controls_state.getSafetySpeed();
    scene.desired_angle_steers = scene.controls_state.getSteeringAngleDesiredDeg();
    scene.gap_by_speed_on = scene.controls_state.getGapBySpeedOn();
  }
  if (sm.updated("carState")) {
    scene.car_state = sm["carState"].getCarState();
    auto cs_data = sm["carState"].getCarState();
    auto cruiseState = scene.car_state.getCruiseState();
    scene.awake = cruiseState.getCruiseSwState();

    if (scene.leftBlinker!=cs_data.getLeftBlinker() || scene.rightBlinker!=cs_data.getRightBlinker()) {
      scene.blinker_blinkingrate = 120;
    }
    scene.brakePress = cs_data.getBrakePressed();
    scene.gasPress = cs_data.getGasPressed();
    scene.brakeLights = cs_data.getBrakeLights();
    scene.getGearShifter = cs_data.getGearShifter();
    scene.leftBlinker = cs_data.getLeftBlinker();
    scene.rightBlinker = cs_data.getRightBlinker();
    scene.leftblindspot = cs_data.getLeftBlindspot();
    scene.rightblindspot = cs_data.getRightBlindspot();
    scene.tpmsUnit = cs_data.getTpms().getUnit();
    scene.tpmsPressureFl = cs_data.getTpms().getFl();
    scene.tpmsPressureFr = cs_data.getTpms().getFr();
    scene.tpmsPressureRl = cs_data.getTpms().getRl();
    scene.tpmsPressureRr = cs_data.getTpms().getRr();
    scene.radarDistance = cs_data.getRadarDistance();
    scene.standStill = cs_data.getStandStill();
    scene.vSetDis = cs_data.getVSetDis();
    scene.cruiseAccStatus = cs_data.getCruiseAccStatus();
    scene.angleSteers = cs_data.getSteeringAngleDeg();
    scene.cruise_gap = cs_data.getCruiseGapSet();
    scene.brakeHold = cs_data.getBrakeHold();
    scene.steer_warning = cs_data.getSteerFaultTemporary();
    scene.stand_still = cs_data.getStandstill();
    scene.a_req_value = cs_data.getAReqValue();
    scene.engine_rpm = cs_data.getEngineRpm();
    scene.gear_step = cs_data.getGearStep();
    scene.charge_meter = cs_data.getChargeMeter();
  }

  if (sm.updated("liveParameters")) {
    //scene.liveParams = sm["liveParameters"].getLiveParameters();
    auto live_data = sm["liveParameters"].getLiveParameters();
    scene.liveParams.angleOffset = live_data.getAngleOffsetDeg();
    scene.liveParams.angleOffsetAverage = live_data.getAngleOffsetAverageDeg();
    scene.liveParams.stiffnessFactor = live_data.getStiffnessFactor();
    scene.liveParams.steerRatio = live_data.getSteerRatio();
  }
  if (sm.updated("modelV2") && s->vg) {
    update_model(s, sm["modelV2"].getModelV2());
  }
  if (sm.updated("radarState") && s->vg) {
    std::optional<cereal::ModelDataV2::XYZTData::Reader> line;
    if (sm.rcv_frame("modelV2") > 0) {
      line = sm["modelV2"].getModelV2().getPosition();
    }
    update_leads(s, sm["radarState"].getRadarState(), line);
  }
  if (sm.updated("liveCalibration")) {
    scene.world_objects_visible = true;
    auto rpy_list = sm["liveCalibration"].getLiveCalibration().getRpyCalib();
    Eigen::Vector3d rpy;
    rpy << rpy_list[0], rpy_list[1], rpy_list[2];
    Eigen::Matrix3d device_from_calib = euler2rot(rpy);
    Eigen::Matrix3d view_from_device;
    view_from_device << 0,1,0,
                        0,0,1,
                        1,0,0;
    Eigen::Matrix3d view_from_calib = view_from_device * device_from_calib;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        scene.view_from_calib.v[i*3 + j] = view_from_calib(i,j);
      }
    }
  }
  if (sm.updated("deviceState")) {
    scene.deviceState = sm["deviceState"].getDeviceState();
    scene.cpuPerc = (scene.deviceState.getCpuUsagePercent()[0] + scene.deviceState.getCpuUsagePercent()[1] + scene.deviceState.getCpuUsagePercent()[2] + scene.deviceState.getCpuUsagePercent()[3])/4;
    scene.cpuTemp = (scene.deviceState.getCpuTempC()[0] + scene.deviceState.getCpuTempC()[1] + scene.deviceState.getCpuTempC()[2] + scene.deviceState.getCpuTempC()[3])/4;
    scene.batTemp = scene.deviceState.getBatteryTempC();
    scene.ambientTemp = scene.deviceState.getAmbientTempC();
    scene.fanSpeed = scene.deviceState.getFanSpeedPercentDesired();
    scene.batPercent = scene.deviceState.getBatteryPercent();
  }
  if (sm.updated("pandaStates")) {
    auto pandaStates = sm["pandaStates"].getPandaStates();
    if (pandaStates.size() > 0) {
      scene.pandaType = pandaStates[0].getPandaType();

      if (scene.pandaType != cereal::PandaState::PandaType::UNKNOWN) {
        scene.ignition = false;
        for (const auto& pandaState : pandaStates) {
          scene.ignition |= pandaState.getIgnitionLine() || pandaState.getIgnitionCan();
          scene.controlAllowed = pandaState.getControlsAllowed();
        }
      }
    }
  } else if ((s->sm->frame - s->sm->rcv_frame("pandaStates")) > 5*UI_FREQ) {
    scene.pandaType = cereal::PandaState::PandaType::UNKNOWN;
  }
  if (sm.updated("ubloxGnss")) {
    auto ub_data = sm["ubloxGnss"].getUbloxGnss();
    if (ub_data.which() == cereal::UbloxGnss::MEASUREMENT_REPORT) {
      scene.satelliteCount = ub_data.getMeasurementReport().getNumMeas();
    }
  }
  if (sm.updated("gpsLocationExternal")) {
    scene.gpsAccuracy = sm["gpsLocationExternal"].getGpsLocationExternal().getAccuracy();
    auto ge_data = sm["gpsLocationExternal"].getGpsLocationExternal();
    scene.gpsAccuracyUblox = ge_data.getAccuracy();
    scene.altitudeUblox = ge_data.getAltitude();
    scene.bearingUblox = ge_data.getBearingDeg();
  }
  if (sm.updated("carParams")) {
    auto cp_data = sm["carParams"].getCarParams();
    scene.longitudinal_control = cp_data.getOpenpilotLongitudinalControl();
    scene.steer_actuator_delay = cp_data.getSteerActuatorDelay();
  }
  if (sm.updated("lateralPlan")) {
    scene.lateral_plan = sm["lateralPlan"].getLateralPlan();
    auto lp_data = sm["lateralPlan"].getLateralPlan();
    scene.lateralPlan.laneWidth = lp_data.getLaneWidth();
    scene.lateralPlan.dProb = lp_data.getDProb();
    scene.lateralPlan.lProb = lp_data.getLProb();
    scene.lateralPlan.rProb = lp_data.getRProb();
    scene.lateralPlan.standstillElapsedTime = lp_data.getStandstillElapsedTime();
    scene.lateralPlan.lanelessModeStatus = lp_data.getLanelessMode();
    scene.lateralPlan.totalCameraOffset = lp_data.getTotalCameraOffset();
  }
  if (sm.updated("longitudinalPlan")) {
    scene.longitudinal_plan = sm["longitudinalPlan"].getLongitudinalPlan();
    auto lop_data = sm["longitudinalPlan"].getLongitudinalPlan();
    for (int i = 0; i < std::size(scene.longitudinalPlan.e2ex); i++) {
      scene.longitudinalPlan.e2ex[i] = lop_data.getE2eX()[i];
    }
    for (int i = 0; i < std::size(scene.longitudinalPlan.lead0); i++) {
      scene.longitudinalPlan.lead0[i] = lop_data.getLead0Obstacle()[i];
    }
    for (int i = 0; i < std::size(scene.longitudinalPlan.lead1); i++) {
      scene.longitudinalPlan.lead1[i] = lop_data.getLead1Obstacle()[i];
    }
    for (int i = 0; i < std::size(scene.longitudinalPlan.cruisetg); i++) {
      scene.longitudinalPlan.cruisetg[i] = lop_data.getCruiseTarget()[i];
    }
    for (int i = 0; i < std::size(scene.longitudinalPlan.stopline); i++) {
      scene.longitudinalPlan.stopline[i] = lop_data.getStopLine()[i];
    }
    scene.longitudinalPlan.stopprob = lop_data.getStoplineProb();
  }
  // opkr
  if (sm.updated("liveNaviData")) {
    scene.live_navi_data = sm["liveNaviData"].getLiveNaviData();
    auto lm_data = sm["liveNaviData"].getLiveNaviData();
    scene.liveNaviData.opkrspeedlimit = lm_data.getSpeedLimit();
    scene.liveNaviData.opkrspeedlimitdist = lm_data.getSafetyDistance();
    scene.liveNaviData.opkrroadsign = lm_data.getSafetySign();
    scene.liveNaviData.opkrspeedsign = lm_data.getSafetySignCam();
    scene.liveNaviData.opkrcurveangle = lm_data.getRoadCurvature();
    scene.liveNaviData.opkrturninfo = lm_data.getTurnInfo();
    scene.liveNaviData.opkrdisttoturn = lm_data.getDistanceToTurn();
  }
  if (sm.updated("liveENaviData")) {
    scene.live_enavi_data = sm["liveENaviData"].getLiveENaviData();
    auto lme_data = sm["liveENaviData"].getLiveENaviData();
    scene.liveENaviData.eopkrspeedlimit = lme_data.getSpeedLimit();
    scene.liveENaviData.eopkrsafetydist = lme_data.getSafetyDistance();
    scene.liveENaviData.eopkrsafetysign = lme_data.getSafetySign();
    scene.liveENaviData.eopkrturninfo = lme_data.getTurnInfo();
    scene.liveENaviData.eopkrdisttoturn = lme_data.getDistanceToTurn();
    scene.liveENaviData.eopkrconalive = lme_data.getConnectionAlive();
    scene.liveENaviData.eopkrroadlimitspeed = lme_data.getRoadLimitSpeed();
    scene.liveENaviData.eopkrlinklength = lme_data.getLinkLength();
    scene.liveENaviData.eopkrcurrentlinkangle = lme_data.getCurrentLinkAngle();
    scene.liveENaviData.eopkrnextlinkangle = lme_data.getNextLinkAngle();
    scene.liveENaviData.eopkrposroadname = lme_data.getPosRoadName();
    scene.liveENaviData.eopkrishighway = lme_data.getIsHighway();
    scene.liveENaviData.eopkristunnel = lme_data.getIsTunnel();
  }
  if (sm.updated("liveMapData")) {
    scene.live_map_data = sm["liveMapData"].getLiveMapData();
    auto lmap_data = sm["liveMapData"].getLiveMapData();
    scene.liveMapData.ospeedLimit = lmap_data.getSpeedLimit();
    scene.liveMapData.ospeedLimitAhead = lmap_data.getSpeedLimitAhead();
    scene.liveMapData.ospeedLimitAheadDistance = lmap_data.getSpeedLimitAheadDistance();
    scene.liveMapData.oturnSpeedLimit = lmap_data.getTurnSpeedLimit();
    scene.liveMapData.oturnSpeedLimitEndDistance = lmap_data.getTurnSpeedLimitEndDistance();
    scene.liveMapData.oturnSpeedLimitSign = lmap_data.getTurnSpeedLimitSign();
    scene.liveMapData.ocurrentRoadName = lmap_data.getCurrentRoadName();
    scene.liveMapData.oref = lmap_data.getRef();
  }
  if ((!scene.started || s->is_OpenpilotViewEnabled || scene.cal_view) && sm.updated("sensorEvents")) {
    for (auto sensor : sm["sensorEvents"].getSensorEvents()) {
      if (sensor.which() == cereal::SensorEventData::ACCELERATION) {
        auto accel = sensor.getAcceleration().getV();
        if (accel.totalSize().wordCount) { // TODO: sometimes empty lists are received. Figure out why
          scene.accel_sensor = accel[2];
          // opkr
          scene.accel_prob[0] = atan(accel[2]/accel[0]) * (180 / M_PI); // back and forth
          scene.accel_prob[1] = atan(accel[1]/accel[0]) * (180 / M_PI); // right and left
        }
      } else if (sensor.which() == cereal::SensorEventData::GYRO_UNCALIBRATED) {
        auto gyro = sensor.getGyroUncalibrated().getV();
        if (gyro.totalSize().wordCount) {
          scene.gyro_sensor = gyro[1];
        }
      }
    }
  }
  if (sm.updated("roadCameraState")) {
    auto camera_state = sm["roadCameraState"].getRoadCameraState();

    float max_lines = Hardware::EON() ? 5408 : 1904;
    float max_gain = Hardware::EON() ? 1.0: 10.0;
    float max_ev = max_lines * max_gain;

    float ev = camera_state.getGain() * float(camera_state.getIntegLines());

    scene.light_sensor = std::clamp<float>(1.0 - (ev / max_ev), 0.0, 1.0);
  } else if (Hardware::TICI() && sm.updated("wideRoadCameraState")) {
    auto camera_state = sm["wideRoadCameraState"].getWideRoadCameraState();

    float max_lines = 1904;
    float max_gain = 10.0;
    float max_ev = max_lines * max_gain / 6;

    float ev = camera_state.getGain() * float(camera_state.getIntegLines());

    scene.light_sensor = std::clamp<float>(1.0 - (ev / max_ev), 0.0, 1.0);
  }
  if (!s->is_OpenpilotViewEnabled) {
    scene.started = sm["deviceState"].getDeviceState().getStarted() && scene.ignition;
  } else {
    scene.started = sm["deviceState"].getDeviceState().getStarted();
  }
}

void ui_update_params(UIState *s) {
  s->scene.is_metric = Params().getBool("IsMetric");
}

static void update_status(UIState *s) {
  if (s->scene.started && s->sm->updated("controlsState")) {
    auto controls_state = (*s->sm)["controlsState"].getControlsState();
    auto alert_status = controls_state.getAlertStatus();
    if (alert_status == cereal::ControlsState::AlertStatus::USER_PROMPT) {
      s->status = STATUS_WARNING;
    } else if (alert_status == cereal::ControlsState::AlertStatus::CRITICAL) {
      s->status = STATUS_ALERT;
    } else {
      if (s->scene.comma_stock_ui == 2) {
        s->status = controls_state.getEnabled() ? STATUS_DND : STATUS_DISENGAGED;
      } else {
        s->status = controls_state.getEnabled() ? STATUS_ENGAGED : STATUS_DISENGAGED;
      }
    }
  }

  // Handle onroad/offroad transition
  static bool started_prev = false;
  if (s->scene.started != started_prev) {
    if (s->scene.started) {
      s->status = STATUS_DISENGAGED;
      s->scene.started_frame = s->sm->frame;
      s->wide_camera = Hardware::TICI() ? Params().getBool("EnableWideCamera") : false;
    }
    // Invisible until we receive a calibration message.
    s->scene.world_objects_visible = false;
  }
  started_prev = s->scene.started;

  if (s->sm->frame % (5*UI_FREQ) == 0) {
  	s->is_OpenpilotViewEnabled = Params().getBool("IsOpenpilotViewEnabled");
  }

  Params params;

  //opkr navi on boot
  if (!s->scene.navi_on_boot && (s->sm->frame - s->scene.started_frame > 5*UI_FREQ)) {
    if (params.getBool("OpkrRunNaviOnBoot") && params.getBool("ControlsReady") && (params.get("CarParams").size() > 0)) {
      s->scene.navi_on_boot = true;
      s->scene.map_is_running = true;
      s->scene.map_on_top = true;
      s->scene.map_on_overlay = false;
      params.putBool("OpkrMapEnable", true);
      if (s->scene.navi_select == 0) {
        system("am start com.thinkware.inaviair/com.thinkware.inaviair.UIActivity");
      } else if (s->scene.navi_select == 1) {
        system("am start com.mnsoft.mappyobn/com.mnsoft.mappy.MainActivity");
      } else if (s->scene.navi_select == 2) {
        system("am start com.waze/com.waze.MainActivity");
      }

    } else if (s->sm->frame - s->scene.started_frame > 20*UI_FREQ) {
      s->scene.navi_on_boot = true;
    }
  }
  if (!s->scene.move_to_background && (s->sm->frame - s->scene.started_frame > 20*UI_FREQ)) {
    if (params.getBool("OpkrRunNaviOnBoot") && params.getBool("OpkrMapEnable") && params.getBool("ControlsReady") && (params.get("CarParams").size() > 0)) {
      s->scene.move_to_background = true;
      s->scene.map_on_top = false;
      s->scene.map_on_overlay = true;
      system("am start --activity-task-on-home com.opkr.maphack/com.opkr.maphack.MainActivity");
    } else if (s->sm->frame - s->scene.started_frame > 30*UI_FREQ) {
      s->scene.move_to_background = true;
    }
  }
  if (!s->scene.auto_gitpull && (s->sm->frame - s->scene.started_frame > 15*UI_FREQ)) {
    if (params.getBool("GitPullOnBoot")) {
      s->scene.auto_gitpull = true;
      system("/data/openpilot/selfdrive/assets/addon/script/gitpull.sh &");
    } else if (s->sm->frame - s->scene.started_frame > 20*UI_FREQ) {
      s->scene.auto_gitpull = true;
      system("/data/openpilot/selfdrive/assets/addon/script/gitcommit.sh &");
    }
  }

  if (!s->scene.read_params_once) {
    // user param value init
    s->scene.end_to_end = params.getBool("EndToEndToggle");
    s->scene.driving_record = params.getBool("OpkrDrivingRecord");
    s->scene.nDebugUi1 = params.getBool("DebugUi1");
    s->scene.nDebugUi2 = params.getBool("DebugUi2");
    s->scene.nDebugUi3 = params.getBool("DebugUi3");
    s->scene.forceGearD = params.getBool("JustDoGearD");
    s->scene.nOpkrBlindSpotDetect = params.getBool("OpkrBlindSpotDetect");
    s->scene.laneless_mode = std::stoi(params.get("LanelessMode"));
    s->scene.recording_count = std::stoi(params.get("RecordingCount"));
    s->scene.recording_quality = std::stoi(params.get("RecordingQuality"));
    s->scene.monitoring_mode = params.getBool("OpkrMonitoringMode");
    s->scene.brightness = std::stoi(params.get("OpkrUIBrightness"));
    s->scene.nVolumeBoost = std::stoi(params.get("OpkrUIVolumeBoost"));
    s->scene.autoScreenOff = std::stoi(params.get("OpkrAutoScreenOff"));
    s->scene.brightness_off = std::stoi(params.get("OpkrUIBrightnessOff"));
    s->scene.cameraOffset = std::stoi(params.get("CameraOffsetAdj"));
    s->scene.pathOffset = std::stoi(params.get("PathOffsetAdj"));
    s->scene.pidKp = std::stoi(params.get("PidKp"));
    s->scene.pidKi = std::stoi(params.get("PidKi"));
    s->scene.pidKd = std::stoi(params.get("PidKd"));
    s->scene.pidKf = std::stoi(params.get("PidKf"));
    s->scene.torqueKp = std::stoi(params.get("TorqueKp"));
    s->scene.torqueKf = std::stoi(params.get("TorqueKf"));
    s->scene.torqueKi = std::stoi(params.get("TorqueKi"));
    s->scene.torqueFriction = std::stoi(params.get("TorqueFriction"));
    s->scene.torqueMaxLatAccel = std::stoi(params.get("TorqueMaxLatAccel"));
    s->scene.indiInnerLoopGain = std::stoi(params.get("InnerLoopGain"));
    s->scene.indiOuterLoopGain = std::stoi(params.get("OuterLoopGain"));
    s->scene.indiTimeConstant = std::stoi(params.get("TimeConstant"));
    s->scene.indiActuatorEffectiveness = std::stoi(params.get("ActuatorEffectiveness"));
    s->scene.lqrScale = std::stoi(params.get("Scale"));
    s->scene.lqrKi = std::stoi(params.get("LqrKi"));
    s->scene.lqrDcGain = std::stoi(params.get("DcGain"));
    s->scene.navi_select = std::stoi(params.get("OPKRNaviSelect"));
    s->scene.radar_long_helper = std::stoi(params.get("RadarLongHelper"));
    s->scene.live_tune_panel_enable = params.getBool("OpkrLiveTunePanelEnable");
    s->scene.top_text_view = std::stoi(params.get("TopTextView"));
    s->scene.max_animated_rpm = std::stoi(params.get("AnimatedRPMMax"));
    s->scene.show_error = params.getBool("ShowError");
    s->scene.speedlimit_signtype = params.getBool("OpkrSpeedLimitSignType");
    s->scene.sl_decel_off = params.getBool("SpeedLimitDecelOff");
    s->scene.osm_enabled = params.getBool("OSMEnable") || params.getBool("OSMSpeedLimitEnable") || std::stoi(params.get("CurvDecelOption")) == 1 || std::stoi(params.get("CurvDecelOption")) == 3;
    s->scene.animated_rpm = params.getBool("AnimatedRPM");
    s->scene.stop_line = params.getBool("ShowStopLine");
    s->scene.lateralControlMethod = std::stoi(params.get("LateralControlMethod"));
    s->scene.do_not_disturb_mode = std::stoi(params.get("DoNotDisturbMode"));
    s->scene.depart_chime_at_resume = params.getBool("DepartChimeAtResume");

    if (s->scene.autoScreenOff > 0) {
      s->scene.nTime = s->scene.autoScreenOff * 60 * UI_FREQ;
    } else if (s->scene.autoScreenOff == 0) {
      s->scene.nTime = 30 * UI_FREQ;
    } else if (s->scene.autoScreenOff == -1) {
      s->scene.nTime = 15 * UI_FREQ;
    } else if (s->scene.autoScreenOff == -2) {
      s->scene.nTime = 5 * UI_FREQ;
    } else {
      s->scene.nTime = -1;
    }
    s->scene.comma_stock_ui = std::stoi(params.get("CommaStockUI"));
    s->scene.opkr_livetune_ui = params.getBool("OpkrLiveTunePanelEnable");
    s->scene.batt_less = params.getBool("OpkrBattLess");
    s->scene.read_params_once = true;
  }
}


QUIState::QUIState(QObject *parent) : QObject(parent) {
  ui_state.sm = std::make_unique<SubMaster, const std::initializer_list<const char *>>({
    "modelV2", "controlsState", "liveCalibration", "radarState", "deviceState", "roadCameraState",
    "pandaStates", "carParams", "driverMonitoringState", "sensorEvents", "carState", "liveLocationKalman",
    "ubloxGnss", "gpsLocationExternal", "liveParameters", "lateralPlan", "liveNaviData",  "liveENaviData","liveMapData", "longitudinalPlan",
  });

  Params params;
  ui_state.wide_camera = Hardware::TICI() ? params.getBool("EnableWideCamera") : false;
  ui_state.has_prime = params.getBool("HasPrime");
  ui_state.sidebar_view = false;
  ui_state.is_OpenpilotViewEnabled = params.getBool("IsOpenpilotViewEnabled");

  // update timer
  timer = new QTimer(this);
  QObject::connect(timer, &QTimer::timeout, this, &QUIState::update);
  timer->start(1000 / UI_FREQ);
}

void QUIState::update() {
  update_sockets(&ui_state);
  update_state(&ui_state);
  update_status(&ui_state);

  if (ui_state.scene.started != started_prev || ui_state.sm->frame == 1) {
    started_prev = ui_state.scene.started;
    emit offroadTransition(!ui_state.scene.started);
  }

  if (ui_state.sm->frame % UI_FREQ == 0) {
    watchdog_kick();
  }
  emit uiUpdate(ui_state);
}

Device::Device(QObject *parent) : brightness_filter(BACKLIGHT_OFFROAD, BACKLIGHT_TS, BACKLIGHT_DT), QObject(parent) {
}

void Device::update(const UIState &s) {
  updateBrightness(s);
  updateWakefulness(s);

  // TODO: remove from UIState and use signals
  QUIState::ui_state.awake = awake;
}

void Device::setAwake(bool on, bool reset) {
  if (on != awake) {
    awake = on;
    Hardware::set_display_power(awake);
    LOGD("setting display power %d", awake);
    emit displayPowerChanged(awake);
  }

  if (reset) {
    awake_timeout = 30 * UI_FREQ;
  }
}

void Device::updateBrightness(const UIState &s) {
  float clipped_brightness = BACKLIGHT_OFFROAD;
  if (s.scene.started) {
    // Scale to 0% to 100%
    clipped_brightness = 100.0 * s.scene.light_sensor;

    // CIE 1931 - https://www.photonstophotos.net/GeneralTopics/Exposure/Psychometric_Lightness_and_Gamma.htm
    if (clipped_brightness <= 8) {
      clipped_brightness = (clipped_brightness / 903.3);
    } else {
      clipped_brightness = std::pow((clipped_brightness + 16.0) / 116.0, 3.0);
    }

    // Scale back to 10% to 100%
    clipped_brightness = std::clamp(100.0f * clipped_brightness, 10.0f, 100.0f);

    // Limit brightness if running for too long
    if (Hardware::TICI()) {
      const float MAX_BRIGHTNESS_HOURS = 4;
      const float HOURLY_BRIGHTNESS_DECREASE = 5;
      float ui_running_hours = s.running_time / (60*60);
      float anti_burnin_max_percent = std::clamp(100.0f - HOURLY_BRIGHTNESS_DECREASE * (ui_running_hours - MAX_BRIGHTNESS_HOURS),
                                                 30.0f, 100.0f);
      clipped_brightness = std::min(clipped_brightness, anti_burnin_max_percent);
    }
  }

  if (s.scene.comma_stock_ui == 2 && (s.scene.do_not_disturb_mode == 1 || s.scene.do_not_disturb_mode == 3)) {
    if (s.scene.touched2) {
      sleep_time = 10 * UI_FREQ;
    } else if (sleep_time > 0) {
      sleep_time--;
    } else if (s.scene.started && sleep_time == -1) {
      sleep_time = 10 * UI_FREQ;
    }
  } else if (s.scene.autoScreenOff != -3 && s.scene.touched2) {
    sleep_time = s.scene.nTime;
  } else if (s.scene.controls_state.getAlertSize() != cereal::ControlsState::AlertSize::NONE && s.scene.autoScreenOff != -3) {
    sleep_time = s.scene.nTime;
  } else if (sleep_time > 0 && s.scene.autoScreenOff != -3) {
    sleep_time--;
  } else if (s.scene.started && sleep_time == -1 && s.scene.autoScreenOff != -3) {
    sleep_time = s.scene.nTime;
  }

  int brightness = brightness_filter.update(clipped_brightness);
  if (!awake) {
    brightness = 0;
  } else if ((s.scene.comma_stock_ui == 2 && (s.scene.do_not_disturb_mode == 1 || s.scene.do_not_disturb_mode == 3)) && s.scene.started && sleep_time == 0) {
    brightness = 0;
  } else if (s.scene.started && sleep_time == 0 && s.scene.autoScreenOff != -3) {
    if (s.scene.brightness_off < 4) {
      brightness = 0;
    } else if (s.scene.brightness_off < 9) {
      brightness = 1;
    } else {
      brightness = s.scene.brightness_off * 0.01 * brightness;
    }
  } else if ( s.scene.brightness ) {
    brightness = s.scene.brightness;
  }

  if (brightness != last_brightness) {
    std::thread{Hardware::set_brightness, brightness}.detach();
  }
  last_brightness = brightness;
}

void Device::updateWakefulness(const UIState &s) {
  awake_timeout = std::max(awake_timeout - 1, 0);

  bool should_wake = s.scene.started || s.scene.ignition;
  if (!should_wake) {
    // tap detection while display is off
    bool accel_trigger = abs(s.scene.accel_sensor - accel_prev) > 0.2;
    bool gyro_trigger = abs(s.scene.gyro_sensor - gyro_prev) > 0.15;
    should_wake = accel_trigger && gyro_trigger;
    gyro_prev = s.scene.gyro_sensor;
    accel_prev = (accel_prev * (accel_samples - 1) + s.scene.accel_sensor) / accel_samples;
  }

  setAwake(awake_timeout, should_wake);
}
