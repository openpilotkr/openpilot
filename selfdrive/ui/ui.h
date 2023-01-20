#pragma once

#include <map>
#include <memory>
#include <string>
#include <optional>

#include <QObject>
#include <QTimer>
#include <QColor>
#include <QTransform>
#include "nanovg.h"

#include "cereal/messaging/messaging.h"
#include "common/transformations/orientation.hpp"
#include "selfdrive/camerad/cameras/camera_common.h"
#include "selfdrive/common/mat.h"
#include "selfdrive/common/modeldata.h"
#include "selfdrive/common/params.h"
#include "selfdrive/common/util.h"

#define COLOR_BLACK nvgRGBA(0, 0, 0, 255)
#define COLOR_BLACK_ALPHA(x) nvgRGBA(0, 0, 0, x)
#define COLOR_WHITE nvgRGBA(255, 255, 255, 255)
#define COLOR_WHITE_ALPHA(x) nvgRGBA(255, 255, 255, x)
#define COLOR_RED_ALPHA(x) nvgRGBA(201, 34, 49, x)
#define COLOR_YELLOW nvgRGBA(218, 202, 37, 255)
#define COLOR_RED nvgRGBA(201, 34, 49, 255)
#define COLOR_OCHRE nvgRGBA(218, 111, 37, 255)
#define COLOR_OCHRE_ALPHA(x) nvgRGBA(218, 111, 37, x)
#define COLOR_GREEN nvgRGBA(0, 255, 0, 255)
#define COLOR_GREEN_ALPHA(x) nvgRGBA(0, 255, 0, x)
#define COLOR_BLUE nvgRGBA(0, 0, 255, 255)
#define COLOR_BLUE_ALPHA(x) nvgRGBA(0, 0, 255, x)
#define COLOR_ORANGE nvgRGBA(255, 175, 3, 255)
#define COLOR_ORANGE_ALPHA(x) nvgRGBA(255, 175, 3, x)
#define COLOR_YELLOW_ALPHA(x) nvgRGBA(218, 202, 37, x)
#define COLOR_GREY nvgRGBA(191, 191, 191, 1)

const int bdr_s = 15;
const int header_h = 420;
const int footer_h = 280;

const int UI_FREQ = 20;   // Hz
typedef cereal::CarControl::HUDControl::AudibleAlert AudibleAlert;

// TODO: this is also hardcoded in common/transformations/camera.py
// TODO: choose based on frame input size
const float y_offset = Hardware::EON() ? 0.0 : 150.0;
const float ZOOM = Hardware::EON() ? 2138.5 : 2912.8;

typedef struct Rect {
  int x, y, w, h;
  int centerX() const { return x + w / 2; }
  int centerY() const { return y + h / 2; }
  int right() const { return x + w; }
  int bottom() const { return y + h; }
  bool ptInRect(int px, int py) const {
    return px >= x && px < (x + w) && py >= y && py < (y + h);
  }
} Rect;

const Rect map_overlay_btn = {0, 465, 150, 150};
const Rect map_return_btn = {1770, 465, 150, 150};
const Rect map_btn = {1425, 905, 140, 140};
const Rect mapbox_btn = {465, 905, 140, 140};
const Rect rec_btn = {1745, 905, 140, 140};
const Rect laneless_btn = {1585, 905, 140, 140};
const Rect monitoring_btn = {50, 770, 140, 150};
const Rect ml_btn = {1265, 905, 140, 140};
const Rect stockui_btn = {15, 15, 184, 202};
const Rect tuneui_btn = {1720, 15, 184, 202};
const Rect livetunepanel_left_above_btn = {470, 570, 210, 170};
const Rect livetunepanel_right_above_btn = {1240, 570, 210, 170};
const Rect livetunepanel_left_btn = {470, 745, 210, 170};
const Rect livetunepanel_right_btn = {1240, 745, 210, 170};
const Rect speedlimit_btn = {220, 15, 190, 190};

struct Alert {
  QString text1;
  QString text2;
  QString type;
  cereal::ControlsState::AlertSize size;
  AudibleAlert sound;
  bool equal(const Alert &a2) {
    return text1 == a2.text1 && text2 == a2.text2 && type == a2.type && sound == a2.sound;
  }

  static Alert get(const SubMaster &sm, uint64_t started_frame) {
    if (sm.updated("controlsState")) {
      const cereal::ControlsState::Reader &cs = sm["controlsState"].getControlsState();
      return {cs.getAlertText1().cStr(), cs.getAlertText2().cStr(),
              cs.getAlertType().cStr(), cs.getAlertSize(),
              cs.getAlertSound()};
    } else if ((sm.frame - started_frame) > 5 * UI_FREQ) {
      const int CONTROLS_TIMEOUT = 5;
      // Handle controls timeout
      if (sm.rcv_frame("controlsState") < started_frame) {
        // car is started, but controlsState hasn't been seen at all
        return {"openpilot Unavailable", "Waiting for controls to start",
                "controlsWaiting", cereal::ControlsState::AlertSize::MID,
                AudibleAlert::NONE};
      } else if ((nanos_since_boot() - sm.rcv_time("controlsState")) / 1e9 > CONTROLS_TIMEOUT) {
        // car is started, but controls is lagging or died
        return {"TAKE CONTROL IMMEDIATELY", "Controls Unresponsive",
                "controlsUnresponsive", cereal::ControlsState::AlertSize::FULL,
                AudibleAlert::WARNING_IMMEDIATE};
      }
    }
    return {};
  }
};

typedef enum UIStatus {
  STATUS_DISENGAGED,
  STATUS_ENGAGED,
  STATUS_WARNING,
  STATUS_ALERT,
  STATUS_DND,
} UIStatus;

const QColor bg_colors [] = {
  [STATUS_DISENGAGED] =  QColor(0x17, 0x33, 0x49, 0xc8),
  [STATUS_ENGAGED] = QColor(0x17, 0x86, 0x44, 0x96),
  [STATUS_WARNING] = QColor(0xDA, 0x6F, 0x25, 0x96),
  [STATUS_ALERT] = QColor(0xC9, 0x22, 0x31, 0x96),
  [STATUS_DND] = QColor(0x32, 0x32, 0x32, 0x96),
};

typedef struct {
  float x, y;
} vertex_data;

typedef struct {
  vertex_data v[TRAJECTORY_SIZE * 2];
  int cnt;
} line_vertices_data;

typedef struct UIScene {

  mat3 view_from_calib;
  bool world_objects_visible;

  std::string alertTextMsg1;
  std::string alertTextMsg2;
  std::string alertTextMsg3;
  float alert_blinking_rate;
  cereal::PandaState::PandaType pandaType;

  bool brakePress;
  bool gasPress;
  bool brakeHold;
  bool touched = false;
  bool map_on_top = false;
  bool map_on_overlay = false;
  bool map_is_running = false;
  bool move_to_background = false;
  bool navi_on_boot = false;

  float gpsAccuracyUblox;
  float altitudeUblox;
  float bearingUblox;

  int cpuPerc;
  float cpuTemp;
  float batTemp;
  float ambientTemp;
  float batPercent;
  bool rightblindspot;
  bool leftblindspot;
  bool leftBlinker;
  bool rightBlinker;
  int blinker_blinkingrate;
  int tpms_blinkingrate = 120;
  int blindspot_blinkingrate = 120;
  int car_valid_status_changed = 0;
  float angleSteers;
  float desired_angle_steers;
  bool gap_by_speed_on;
  float steerRatio;
  bool brakeLights;
  bool steerOverride;
  float output_scale;
  int batteryPercent;
  bool batteryCharging;
  char batteryStatus[64];
  int fanSpeed;
  int tpmsUnit;
  float tpmsPressureFl;
  float tpmsPressureFr;
  float tpmsPressureRl;
  float tpmsPressureRr;
  int lateralControlMethod;
  float radarDistance;
  bool standStill;
  int limitSpeedCamera = 0;
  float limitSpeedCameraDist = 0;
  int mapSign;
  int mapSignCam;
  float vSetDis;
  bool cruiseAccStatus;
  int laneless_mode;
  int recording_count;
  int recording_quality;
  bool monitoring_mode;
  bool forceGearD;
  bool opkr_livetune_ui;
  bool driving_record;
  float steer_actuator_delay;
  bool batt_less;
  int cruise_gap;
  int dynamic_tr_mode;
  float dynamic_tr_value;
  bool touched2 = false;
  int brightness_off;
  int cameraOffset, pathOffset;
  int pidKp, pidKi, pidKd, pidKf;
  int indiInnerLoopGain, indiOuterLoopGain, indiTimeConstant, indiActuatorEffectiveness;
  int lqrScale, lqrKi, lqrDcGain;
  int torqueKp, torqueKf, torqueKi, torqueFriction, torqueMaxLatAccel;
  bool live_tune_panel_enable;
  int top_text_view;
  int live_tune_panel_list = 0;
  int list_count = 2;
  int nTime, autoScreenOff, brightness, awake;
  int nVolumeBoost = 0;
  bool read_params_once = false;
  bool nDebugUi1;
  bool nDebugUi2;
  bool nDebugUi3;
  bool nOpkrBlindSpotDetect;
  bool auto_gitpull = false;
  bool is_speed_over_limit = false;
  bool controlAllowed;
  bool steer_warning;
  bool stand_still;
  bool show_error;
  int display_maxspeed_time = 0;
  bool mapbox_running;
  int navi_select;
  bool tmux_error_check = false;
  bool speedlimit_signtype;
  bool sl_decel_off;
  bool osm_off_spdlimit;
  float a_req_value;
  bool osm_enabled;
  int radar_long_helper;
  float engine_rpm;
  bool cal_view = false;
  float ctrl_speed;
  float accel;
  bool animated_rpm;
  int max_animated_rpm;
  bool stop_line;
  int gear_step;
  float charge_meter;
  float multi_lat_selected;
  int do_not_disturb_mode;
  bool depart_chime_at_resume;
  int comma_stock_ui;

  cereal::DeviceState::Reader deviceState;
  cereal::CarState::Reader car_state;
  cereal::ControlsState::Reader controls_state;
  cereal::CarState::GearShifter getGearShifter;
  cereal::LateralPlan::Reader lateral_plan;
  cereal::LiveNaviData::Reader live_navi_data;
  cereal::LiveENaviData::Reader live_enavi_data;
  cereal::LiveMapData::Reader live_map_data;
  cereal::LongitudinalPlan::Reader longitudinal_plan;

  // gps
  int satelliteCount;
  float gpsAccuracy;

  // modelV2
  float lane_blindspot_probs[2];
  float lane_line_probs[4];
  float road_edge_stds[2];
  line_vertices_data track_vertices;
  line_vertices_data lane_line_vertices[4];
  line_vertices_data road_edge_vertices[2];
  line_vertices_data lane_blindspot_vertices[2];
  line_vertices_data stop_line_vertices;

  bool dm_active, engageable;

  // lead
  vertex_data lead_vertices[2];

  float light_sensor, accel_sensor, gyro_sensor;
  bool started, ignition, is_metric, longitudinal_control, end_to_end;
  uint64_t started_frame;

  float accel_prob[2];

  // atom
  struct _LiveParams
  {
    float angleOffset;
    float angleOffsetAverage;
    float stiffnessFactor;
    float steerRatio;
  } liveParams;

  struct _LateralPlan
  {
    float laneWidth;
    int standstillElapsedTime = 0;

    float dProb;
    float lProb;
    float rProb;

    float angleOffset;
    bool lanelessModeStatus;
    float totalCameraOffset;
  } lateralPlan;

  struct _LiveNaviData
  {
    int opkrspeedlimit;
    float opkrspeedlimitdist;
    int opkrroadsign;
    int opkrspeedsign;
    float opkrcurveangle;
    int   opkrturninfo;
    float opkrdisttoturn;
  } liveNaviData;

  struct _LiveENaviData
  {
    int eopkrspeedlimit;
    float eopkrsafetydist;
    int eopkrsafetysign;
    int eopkrturninfo;
    float eopkrdisttoturn;
    bool eopkrconalive;
    int eopkrroadlimitspeed;
    int eopkrlinklength;
    int eopkrcurrentlinkangle;
    int eopkrnextlinkangle;
    std::string eopkrposroadname;
    bool eopkrishighway;
    bool eopkristunnel;
  } liveENaviData;

  struct _LiveMapData
  {
    float ospeedLimit;
    float ospeedLimitAhead;
    float ospeedLimitAheadDistance;
    float oturnSpeedLimit;
    float oturnSpeedLimitEndDistance;
    int oturnSpeedLimitSign;
    std::string ocurrentRoadName;
    std::string oref;
    //float turnSpeedLimitsAhead[16]; // List
    //float turnSpeedLimitsAheadDistances[16]; // List
    //int turnSpeedLimitsAheadSigns[16]; // List
  } liveMapData;

  struct _LongitudinalPlan
  {
    float e2ex[13] = {0};
    float lead0[13] = {0};
    float lead1[13] = {0};
    float cruisetg[13] = {0};
    float stopline[13] = {0};
    float stopprob;
  } longitudinalPlan;

} UIScene;

typedef struct UIState {
  int fb_w = 0, fb_h = 0;
  NVGcontext *vg;

  // images
  std::map<std::string, int> images;

  std::unique_ptr<SubMaster> sm;

  UIStatus status;
  UIScene scene = {};

  bool awake;
  bool has_prime = false;
  bool sidebar_view;
  bool is_OpenpilotViewEnabled = false;

  QTransform car_space_transform;
  bool wide_camera;
  
  float running_time;
} UIState;


class QUIState : public QObject {
  Q_OBJECT

public:
  QUIState(QObject* parent = 0);

  // TODO: get rid of this, only use signal
  inline static UIState ui_state = {0};

signals:
  void uiUpdate(const UIState &s);
  void offroadTransition(bool offroad);

private slots:
  void update();

private:
  QTimer *timer;
  bool started_prev = true;
};


// device management class

class Device : public QObject {
  Q_OBJECT

public:
  Device(QObject *parent = 0);

private:
  // auto brightness
  const float accel_samples = 5*UI_FREQ;

  bool awake = false;
  int awake_timeout = 0;
  float accel_prev = 0;
  float gyro_prev = 0;
  int last_brightness = 0;
  FirstOrderFilter brightness_filter;

  QTimer *timer;
  int sleep_time = -1;

  void updateBrightness(const UIState &s);
  void updateWakefulness(const UIState &s);

signals:
  void displayPowerChanged(bool on);

public slots:
  void setAwake(bool on, bool reset);
  void update(const UIState &s);
};
