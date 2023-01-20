#include "selfdrive/ui/paint.h"

#include <cassert>
#include <cmath>

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#define NANOVG_GL3_IMPLEMENTATION
#define nvgCreate nvgCreateGL3
#else
#include <GLES3/gl3.h>
#define NANOVG_GLES3_IMPLEMENTATION
#define nvgCreate nvgCreateGLES3
#endif

#define NANOVG_GLES3_IMPLEMENTATION
#include <nanovg_gl.h>
#include <nanovg_gl_utils.h>

#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/ui.h"
#include <time.h> // opkr
#include "selfdrive/ui/dashcam.h"

static void ui_print(UIState *s, int x, int y,  const char* fmt, ... )
{
  char* msg_buf = NULL;
  va_list args;
  va_start(args, fmt);
  vasprintf( &msg_buf, fmt, args);
  va_end(args);
  nvgText(s->vg, x, y, msg_buf, NULL);
}

static void ui_draw_text(const UIState *s, float x, float y, const char *string, float size, NVGcolor color, const char *font_name) {
  nvgFontFace(s->vg, font_name);
  nvgFontSize(s->vg, size);
  nvgFillColor(s->vg, color);
  nvgText(s->vg, x, y, string, NULL);
}

static void draw_chevron(UIState *s, float x, float y, float sz, NVGcolor fillColor, NVGcolor glowColor) {
  // glow
  float g_xo = sz/5;
  float g_yo = sz/10;
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x+(sz*1.35)+g_xo, y+sz+g_yo);
  nvgLineTo(s->vg, x, y-g_xo);
  nvgLineTo(s->vg, x-(sz*1.35)-g_xo, y+sz+g_yo);
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, glowColor);
  nvgFill(s->vg);

  // chevron
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x+(sz*1.25), y+sz);
  nvgLineTo(s->vg, x, y);
  nvgLineTo(s->vg, x-(sz*1.25), y+sz);
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, fillColor);
  nvgFill(s->vg);
}

//atom(conan)'s steering wheel
static void ui_draw_circle_image_rotation(const UIState *s, int center_x, int center_y, int radius, const char *image, NVGcolor color, float img_alpha, float angleSteers = 0) {
  const int img_size = radius * 1.5;
  float img_rotation =  angleSteers/180*3.141592;
  int ct_pos = -radius * 0.75;

  nvgBeginPath(s->vg);
  nvgCircle(s->vg, center_x, center_y + (bdr_s+7), radius);
  nvgFillColor(s->vg, color);
  nvgFill(s->vg);
  //ui_draw_image(s, {center_x - (img_size / 2), center_y - (img_size / 2), img_size, img_size}, image, img_alpha);

  nvgSave( s->vg );
  nvgTranslate(s->vg, center_x, (center_y + (bdr_s*1.5)));
  nvgRotate(s->vg, -img_rotation);  

  ui_draw_image(s, {ct_pos, ct_pos, img_size, img_size}, image, img_alpha);
  nvgRestore(s->vg); 
}

static void ui_draw_circle_image(const UIState *s, int center_x, int center_y, int radius, const char *image, bool active) {
  float bg_alpha = active ? 0.3f : 0.1f;
  float img_alpha = active ? 1.0f : 0.15f;
  if (s->scene.monitoring_mode) {
    ui_draw_circle_image_rotation(s, center_x, center_y, radius, image, nvgRGBA(10, 120, 20, (255 * bg_alpha * 1.1)), img_alpha);
  } else {
    ui_draw_circle_image_rotation(s, center_x, center_y, radius, image, nvgRGBA(0, 0, 0, (255 * bg_alpha)), img_alpha);
  }
}

static void ui_draw_line(UIState *s, const line_vertices_data &vd, NVGcolor *color, NVGpaint *paint) {
  if (vd.cnt == 0) return;

  const vertex_data *v = &vd.v[0];
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, v[0].x, v[0].y);
  for (int i = 1; i < vd.cnt; i++) {
    nvgLineTo(s->vg, v[i].x, v[i].y);
  }
  nvgClosePath(s->vg);
  if (color) {
    nvgFillColor(s->vg, *color);
  } else if (paint) {
    nvgFillPaint(s->vg, *paint);
  }
  nvgFill(s->vg);
}

static void draw_lead(UIState *s, const cereal::RadarState::LeadData::Reader &lead_data, const vertex_data &vd) {
  // Draw lead car indicator
  auto [x, y] = vd;

  float fillAlpha = 0;
  float speedBuff = 10.;
  float leadBuff = 40.;
  float d_rel = lead_data.getDRel();
  float v_rel = lead_data.getVRel();
  if (d_rel < leadBuff) {
    fillAlpha = 255*(1.0-(d_rel/leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255*(-1*(v_rel/speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }

  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  x = std::clamp(x, 0.f, s->fb_w - sz / 2);
  y = std::fmin(s->fb_h - sz * .6, y);
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);

  if (s->scene.radarDistance < 149) {
    draw_chevron(s, x, y, sz, nvgRGBA(201, 34, 49, fillAlpha), COLOR_YELLOW);
    ui_draw_text(s, x, y + sz/1.5f, "R", 60, COLOR_BLUE, "sans-bold"); //neokii
  } else {
    draw_chevron(s, x, y, sz, nvgRGBA(165, 255, 135, fillAlpha), COLOR_GREEN);
    ui_draw_text(s, x, y + sz/1.5f, "V", 60, COLOR_BLACK, "sans-bold"); //hoya, vision
  }
}

static void ui_draw_stop_line(UIState *s, const cereal::ModelDataV2::StopLineData::Reader &stop_line_data, const line_vertices_data &vd) {
  NVGcolor color = nvgRGBAf(0.7, 0.0, 0.0, stop_line_data.getProb());
  ui_draw_line(s, vd, &color, nullptr);
}

static void ui_draw_stop_sign(UIState *s) {
  float center_x = 1500.0f;
  float center_y = 105.0f;
  float radius_i = 5.0f;
  float radius_o = 75.0f;

  if (s->scene.longitudinalPlan.e2ex[12] > 30 && s->scene.longitudinalPlan.stopline[12] < 10 && s->scene.car_state.getVEgo() < 1.0) {
    nvgBeginPath(s->vg);
    nvgCircle(s->vg, center_x, center_y, radius_i+radius_o);
    NVGpaint stop_sign = nvgRadialGradient(s->vg, center_x, center_y, radius_i, radius_o, nvgRGBAf(0.0, 1.0, 0.0, 0.9), nvgRGBAf(0.0, 0.0, 0.0, 0.3));
    nvgFillPaint(s->vg, stop_sign);
    nvgFill(s->vg);
  } else if (s->scene.longitudinalPlan.e2ex[12] > 0 && s->scene.longitudinalPlan.e2ex[12] < 85 && s->scene.longitudinalPlan.stopline[12] < 85) {
    nvgBeginPath(s->vg);
    nvgCircle(s->vg, center_x, center_y, radius_i+radius_o);
    NVGpaint stop_sign = nvgRadialGradient(s->vg, center_x, center_y, radius_i, radius_o, nvgRGBAf(1.0, 0.0, 0.0, 0.9), nvgRGBAf(0.0, 0.0, 0.0, 0.3));
    nvgFillPaint(s->vg, stop_sign);
    nvgFill(s->vg);
  }
}

static void ui_draw_vision_lane_lines(UIState *s) {
  const UIScene &scene = s->scene;
  NVGpaint track_bg;
  int steerOverride = scene.car_state.getSteeringPressed();
  int torque_scale = (int)fabs(255*(float)scene.output_scale*0.9);
  int red_lvl = fmin(255, torque_scale);
  int green_lvl = fmin(255, 255-torque_scale);

  float red_lvl_line = 0;
  float green_lvl_line = 0;



  // paint left blindspot line
  NVGcolor color;
  if( scene.leftblindspot )
  {
    color = nvgRGBAf(1.0, 0.0, 0.0, std::clamp<float>(1.0 - scene.lane_blindspot_probs[0], 0.0, 1.0));
    ui_draw_line(s, scene.lane_blindspot_vertices[0], &color, nullptr);
   }

  if( scene.rightblindspot )
  {
    color = nvgRGBAf(1.0, 0.0, 0.0, std::clamp<float>(1.0 - scene.lane_blindspot_probs[1], 0.0, 1.0));
    ui_draw_line(s, scene.lane_blindspot_vertices[1], &color, nullptr);
  }

  if (!scene.lateralPlan.lanelessModeStatus) {
    // paint lanelines, Hoya's colored lane line
    for (int i = 0; i < std::size(scene.lane_line_vertices); i++) {
      if (scene.lane_line_probs[i] > 0.4){
        red_lvl_line = 1.0 - ((scene.lane_line_probs[i] - 0.4) * 2.5);
        green_lvl_line = 1.0;
      } else {
        red_lvl_line = 1.0;
        green_lvl_line = 1.0 - ((0.4 - scene.lane_line_probs[i]) * 2.5);
      }
      color = nvgRGBAf(1.0, 1.0, 1.0, scene.lane_line_probs[i]);
      if (scene.comma_stock_ui != 1) {
        color = nvgRGBAf(red_lvl_line, green_lvl_line, 0, 1);
      }
      ui_draw_line(s, scene.lane_line_vertices[i], &color, nullptr);
    }

    // paint road edges
    for (int i = 0; i < std::size(scene.road_edge_vertices); i++) {
      color = nvgRGBAf(1.0, 0.0, 0.0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0));
      ui_draw_line(s, scene.road_edge_vertices[i], &color, nullptr);
    }
  }
  if (scene.controls_state.getEnabled() && scene.comma_stock_ui != 1) {
    if (steerOverride) {
      track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h*.4,
        COLOR_BLACK_ALPHA(80), COLOR_BLACK_ALPHA(20));
    } else {
      if (!scene.lateralPlan.lanelessModeStatus) {
        track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h*.4,
          nvgRGBA(red_lvl, green_lvl, 0, 160), nvgRGBA((int)(0.7*red_lvl), (int)(0.7*green_lvl), 0, 30));
      } else {
        track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h*.4,
          nvgRGBA(red_lvl, 150, green_lvl, 160), nvgRGBA((int)(0.7*red_lvl), 150, (int)(0.7*green_lvl), 30));
      }
    }
  } else {
    // Draw white vision track
    track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                        COLOR_WHITE_ALPHA(150), COLOR_WHITE_ALPHA(20));
  }

 
  // paint path
  ui_draw_line(s, scene.track_vertices, nullptr, &track_bg);
}

// Draw all world space objects.
static void ui_draw_world(UIState *s) {
  nvgScissor(s->vg, 0, 0, s->fb_w, s->fb_h);

  // Draw lane edges and vision/mpc tracks
  ui_draw_vision_lane_lines(s);

  // Draw lead and stop line indicators if openpilot is handling longitudinal
  //if (s->scene.longitudinal_control) {
  if (true) {
    auto lead_one = (*s->sm)["radarState"].getRadarState().getLeadOne();
    auto lead_two = (*s->sm)["radarState"].getRadarState().getLeadTwo();
    if (lead_one.getStatus()) {
      draw_lead(s, lead_one, s->scene.lead_vertices[0]);
    }
    if (lead_two.getStatus() && (std::abs(lead_one.getDRel() - lead_two.getDRel()) > 3.0)) {
      draw_lead(s, lead_two, s->scene.lead_vertices[1]);
    }
    if (s->scene.stop_line && s->scene.longitudinalPlan.stopline[12] > 3.0) {
      auto stop_line = (*s->sm)["modelV2"].getModelV2().getStopLine();
      if (stop_line.getProb() > .5) {
        ui_draw_stop_line(s, stop_line, s->scene.stop_line_vertices);
      }
    }
  }
  nvgResetScissor(s->vg);
}

static void ui_draw_standstill(UIState *s) {
  const UIScene &scene = s->scene;

  int viz_standstill_x = s->fb_w - 560;
  int viz_standstill_y = bdr_s + 160 + 250;
  
  int minute = 0;
  int second = 0;

  minute = int(scene.lateralPlan.standstillElapsedTime / 60);
  second = int(scene.lateralPlan.standstillElapsedTime) - (minute * 60);

  if (scene.standStill) {
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
    if (scene.mapbox_running) {
      nvgFontSize(s->vg, 125);
    } else {
      nvgFontSize(s->vg, 170);
    }
    nvgFillColor(s->vg, COLOR_ORANGE_ALPHA(240));
    ui_print(s, scene.mapbox_running ? viz_standstill_x + 250 : viz_standstill_x, viz_standstill_y, "STOP");
    if (scene.mapbox_running) {
      nvgFontSize(s->vg, 150);
    } else {
      nvgFontSize(s->vg, 200);
    }
    nvgFillColor(s->vg, COLOR_WHITE_ALPHA(240));
    ui_print(s, scene.mapbox_running ? viz_standstill_x + 250 : viz_standstill_x, scene.mapbox_running ? viz_standstill_y+100 : viz_standstill_y+150, "%01d:%02d", minute, second);
  }
}

static void ui_draw_debug(UIState *s) {
  const UIScene &scene = s->scene;

  int ui_viz_rx = bdr_s + 190;
  int ui_viz_ry = bdr_s + 100;
  int ui_viz_rx_center = s->fb_w/2;
  int debug_y1 = 1010-bdr_s+(scene.mapbox_running ? 18:0)-(scene.animated_rpm?60:0);
  int debug_y2 = 1050-bdr_s+(scene.mapbox_running ? 3:0)-(scene.animated_rpm?60:0);
  int debug_y3 = 970-bdr_s+(scene.mapbox_running ? 18:0)-(scene.animated_rpm?60:0);
  
  nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);

  if (scene.nDebugUi1) {
    ui_draw_text(s, 0, debug_y1, scene.alertTextMsg1.c_str(), scene.mapbox_running?34:45, COLOR_WHITE_ALPHA(125), "sans-semibold");
    ui_draw_text(s, 0, debug_y2, scene.alertTextMsg2.c_str(), scene.mapbox_running?34:45, COLOR_WHITE_ALPHA(125), "sans-semibold");
  }
  if (scene.nDebugUi3) {
    ui_draw_text(s, 0, debug_y3, scene.alertTextMsg3.c_str(), scene.mapbox_running?34:45, COLOR_WHITE_ALPHA(125), "sans-semibold");
  }

  
  nvgFillColor(s->vg, COLOR_WHITE_ALPHA(125));
  if (scene.nDebugUi2) {
    //if (scene.gpsAccuracyUblox != 0.00) {
    //  nvgFontSize(s->vg, 34);
    //  ui_print(s, 28, 28, "LAT／LON: %.5f／%.5f", scene.latitudeUblox, scene.longitudeUblox);
    //}
    if (scene.mapbox_running) {
      nvgFontSize(s->vg, 40);
    } else {
      nvgFontSize(s->vg, 50);
    }
    //ui_print(s, ui_viz_rx, ui_viz_ry, "Live Parameters");
    //ui_print(s, ui_viz_rx, ui_viz_ry+200, "%.0f", scene.ctrl_speed);
    ui_print(s, ui_viz_rx, ui_viz_ry+240, "SR:%.2f", scene.liveParams.steerRatio);
    //ui_print(s, ui_viz_rx, ui_viz_ry+100, "AOfs:%.2f", scene.liveParams.angleOffset);
    ui_print(s, ui_viz_rx, ui_viz_ry+280, "AA:%.2f", scene.liveParams.angleOffsetAverage);
    ui_print(s, ui_viz_rx, ui_viz_ry+320, "SF:%.2f", scene.liveParams.stiffnessFactor);

    ui_print(s, ui_viz_rx, ui_viz_ry+360, "AD:%.2f", scene.steer_actuator_delay);
    ui_print(s, ui_viz_rx, ui_viz_ry+400, "OS:%.2f", abs(scene.output_scale));
    ui_print(s, ui_viz_rx, ui_viz_ry+440, "%.2f|%.2f", scene.lateralPlan.lProb, scene.lateralPlan.rProb);
    ui_print(s, ui_viz_rx, ui_viz_ry+480, "%.1f/%.1fm", scene.lateralPlan.dProb, scene.lateralPlan.laneWidth); // High dProb is more related to LaneLine, Low is Laneless
    ui_print(s, ui_viz_rx, ui_viz_ry+520, "%.1f/%.1f/%.1f/%.1f/%.1f/%.1f", std::clamp<float>(1.0 - scene.road_edge_stds[0], 0.0, 1.0), scene.lane_line_probs[0], scene.lane_line_probs[1], scene.lane_line_probs[2], scene.lane_line_probs[3], std::clamp<float>(1.0 - scene.road_edge_stds[1], 0.0, 1.0));
    // const std::string stateStrings[] = {"disabled", "preEnabled", "enabled", "softDisabling"};
    // ui_print(s, ui_viz_rx, ui_viz_ry+520, "%s", stateStrings[(int)(*s->sm)["controlsState"].getControlsState().getState()].c_str());
    //ui_print(s, ui_viz_rx, ui_viz_ry+800, "A:%.5f", scene.accel_sensor2);
    if (!scene.nDebugUi3) {
      if (scene.navi_select == 3) {
        if (scene.liveENaviData.eopkrsafetysign) ui_print(s, ui_viz_rx, ui_viz_ry+560, "CS:%d", scene.liveENaviData.eopkrsafetysign);
        if (scene.liveENaviData.eopkrspeedlimit) ui_print(s, ui_viz_rx, ui_viz_ry+600, "SL:%d/DS:%.0f", scene.liveENaviData.eopkrspeedlimit, scene.liveENaviData.eopkrsafetydist);
        if (scene.liveENaviData.eopkrturninfo) ui_print(s, ui_viz_rx, ui_viz_ry+640, "TI:%d/DT:%.0f", scene.liveENaviData.eopkrturninfo, scene.liveENaviData.eopkrdisttoturn);
        if (scene.liveENaviData.eopkrroadlimitspeed > 0 && scene.liveENaviData.eopkrroadlimitspeed < 200) ui_print(s, ui_viz_rx, ui_viz_ry+680, "RS:%d", scene.liveENaviData.eopkrroadlimitspeed);
        if (scene.liveENaviData.eopkrishighway || scene.liveENaviData.eopkristunnel) ui_print(s, ui_viz_rx, ui_viz_ry+720, "H:%d/T:%d", scene.liveENaviData.eopkrishighway, scene.liveENaviData.eopkristunnel);
        //if (scene.liveENaviData.eopkrlinklength || scene.liveENaviData.eopkrcurrentlinkangle || scene.liveENaviData.eopkrnextlinkangle) ui_print(s, ui_viz_rx, ui_viz_ry+840, "L:%d/C:%d/N:%d", scene.liveENaviData.eopkrlinklength, scene.liveENaviData.eopkrcurrentlinkangle, scene.liveENaviData.eopkrnextlinkangle);
      } else if (scene.map_is_running) {
        if (scene.liveNaviData.opkrroadsign) ui_print(s, ui_viz_rx, ui_viz_ry+560, "RS:%d", scene.liveNaviData.opkrroadsign);
        if (scene.liveNaviData.opkrspeedsign) ui_print(s, ui_viz_rx, ui_viz_ry+600, "CS:%d", scene.liveNaviData.opkrspeedsign);
        if (scene.liveNaviData.opkrspeedlimit) ui_print(s, ui_viz_rx, ui_viz_ry+640, "SL:%d", scene.liveNaviData.opkrspeedlimit);
        if (scene.liveNaviData.opkrspeedlimitdist) ui_print(s, ui_viz_rx, ui_viz_ry+680, "DS:%.0f", scene.liveNaviData.opkrspeedlimitdist);
        if (scene.liveNaviData.opkrturninfo) ui_print(s, ui_viz_rx, ui_viz_ry+720, "TI:%d", scene.liveNaviData.opkrturninfo);
        if (scene.liveNaviData.opkrdisttoturn) ui_print(s, ui_viz_rx, ui_viz_ry+760, "DT:%.0f", scene.liveNaviData.opkrdisttoturn);
      } else if (!scene.map_is_running && (*s->sm)["carState"].getCarState().getSafetySign() > 19) {
        ui_print(s, ui_viz_rx, ui_viz_ry+560, "SL:%.0f", (*s->sm)["carState"].getCarState().getSafetySign());
        ui_print(s, ui_viz_rx, ui_viz_ry+600, "DS:%.0f", (*s->sm)["carState"].getCarState().getSafetyDist());
      }
    }
  if (scene.nDebugUi3) {
    ui_print(s, ui_viz_rx, ui_viz_ry+560, "0: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
     scene.longitudinalPlan.lead0[0], scene.longitudinalPlan.lead0[1], scene.longitudinalPlan.lead0[2], scene.longitudinalPlan.lead0[3], scene.longitudinalPlan.lead0[4],
     scene.longitudinalPlan.lead0[5], scene.longitudinalPlan.lead0[6], scene.longitudinalPlan.lead0[7], scene.longitudinalPlan.lead0[8], scene.longitudinalPlan.lead0[9],
     scene.longitudinalPlan.lead0[10], scene.longitudinalPlan.lead0[11], scene.longitudinalPlan.lead0[12]);
    ui_print(s, ui_viz_rx, ui_viz_ry+600, "1: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
     scene.longitudinalPlan.lead1[0], scene.longitudinalPlan.lead1[1], scene.longitudinalPlan.lead1[2], scene.longitudinalPlan.lead1[3], scene.longitudinalPlan.lead1[4],
     scene.longitudinalPlan.lead1[5], scene.longitudinalPlan.lead1[6], scene.longitudinalPlan.lead1[7], scene.longitudinalPlan.lead1[8], scene.longitudinalPlan.lead1[9],
     scene.longitudinalPlan.lead1[10], scene.longitudinalPlan.lead1[11], scene.longitudinalPlan.lead1[12]);
    ui_print(s, ui_viz_rx, ui_viz_ry+640, "C: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
     scene.longitudinalPlan.cruisetg[0], scene.longitudinalPlan.cruisetg[1], scene.longitudinalPlan.cruisetg[2], scene.longitudinalPlan.cruisetg[3], scene.longitudinalPlan.cruisetg[4],
     scene.longitudinalPlan.cruisetg[5], scene.longitudinalPlan.cruisetg[6], scene.longitudinalPlan.cruisetg[7], scene.longitudinalPlan.cruisetg[8], scene.longitudinalPlan.cruisetg[9],
     scene.longitudinalPlan.cruisetg[10], scene.longitudinalPlan.cruisetg[11], scene.longitudinalPlan.cruisetg[12]);
    ui_print(s, ui_viz_rx, ui_viz_ry+680, "X: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
     scene.longitudinalPlan.e2ex[0], scene.longitudinalPlan.e2ex[1], scene.longitudinalPlan.e2ex[2], scene.longitudinalPlan.e2ex[3], scene.longitudinalPlan.e2ex[4],
     scene.longitudinalPlan.e2ex[5], scene.longitudinalPlan.e2ex[6], scene.longitudinalPlan.e2ex[7], scene.longitudinalPlan.e2ex[8], scene.longitudinalPlan.e2ex[9],
     scene.longitudinalPlan.e2ex[10], scene.longitudinalPlan.e2ex[11], scene.longitudinalPlan.e2ex[12]);
    ui_print(s, ui_viz_rx, ui_viz_ry+720, "S: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
     scene.longitudinalPlan.stopline[0], scene.longitudinalPlan.stopline[1], scene.longitudinalPlan.stopline[2], scene.longitudinalPlan.stopline[3], scene.longitudinalPlan.stopline[4],
     scene.longitudinalPlan.stopline[5], scene.longitudinalPlan.stopline[6], scene.longitudinalPlan.stopline[7], scene.longitudinalPlan.stopline[8], scene.longitudinalPlan.stopline[9],
     scene.longitudinalPlan.stopline[10], scene.longitudinalPlan.stopline[11], scene.longitudinalPlan.stopline[12]);
    ui_print(s, ui_viz_rx, ui_viz_ry+760, "P:%.1f", scene.longitudinalPlan.stopprob);
  }
    nvgFontSize(s->vg, 50);
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);


    char const* szLaCMethod = nullptr;
    char const* szLaCMethodCur = nullptr;
    switch( scene.lateralControlMethod  )
      {
        case  0: szLaCMethod = "PID"; break;
        case  1: szLaCMethod = "INDI"; break;
        case  2: szLaCMethod = "LQR"; break;
        case  3: szLaCMethod = "TORQUE"; break;
        case  4: szLaCMethod = "MULTI"; break;
      }
    switch( (int)scene.multi_lat_selected  )
      {
        case  0: szLaCMethodCur = "PID"; break;
        case  1: szLaCMethodCur = "INDI"; break;
        case  2: szLaCMethodCur = "LQR"; break;
        case  3: szLaCMethodCur = "TORQUE"; break;
      }
    if ( !scene.animated_rpm )
    {
      if( szLaCMethod )
          ui_print(s, ui_viz_rx_center, bdr_s+295, szLaCMethod );
      if (scene.lateralControlMethod == 4) {
        if( szLaCMethodCur )
            ui_print(s, ui_viz_rx_center, bdr_s+330, szLaCMethodCur );
        }
    } else {
      if( szLaCMethod )
          ui_print(s, ui_viz_rx_center, bdr_s+320, szLaCMethod );
      if (scene.lateralControlMethod == 4) {
        if( szLaCMethodCur )
            ui_print(s, ui_viz_rx_center, bdr_s+355, szLaCMethodCur );
        }
    }
    nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
    if (scene.osm_enabled) {
      ui_print(s, ui_viz_rx+(scene.mapbox_running ? 150:200), ui_viz_ry+240, "SL:%.0f", scene.liveMapData.ospeedLimit);
      ui_print(s, ui_viz_rx+(scene.mapbox_running ? 150:200), ui_viz_ry+280, "SLA:%.0f", scene.liveMapData.ospeedLimitAhead);
      ui_print(s, ui_viz_rx+(scene.mapbox_running ? 150:200), ui_viz_ry+320, "SLAD:%.0f", scene.liveMapData.ospeedLimitAheadDistance);
      ui_print(s, ui_viz_rx+(scene.mapbox_running ? 150:200), ui_viz_ry+360, "TSL:%.0f", scene.liveMapData.oturnSpeedLimit);
      ui_print(s, ui_viz_rx+(scene.mapbox_running ? 150:200), ui_viz_ry+400, "TSLED:%.0f", scene.liveMapData.oturnSpeedLimitEndDistance);
      ui_print(s, ui_viz_rx+(scene.mapbox_running ? 150:200), ui_viz_ry+440, "TSLS:%d", scene.liveMapData.oturnSpeedLimitSign);
      ui_print(s, ui_viz_rx+(scene.mapbox_running ? 150:200), ui_viz_ry+480, "TCO:%.2f", -scene.lateralPlan.totalCameraOffset);
    }
  }
  if (scene.cal_view) {
    ui_print(s, ui_viz_rx, ui_viz_ry+800, "BF:%.1f   RL:%.1f°", scene.accel_prob[0], scene.accel_prob[1]);
  }
}

/*
  park @1;
  drive @2;
  neutral @3;
  reverse @4;
  sport @5;
  low @6;
  brake @7;
  eco @8;
*/
static void ui_draw_gear( UIState *s ) {
  const UIScene &scene = s->scene;  
  NVGcolor nColor = COLOR_WHITE;
  int x_pos = s->fb_w - (90 + bdr_s);
  int y_pos = bdr_s + 140;
  int ngetGearShifter = int(scene.getGearShifter);
  //int  x_pos = 1795;
  //int  y_pos = 155;
  char str_msg[512];

  nvgFontFace(s->vg, "sans-bold");
  nvgFontSize(s->vg, 230 );
  switch( ngetGearShifter )
  {
    case 1 : strcpy( str_msg, "P" ); nColor = nvgRGBA(200, 200, 255, 255); break;
    case 2 : strcpy( str_msg, "D" ); nColor = COLOR_GREEN; break;
    case 3 : strcpy( str_msg, "N" ); nColor = COLOR_WHITE; break;
    case 4 : strcpy( str_msg, "R" ); nColor = COLOR_RED; break;
    case 5 : strcpy( str_msg, "M" ); nColor = COLOR_GREEN; break;
    case 7 : strcpy( str_msg, "B" ); break;
    default: sprintf( str_msg, "%d", ngetGearShifter ); break;
  }

  nvgFillColor(s->vg, nColor);
  ui_print( s, x_pos, y_pos, str_msg );
}

static void ui_draw_vision_maxspeed_org(UIState *s) {
  const int SET_SPEED_NA = 255;
  float maxspeed = round(s->scene.controls_state.getVCruise());
  float cruise_speed = round(s->scene.vSetDis);
  const bool is_cruise_set = maxspeed != 0 && maxspeed != SET_SPEED_NA;
  int limitspeedcamera = s->scene.limitSpeedCamera;

  if (s->scene.cruiseAccStatus ) {
    if (s->scene.navi_select != 3 || (s->scene.navi_select == 3 && (s->scene.mapSign != 20 && s->scene.mapSign != 21))) {
      s->scene.is_speed_over_limit = limitspeedcamera > 21 && ((s->scene.car_state.getVEgo() * (s->scene.is_metric ? 3.6 : 2.2369363)) > s->scene.ctrl_speed+1.5);
    } else {
      s->scene.is_speed_over_limit = false;
    }
  } else {
    if (s->scene.navi_select != 3 || (s->scene.navi_select == 3 && (s->scene.mapSign != 20 && s->scene.mapSign != 21))) {
      s->scene.is_speed_over_limit = limitspeedcamera > 21 && ((s->scene.car_state.getVEgo() * (s->scene.is_metric ? 3.6 : 2.2369363)) > s->scene.limitSpeedCamera+1.5);
    } else {
      s->scene.is_speed_over_limit = false;
    }
  }
  if ((s->scene.navi_select == 3 && (s->scene.mapSign == 20 || s->scene.mapSign == 21))) {
    limitspeedcamera = 0;
  }

  const Rect rect = {bdr_s, bdr_s, 184, 202};
  NVGcolor color = COLOR_BLACK_ALPHA(100);
  if (s->scene.is_speed_over_limit) {
    color = COLOR_OCHRE_ALPHA(100);
  } else if (limitspeedcamera > 21 && !s->scene.is_speed_over_limit) {
    color = nvgRGBA(0, 120, 0, 100);
  } else if (s->scene.cruiseAccStatus) {
    color = nvgRGBA(0, 100, 200, 100);
  } else if (s->scene.controls_state.getEnabled()) {
    color = COLOR_WHITE_ALPHA(75);
  }
  ui_fill_rect(s->vg, rect, color, 30.);
  ui_draw_rect(s->vg, rect, COLOR_WHITE_ALPHA(100), 10, 20.);

  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  if (cruise_speed >= 20 && s->scene.controls_state.getEnabled()) {
    const std::string cruise_speed_str = std::to_string((int)std::nearbyint(cruise_speed));
    ui_draw_text(s, rect.centerX(), bdr_s+65, cruise_speed_str.c_str(), 26 * 3.3, COLOR_WHITE_ALPHA(is_cruise_set ? 200 : 100), "sans-bold");
  } else {
  	ui_draw_text(s, rect.centerX(), bdr_s+65, "-", 26 * 3.3, COLOR_WHITE_ALPHA(is_cruise_set ? 200 : 100), "sans-semibold");
  }
  if (is_cruise_set) {
    const std::string maxspeed_str = std::to_string((int)std::nearbyint(maxspeed));
    ui_draw_text(s, rect.centerX(), bdr_s+165, maxspeed_str.c_str(), 48 * 2.7, COLOR_WHITE, "sans-bold");
  } else {
    ui_draw_text(s, rect.centerX(), bdr_s+165, "-", 42 * 2.7, COLOR_WHITE_ALPHA(100), "sans-semibold");
  }
}

static void ui_draw_vision_maxspeed(UIState *s) {
  const int SET_SPEED_NA = 255;
  float maxspeed = round((*s->sm)["controlsState"].getControlsState().getVCruise());
  const bool is_cruise_set = maxspeed != 0 && maxspeed != SET_SPEED_NA && s->scene.controls_state.getEnabled();

  int viz_max_o = 184; //offset value to move right
  const Rect rect = {bdr_s, bdr_s, 184+viz_max_o, 202};
  ui_fill_rect(s->vg, rect, COLOR_BLACK_ALPHA(100), 30.);
  ui_draw_rect(s->vg, rect, COLOR_WHITE_ALPHA(100), 10, 20.);

  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  ui_draw_text(s, rect.centerX()+viz_max_o/2, bdr_s+65, "MAX", 26 * 2.5, COLOR_WHITE_ALPHA(is_cruise_set ? 200 : 100), "sans-regular");
  if (is_cruise_set) {
    const std::string maxspeed_str = std::to_string((int)std::nearbyint(maxspeed));
    ui_draw_text(s, rect.centerX()+viz_max_o/2, bdr_s+165, maxspeed_str.c_str(), 48 * 2.5, COLOR_WHITE, "sans-bold");
  } else {
    ui_draw_text(s, rect.centerX()+viz_max_o/2, bdr_s+165, "-", 42 * 2.5, COLOR_WHITE_ALPHA(100), "sans-semibold");
  }
}

static void ui_draw_vision_cruise_speed(UIState *s) {
  const int SET_SPEED_NA = 255;
  float maxspeed = round(s->scene.controls_state.getVCruise());
  const bool is_cruise_set = maxspeed != 0 && maxspeed != SET_SPEED_NA;
  int limitspeedcamera = s->scene.limitSpeedCamera;
  //if (is_cruise_set && !s->scene.is_metric) { maxspeed *= 0.6225; }
  float cruise_speed = round(s->scene.vSetDis);
  if (s->scene.cruiseAccStatus) {
    if (s->scene.navi_select != 3 || (s->scene.navi_select == 3 && (s->scene.mapSign != 20 && s->scene.mapSign != 21))) {
      s->scene.is_speed_over_limit = limitspeedcamera > 21 && ((s->scene.car_state.getVEgo() * (s->scene.is_metric ? 3.6 : 2.2369363)) > s->scene.ctrl_speed+1.5);
    } else {
      s->scene.is_speed_over_limit = false;
    }
  } else {
    if (s->scene.navi_select != 3 || (s->scene.navi_select == 3 && (s->scene.mapSign != 20 && s->scene.mapSign != 21))) {
      s->scene.is_speed_over_limit = limitspeedcamera > 21 && ((s->scene.car_state.getVEgo() * (s->scene.is_metric ? 3.6 : 2.2369363)) > limitspeedcamera+1.5);
    } else {
      s->scene.is_speed_over_limit = false;
    }
  }
  if ((s->scene.navi_select == 3 && (s->scene.mapSign == 20 || s->scene.mapSign == 21))) {
    limitspeedcamera = 0;
  }

  const Rect rect = {bdr_s, bdr_s, 184, 202};

  NVGcolor color = COLOR_GREY;
  if (s->scene.is_speed_over_limit) {
    color = COLOR_OCHRE_ALPHA(200);
  } else if (limitspeedcamera > 21 && (!s->scene.is_speed_over_limit )) {
    color = nvgRGBA(0, 120, 0, 200);
  } else if (s->scene.cruiseAccStatus) {
    color = nvgRGBA(0, 100, 200, 200);
  } else if (s->scene.controls_state.getEnabled()) {
    color = COLOR_WHITE_ALPHA(75);
  }
  ui_fill_rect(s->vg, rect, color, 30.);
  ui_draw_rect(s->vg, rect, COLOR_WHITE_ALPHA(100), 10, 20.);

  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  if (limitspeedcamera > 21 && limitspeedcamera <= round(maxspeed)) {
    ui_draw_text(s, rect.centerX(), bdr_s+65, "LIMIT", 26 * 2.5, COLOR_WHITE_ALPHA(s->scene.cruiseAccStatus ? 200 : 100), "sans-regular");
  } else if (is_cruise_set) {
    const std::string maxspeed_str = std::to_string((int)std::nearbyint(maxspeed));
    ui_draw_text(s, rect.centerX(), bdr_s+65, maxspeed_str.c_str(), 26 * 3.3, COLOR_WHITE, "sans-bold");
  } else {
    ui_draw_text(s, rect.centerX(), bdr_s+65, "-", 26 * 3.3, COLOR_WHITE_ALPHA(100), "sans-semibold");
  }

  const std::string cruise_speed_str = std::to_string((int)std::nearbyint(cruise_speed));
  if (s->scene.controls_state.getEnabled() && !s->scene.cruiseAccStatus && limitspeedcamera > 21) {
    const std::string limitspeedcamera_str = std::to_string((int)std::nearbyint(limitspeedcamera));
    ui_draw_text(s, rect.centerX(), bdr_s+165, limitspeedcamera_str.c_str(), 48 * 2.5, COLOR_WHITE, "sans-bold");
  } else if (cruise_speed >= 20 && s->scene.controls_state.getEnabled()) {
    ui_draw_text(s, rect.centerX(), bdr_s+165, cruise_speed_str.c_str(), 48 * 2.5, COLOR_WHITE, "sans-bold");
  } else {
    ui_draw_text(s, rect.centerX(), bdr_s+165, "-", 42 * 2.5, COLOR_WHITE_ALPHA(100), "sans-semibold");
  }
}

static void ui_draw_vision_speed(UIState *s) {
  const float speed = std::max(0.0, (*s->sm)["carState"].getCarState().getVEgo()*(s->scene.is_metric ? 3.6 : 2.2369363));
  const std::string speed_str = std::to_string((int)std::nearbyint(speed));
  UIScene &scene = s->scene;  
  const int viz_speed_w = 250;
  const int viz_speed_x = s->fb_w/2 - viz_speed_w/2;
  const int header_h2 = 400;

  // turning blinker from kegman, moving signal by OPKR
  if (scene.leftBlinker && scene.comma_stock_ui != 1) {
    nvgBeginPath(s->vg);
    nvgMoveTo(s->vg, viz_speed_x, header_h2/4);
    nvgLineTo(s->vg, viz_speed_x - viz_speed_w/4, header_h2/4);
    nvgLineTo(s->vg, viz_speed_x - viz_speed_w/2, header_h2/2);
    nvgLineTo(s->vg, viz_speed_x - viz_speed_w/4, header_h2/4 + header_h2/2);
    nvgLineTo(s->vg, viz_speed_x, header_h2/4 + header_h2/2);
    nvgLineTo(s->vg, viz_speed_x - viz_speed_w/4, header_h2/2);
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, nvgRGBA(255,230,70,(scene.blinker_blinkingrate<=120 && scene.blinker_blinkingrate>=50)?70:0));
    nvgFill(s->vg);
    nvgBeginPath(s->vg);
    nvgMoveTo(s->vg, viz_speed_x - 125, header_h2/4);
    nvgLineTo(s->vg, viz_speed_x - 125 - viz_speed_w/4, header_h2/4);
    nvgLineTo(s->vg, viz_speed_x - 125 - viz_speed_w/2, header_h2/2);
    nvgLineTo(s->vg, viz_speed_x - 125 - viz_speed_w/4, header_h2/4 + header_h2/2);
    nvgLineTo(s->vg, viz_speed_x - 125, header_h2/4 + header_h2/2);
    nvgLineTo(s->vg, viz_speed_x - 125 - viz_speed_w/4, header_h2/2);
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, nvgRGBA(255,230,70,(scene.blinker_blinkingrate<=100 && scene.blinker_blinkingrate>=50)?140:0));
    nvgFill(s->vg);
    nvgBeginPath(s->vg);
    nvgMoveTo(s->vg, viz_speed_x - 250, header_h2/4);
    nvgLineTo(s->vg, viz_speed_x - 250 - viz_speed_w/4, header_h2/4);
    nvgLineTo(s->vg, viz_speed_x - 250 - viz_speed_w/2, header_h2/2);
    nvgLineTo(s->vg, viz_speed_x - 250 - viz_speed_w/4, header_h2/4 + header_h2/2);
    nvgLineTo(s->vg, viz_speed_x - 250, header_h2/4 + header_h2/2);
    nvgLineTo(s->vg, viz_speed_x - 250 - viz_speed_w/4, header_h2/2);
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, nvgRGBA(255,230,70,(scene.blinker_blinkingrate<=80 && scene.blinker_blinkingrate>=50)?210:0));
    nvgFill(s->vg);
  }
  if (scene.rightBlinker && scene.comma_stock_ui != 1) {
    nvgBeginPath(s->vg);
    nvgMoveTo(s->vg, viz_speed_x + viz_speed_w, header_h2/4);
    nvgLineTo(s->vg, viz_speed_x + viz_speed_w + viz_speed_w/4, header_h2/4);
    nvgLineTo(s->vg, viz_speed_x + viz_speed_w + viz_speed_w/2, header_h2/2);
    nvgLineTo(s->vg, viz_speed_x + viz_speed_w + viz_speed_w/4, header_h2/4 + header_h2/2);
    nvgLineTo(s->vg, viz_speed_x + viz_speed_w, header_h2/4 + header_h2/2);
    nvgLineTo(s->vg, viz_speed_x + viz_speed_w + viz_speed_w/4, header_h2/2);
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, nvgRGBA(255,230,70,(scene.blinker_blinkingrate<=120 && scene.blinker_blinkingrate>=50)?70:0));
    nvgFill(s->vg);
    nvgBeginPath(s->vg);
    nvgMoveTo(s->vg, viz_speed_x + viz_speed_w + 125, header_h2/4);
    nvgLineTo(s->vg, viz_speed_x + viz_speed_w + 125 + viz_speed_w/4, header_h2/4);
    nvgLineTo(s->vg, viz_speed_x + viz_speed_w + 125 + viz_speed_w/2, header_h2/2);
    nvgLineTo(s->vg, viz_speed_x + viz_speed_w + 125 + viz_speed_w/4, header_h2/4 + header_h2/2);
    nvgLineTo(s->vg, viz_speed_x + viz_speed_w + 125, header_h2/4 + header_h2/2);
    nvgLineTo(s->vg, viz_speed_x + viz_speed_w + 125 + viz_speed_w/4, header_h2/2);
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, nvgRGBA(255,230,70,(scene.blinker_blinkingrate<=100 && scene.blinker_blinkingrate>=50)?140:0));
    nvgFill(s->vg);
    nvgBeginPath(s->vg);
    nvgMoveTo(s->vg, viz_speed_x + viz_speed_w + 250, header_h2/4);
    nvgLineTo(s->vg, viz_speed_x + viz_speed_w + 250 + viz_speed_w/4, header_h2/4);
    nvgLineTo(s->vg, viz_speed_x + viz_speed_w + 250 + viz_speed_w/2, header_h2/2);
    nvgLineTo(s->vg, viz_speed_x + viz_speed_w + 250 + viz_speed_w/4, header_h2/4 + header_h2/2);
    nvgLineTo(s->vg, viz_speed_x + viz_speed_w + 250, header_h2/4 + header_h2/2);
    nvgLineTo(s->vg, viz_speed_x + viz_speed_w + 250 + viz_speed_w/4, header_h2/2);
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, nvgRGBA(255,230,70,(scene.blinker_blinkingrate<=80 && scene.blinker_blinkingrate>=50)?210:0));
    nvgFill(s->vg);
    }
  if (scene.leftBlinker || scene.rightBlinker) {
    scene.blinker_blinkingrate -= 5;
    if(scene.blinker_blinkingrate < 0) scene.blinker_blinkingrate = 120;
  }

  NVGcolor val_color = COLOR_WHITE;

  float act_accel = (!scene.longitudinal_control)?scene.a_req_value:scene.accel;
  float gas_opacity = act_accel*255>255?255:act_accel*255;
  float brake_opacity = abs(act_accel*175)>255?255:abs(act_accel*175);

  if (scene.brakePress && scene.comma_stock_ui != 1) {
  	val_color = COLOR_RED;
  } else if (scene.brakeLights && speed_str == "0" && scene.comma_stock_ui != 1) {
  	val_color = nvgRGBA(201, 34, 49, 100);
  } else if (scene.gasPress && scene.comma_stock_ui != 1) {
    val_color = nvgRGBA(0, 240, 0, 255);
  } else if (act_accel < 0 && act_accel > -5.0 && scene.comma_stock_ui != 1) {
    val_color = nvgRGBA((255-int(abs(act_accel*8))), (255-int(brake_opacity)), (255-int(brake_opacity)), 255);
  } else if (act_accel > 0 && act_accel < 3.0 && scene.comma_stock_ui != 1) {
    val_color = nvgRGBA((255-int(gas_opacity)), (255-int((act_accel*10))), (255-int(gas_opacity)), 255);
  }
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  ui_draw_text(s, s->fb_w/2, 210+(s->scene.animated_rpm?35:0), speed_str.c_str(), 96 * 2.5, val_color, "sans-bold");
  if (!s->scene.animated_rpm) {
    ui_draw_text(s, s->fb_w/2, 290, s->scene.is_metric ? "km/h" : "mph", 36 * 2.5, scene.brakeLights?nvgRGBA(201, 34, 49, 100):COLOR_WHITE_ALPHA(200), "sans-regular");
  }
}

static void ui_draw_vision_event(UIState *s) {
  const int viz_event_w = 220;
  const int viz_event_x = s->fb_w - (viz_event_w + bdr_s);
  const int viz_event_y = bdr_s;

  if ((s->scene.liveNaviData.opkrroadsign == 107 || s->scene.liveNaviData.opkrspeedsign == 124 || s->scene.liveENaviData.eopkrsafetysign == 22) && s->scene.comma_stock_ui != 1) {
    ui_draw_image(s, {s->fb_w/2 - 500/2, s->fb_h/2 - 450/2, 500, 450}, "speed_bump", 0.35f);
  }

  // draw steering wheel
  const int bg_wheel_size = 90;
  const int bg_wheel_x = viz_event_x + (viz_event_w-bg_wheel_size);
  const int bg_wheel_y = viz_event_y + (bg_wheel_size/2);
  const QColor &color = bg_colors[s->status];
  NVGcolor nvg_color = nvgRGBA(color.red(), color.green(), color.blue(), color.alpha());
  if (s->scene.controls_state.getEnabled() || s->scene.forceGearD || s->scene.comma_stock_ui == 1) {
    float angleSteers = s->scene.car_state.getSteeringAngleDeg();
    if (s->scene.controlAllowed) {
      ui_draw_circle_image_rotation(s, bg_wheel_x, bg_wheel_y+20, bg_wheel_size, "wheel", nvg_color, 1.0f, angleSteers);
    } else {
      ui_draw_circle_image_rotation(s, bg_wheel_x, bg_wheel_y+20, bg_wheel_size, "wheel", nvgRGBA(0x17, 0x33, 0x49, 0xc8), 1.0f, angleSteers);
    }
  } else {
    if (s->scene.comma_stock_ui != 1) ui_draw_gear(s);
  }
  if (s->scene.comma_stock_ui != 1) ui_draw_debug(s);
}

static void ui_draw_vision_face(UIState *s) {
  const int radius = 85;
  const int center_x = radius + bdr_s;
  const int center_y = s->fb_h - footer_h + ((footer_h - radius) / 2);
  ui_draw_circle_image(s, center_x, !s->scene.animated_rpm?center_y:(center_y-60), radius, "driver_face", s->scene.dm_active);
}

//BB START: functions added for the display of various items
static int bb_ui_draw_measure(UIState *s, const char* bb_value, const char* bb_uom, const char* bb_label,
    int bb_x, int bb_y, int bb_uom_dx,
    NVGcolor bb_valueColor, NVGcolor bb_labelColor, NVGcolor bb_uomColor,
    int bb_valueFontSize, int bb_labelFontSize, int bb_uomFontSize, int custom)  {
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  if (custom == 1) {
    int num_value = atoi(bb_value);
    nvgBeginPath(s->vg);
    nvgMoveTo(s->vg, bb_x-80, bb_y+90);
    nvgLineTo(s->vg, bb_x+80, bb_y+32);
    nvgLineTo(s->vg, bb_x+80, bb_y+90);
    nvgLineTo(s->vg, bb_x-80, bb_y+90);
    nvgClosePath(s->vg);
    nvgStrokeWidth(s->vg, 1);
    nvgStrokeColor(s->vg, COLOR_WHITE_ALPHA(200));
    nvgStroke(s->vg);

    nvgBeginPath(s->vg);
    nvgMoveTo(s->vg, bb_x-80, bb_y+90);
    nvgLineTo(s->vg, bb_x-80+(fmin(num_value, 3200)*0.05), bb_y+90-(fmin(num_value, 3200)*0.018125));
    nvgLineTo(s->vg, bb_x-80+(fmin(num_value, 3200)*0.05), bb_y+90);
    nvgLineTo(s->vg, bb_x-80, bb_y+90);
    nvgClosePath(s->vg);
    NVGpaint rpm_gradient = nvgLinearGradient(s->vg, bb_x-80, bb_y+90, bb_x+80, bb_y+32, COLOR_GREEN_ALPHA(80), COLOR_RED_ALPHA(255));
    nvgFillPaint(s->vg, rpm_gradient);
    nvgFill(s->vg);

    //print value
    int dx = 0;
    if (strlen(bb_uom) > 0) {
      dx = (int)(bb_uomFontSize*2.5/2);
    }
    //print value
    // nvgFontFace(s->vg, "sans-semibold");
    // nvgFontSize(s->vg, bb_valueFontSize*0.8);
    // nvgFillColor(s->vg, bb_valueColor);
    // nvgText(s->vg, bb_x-dx/2-20, bb_y+ (int)(bb_valueFontSize*2.5)+5-20, bb_value, NULL);
    //print label
    nvgFontFace(s->vg, "sans-regular");
    nvgFontSize(s->vg, bb_labelFontSize*2.5);
    nvgFillColor(s->vg, bb_labelColor);
    nvgText(s->vg, bb_x, bb_y + (int)(bb_valueFontSize*2.0)+5, bb_label, NULL);
    //print uom
    if (strlen(bb_uom) > 0) {
        nvgSave(s->vg);
      int rx =bb_x + bb_uom_dx + bb_valueFontSize -3;
      int ry = bb_y + (int)(bb_valueFontSize*2.0/2) + (int)(bb_labelFontSize*2.0)+5+25;
      nvgTranslate(s->vg,rx,ry);
      nvgRotate(s->vg, -1.5708); //-90deg in radians
      nvgFontFace(s->vg, "sans-regular");
      nvgFontSize(s->vg, (int)(bb_uomFontSize*2.5));
      nvgFillColor(s->vg, bb_uomColor);
      nvgText(s->vg, 0, 0, bb_uom, NULL);
      nvgRestore(s->vg);
    }
  } else {
    int dx = 0;
    if (strlen(bb_uom) > 0) {
      dx = (int)(bb_uomFontSize*2.5/2);
    }
    //print label
    nvgFontFace(s->vg, "sans-regular");
    nvgFontSize(s->vg, bb_labelFontSize*2.5);
    nvgFillColor(s->vg, bb_labelColor);
    nvgText(s->vg, bb_x, bb_y + (int)(bb_valueFontSize*2.0)+5, bb_label, NULL);
    //print value
    nvgFontFace(s->vg, "sans-semibold");
    nvgFontSize(s->vg, bb_valueFontSize*2.5);
    nvgFillColor(s->vg, bb_valueColor);
    nvgText(s->vg, bb_x-dx/2, bb_y+ (int)(bb_valueFontSize*2.0)+5 + (int)(bb_labelFontSize*2.0)+25, bb_value, NULL);
    //print uom
    if (strlen(bb_uom) > 0) {
        nvgSave(s->vg);
      int rx =bb_x + bb_uom_dx + bb_valueFontSize -3;
      int ry = bb_y + (int)(bb_valueFontSize*2.0/2) + (int)(bb_labelFontSize*2.0)+5+25;
      nvgTranslate(s->vg,rx,ry);
      nvgRotate(s->vg, -1.5708); //-90deg in radians
      nvgFontFace(s->vg, "sans-regular");
      nvgFontSize(s->vg, (int)(bb_uomFontSize*2.5));
      nvgFillColor(s->vg, bb_uomColor);
      if (custom == 2) {
        nvgFontFace(s->vg, "KaiGenGothicKR-Medium");
        nvgFontSize(s->vg, (int)(bb_uomFontSize*1.5));
      }
      nvgText(s->vg, 0, 0, bb_uom, NULL);
      nvgRestore(s->vg);
    }
  }
  return (int)((bb_valueFontSize + bb_labelFontSize)*2) + 5;
}

static void bb_ui_draw_measures_right(UIState *s, int bb_x, int bb_y, int bb_w ) {
  UIScene &scene = s->scene;
  int bb_rx = bb_x + (int)(bb_w/2);
  int bb_ry = bb_y - 20;
  NVGcolor lab_color = COLOR_WHITE_ALPHA(200);
  NVGcolor uom_color = COLOR_WHITE_ALPHA(200);
  int value_fontSize=30;
  int label_fontSize=15;
  int uom_fontSize = 15;
  int bb_uom_dx =  (int)(bb_w /2 - uom_fontSize*2.5) ;
  //CPU TEMP
  if (true) {
    //char val_str[16];
    char uom_str[6];
    std::string cpu_temp_val = std::to_string(int(scene.cpuTemp)) + "°C";
    NVGcolor val_color = COLOR_WHITE_ALPHA(200);
    if(scene.cpuTemp > 75) {
      val_color = nvgRGBA(255, 188, 3, 200);
    }
    if(scene.cpuTemp > 85) {
      val_color = nvgRGBA(255, 0, 0, 200);
    }
    //snprintf(val_str, sizeof(val_str), "%.0fC", (round(scene.cpuTemp)));
    if (!scene.batt_less) {
      snprintf(uom_str, sizeof(uom_str), "%d%%", (scene.cpuPerc));
    } else {
      snprintf(uom_str, sizeof(uom_str), "%.0f°C", (scene.ambientTemp));
    }
    bb_ry +=bb_ui_draw_measure(s, cpu_temp_val.c_str(), uom_str, "CPU TEMP",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize, 0);
  }
  //CPU LOAD
  if (scene.batt_less) {
    //char val_str[16];
    char uom_str[6];
    std::string cpu_usage_val = std::to_string(int(scene.cpuPerc)) + "%";
    NVGcolor val_color = COLOR_GREEN_ALPHA(200);
    if(scene.cpuPerc > 60) {
      val_color = nvgRGBA(255, 188, 3, 200);
    }
    if(scene.cpuPerc > 80) {
      val_color = nvgRGBA(255, 0, 0, 200);
    }
    // temp is alway in C * 1000
    //snprintf(val_str, sizeof(val_str), "%.0fC", batteryTemp);
    snprintf(uom_str, sizeof(uom_str), "%d", (scene.fanSpeed)/1000);
    bb_ry +=bb_ui_draw_measure(s, cpu_usage_val.c_str(), uom_str, "CPU LOAD",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize, 0);
  }
  //BAT STAT
  if (!scene.batt_less) {
    //char val_str[16];
    std::string bat_temp_val = std::to_string(int(scene.batTemp)) + "°C";
    std::string bat_level_val = "";
    NVGcolor val_color = COLOR_WHITE_ALPHA(200);
    if (scene.batTemp > 40) {
      val_color = nvgRGBA(255, 188, 3, 200);
    }
    if (scene.batTemp > 50) {
      val_color = nvgRGBA(255, 0, 0, 200);
    }
    if (scene.deviceState.getBatteryStatus() == "Charging") {
      bat_level_val = std::to_string(int(scene.batPercent)) + "%++";
    } else {
      bat_level_val = std::to_string(int(scene.batPercent)) + "%--";
    }
    NVGcolor uom_color2 = COLOR_WHITE_ALPHA(200);
    if ((scene.fanSpeed/1000) > 64) {
      uom_color2 = nvgRGBA(255, 0, 0, 200);
    } else if ((scene.fanSpeed/1000) > 31) {
      uom_color2 = nvgRGBA(255, 188, 3, 200);
    } else if ((scene.fanSpeed/1000) > 15) {
      uom_color2 = nvgRGBA(0, 255, 0, 200);
    }
    // temp is alway in C * 1000
    //snprintf(val_str, sizeof(val_str), "%.0fC", batteryTemp);
    bb_ry +=bb_ui_draw_measure(s, bat_temp_val.c_str(), bat_level_val.c_str(), "BAT STAT",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color2,
        value_fontSize, label_fontSize, uom_fontSize, 0);
  }
  //add Ublox GPS accuracy
  if (scene.gpsAccuracyUblox != 0.00) {
    char val_str[16];
    char uom_str[6];
    NVGcolor val_color = COLOR_WHITE_ALPHA(200);
    //show red/orange if gps accuracy is low
      if(scene.gpsAccuracyUblox > 0.85) {
         val_color = COLOR_ORANGE_ALPHA(200);
      }
      if(scene.gpsAccuracyUblox > 1.3) {
         val_color = COLOR_RED_ALPHA(200);
      }
    // gps accuracy is always in meters
    if(scene.gpsAccuracyUblox > 99 || scene.gpsAccuracyUblox == 0) {
       snprintf(val_str, sizeof(val_str), "None");
    }else if(scene.gpsAccuracyUblox > 9.99) {
      snprintf(val_str, sizeof(val_str), "%.1f", (scene.gpsAccuracyUblox));
    }
    else {
      snprintf(val_str, sizeof(val_str), "%.2f", (scene.gpsAccuracyUblox));
    }
    snprintf(uom_str, sizeof(uom_str), "%d", (scene.satelliteCount));
    bb_ry +=bb_ui_draw_measure(s, val_str, uom_str, "GPS PREC",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize, 0);
  }
  //add altitude
  if (scene.gpsAccuracyUblox != 0.00) {
    char val_str[16];
    char uom_str[6];
    NVGcolor val_color = COLOR_WHITE_ALPHA(200);
    snprintf(val_str, sizeof(val_str), "%.0f", (scene.altitudeUblox));
    snprintf(uom_str, sizeof(uom_str), "m");
    bb_ry +=bb_ui_draw_measure(s, val_str, uom_str, "ALTITUDE",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize, 0);
  }

  //finally draw the frame
  nvgBeginPath(s->vg);
  nvgRoundedRect(s->vg, bb_x, bb_y, bb_w, bb_ry - bb_y + 45, 20);
  nvgStrokeColor(s->vg, COLOR_WHITE_ALPHA(80));
  nvgStrokeWidth(s->vg, 6);
  nvgStroke(s->vg);

// TPMS code added from OPKR
  char tpmsFl[64];
  char tpmsFr[64];
  char tpmsRl[64];
  char tpmsRr[64];
  int viz_tpms_w = 180;
  int viz_tpms_h = 145;
  int viz_tpms_x = s->fb_w - viz_tpms_w - bdr_s;
  int viz_tpms_y = bb_ry + 60;
  float maxv = 0;
  float minv = 300;
  const Rect rect = {viz_tpms_x, viz_tpms_y, viz_tpms_w, viz_tpms_h};
  int font_size;

  if (maxv < scene.tpmsPressureFl) {maxv = scene.tpmsPressureFl;}
  if (maxv < scene.tpmsPressureFr) {maxv = scene.tpmsPressureFr;}
  if (maxv < scene.tpmsPressureRl) {maxv = scene.tpmsPressureRl;}
  if (maxv < scene.tpmsPressureRr) {maxv = scene.tpmsPressureRr;}
  if (minv > scene.tpmsPressureFl) {minv = scene.tpmsPressureFl;}
  if (minv > scene.tpmsPressureFr) {minv = scene.tpmsPressureFr;}
  if (minv > scene.tpmsPressureRl) {minv = scene.tpmsPressureRl;}
  if (minv > scene.tpmsPressureRr) {minv = scene.tpmsPressureRr;}

  // Draw Border
  ui_draw_rect(s->vg, rect, COLOR_WHITE_ALPHA(100), 7, 20.);
  // Draw Background
  if (((maxv - minv) > 3 && scene.tpmsUnit != 2) || ((maxv - minv) > 0.2 && scene.tpmsUnit == 2)) {
    ui_fill_rect(s->vg, rect, COLOR_RED_ALPHA(80), 20);
  }

  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  const int pos_x = viz_tpms_x + (viz_tpms_w / 2);
  const int pos_y = viz_tpms_y + 40;
  if (scene.tpmsUnit != 0) {
    ui_draw_text(s, pos_x, pos_y, (scene.tpmsUnit == 2) ? "TPMS(bar)" : "TPMS(psi)", 45, COLOR_WHITE_ALPHA(180), "sans-regular");
    snprintf(tpmsFl, sizeof(tpmsFl), "%.1f", scene.tpmsPressureFl);
    snprintf(tpmsFr, sizeof(tpmsFr), "%.1f", scene.tpmsPressureFr);
    snprintf(tpmsRl, sizeof(tpmsRl), "%.1f", scene.tpmsPressureRl);
    snprintf(tpmsRr, sizeof(tpmsRr), "%.1f", scene.tpmsPressureRr);
    font_size = (scene.tpmsUnit == 2) ? 60 : 55;
  } else {
    ui_draw_text(s, pos_x, pos_y, "TPMS(psi)", 45, COLOR_WHITE_ALPHA(180), "sans-regular");
    snprintf(tpmsFl, sizeof(tpmsFl), "%.0f", scene.tpmsPressureFl);
    snprintf(tpmsFr, sizeof(tpmsFr), "%.0f", scene.tpmsPressureFr);
    snprintf(tpmsRl, sizeof(tpmsRl), "%.0f", scene.tpmsPressureRl);
    snprintf(tpmsRr, sizeof(tpmsRr), "%.0f", scene.tpmsPressureRr);
    font_size = 65;
  }
  if ((scene.tpmsPressureFl < 32 && scene.tpmsUnit != 2) || (scene.tpmsPressureFl < 2.2 && scene.tpmsUnit == 2)) {
    ui_draw_text(s, pos_x-45, pos_y+45, tpmsFl, font_size, COLOR_YELLOW_ALPHA(scene.tpms_blinkingrate>=60?200:0), "sans-bold");
  } else if (scene.tpmsPressureFl > 50) {
    ui_draw_text(s, pos_x-45, pos_y+45, "N/A", 50, COLOR_WHITE_ALPHA(200), "sans-semibold");
  } else if ((scene.tpmsPressureFl > 45 && scene.tpmsUnit != 2) || (scene.tpmsPressureFl > 2.8 && scene.tpmsUnit == 2)) {
    ui_draw_text(s, pos_x-45, pos_y+45, tpmsFl, font_size, COLOR_RED, "sans-semibold");
  } else {
    ui_draw_text(s, pos_x-45, pos_y+45, tpmsFl, font_size, COLOR_GREEN_ALPHA(200), "sans-semibold");
  }
  if ((scene.tpmsPressureFr < 32 && scene.tpmsUnit != 2) || (scene.tpmsPressureFr < 2.2 && scene.tpmsUnit == 2)) {
    ui_draw_text(s, pos_x+45, pos_y+45, tpmsFr, font_size, COLOR_YELLOW_ALPHA(scene.tpms_blinkingrate>=60?200:0), "sans-bold");
  } else if (scene.tpmsPressureFr > 50) {
    ui_draw_text(s, pos_x+45, pos_y+45, "N/A", 50, COLOR_WHITE_ALPHA(200), "sans-semibold");
  } else if ((scene.tpmsPressureFr > 45 && scene.tpmsUnit != 2) || (scene.tpmsPressureFr > 2.8 && scene.tpmsUnit == 2)) {
    ui_draw_text(s, pos_x+45, pos_y+45, tpmsFr, font_size, COLOR_RED, "sans-semibold");
  } else {
    ui_draw_text(s, pos_x+45, pos_y+45, tpmsFr, font_size, COLOR_GREEN_ALPHA(200), "sans-semibold");
  }
  if ((scene.tpmsPressureRl < 32 && scene.tpmsUnit != 2) || (scene.tpmsPressureRl < 2.2 && scene.tpmsUnit == 2)) {
    ui_draw_text(s, pos_x-45, pos_y+90, tpmsRl, font_size, COLOR_YELLOW_ALPHA(scene.tpms_blinkingrate>=60?200:0), "sans-bold");
  } else if (scene.tpmsPressureRl > 50) {
    ui_draw_text(s, pos_x-45, pos_y+90, "N/A", 50, COLOR_WHITE_ALPHA(200), "sans-semibold");
  } else if ((scene.tpmsPressureRl > 45 && scene.tpmsUnit != 2) || (scene.tpmsPressureRl > 2.8 && scene.tpmsUnit == 2)) {
    ui_draw_text(s, pos_x-45, pos_y+90, tpmsFr, font_size, COLOR_RED, "sans-semibold");
  } else {
    ui_draw_text(s, pos_x-45, pos_y+90, tpmsRl, font_size, COLOR_GREEN_ALPHA(200), "sans-semibold");
  }
  if ((scene.tpmsPressureRr < 32 && scene.tpmsUnit != 2) || (scene.tpmsPressureRr < 2.2 && scene.tpmsUnit == 2)) {
    ui_draw_text(s, pos_x+45, pos_y+90, tpmsRr, font_size, COLOR_YELLOW_ALPHA(scene.tpms_blinkingrate>=60?200:0), "sans-bold");
  } else if (scene.tpmsPressureRr > 50) {
    ui_draw_text(s, pos_x+45, pos_y+90, "N/A", 50, COLOR_WHITE_ALPHA(200), "sans-semibold");
  } else if ((scene.tpmsPressureRr > 45 && scene.tpmsUnit != 2) || (scene.tpmsPressureRr > 2.8 && scene.tpmsUnit == 2)) {
    ui_draw_text(s, pos_x+45, pos_y+90, tpmsFr, font_size, COLOR_RED, "sans-semibold");
  } else {
    ui_draw_text(s, pos_x+45, pos_y+90, tpmsRr, font_size, COLOR_GREEN_ALPHA(200), "sans-semibold");
  }
  if (((scene.tpmsPressureFl < 32 || scene.tpmsPressureFr < 32 || scene.tpmsPressureRl < 32 || scene.tpmsPressureRr < 32) && scene.tpmsUnit != 2) || ((scene.tpmsPressureFl < 2.2 || scene.tpmsPressureFr < 2.2 || scene.tpmsPressureRl < 2.2 || scene.tpmsPressureRr < 2.2) && scene.tpmsUnit == 2)){
    scene.tpms_blinkingrate -= 5;
    if(scene.tpms_blinkingrate < 0) scene.tpms_blinkingrate = 120;
  }
}

static void bb_ui_draw_measures_left(UIState *s, int bb_x, int bb_y, int bb_w ) {
  const UIScene &scene = s->scene;
  int bb_rx = bb_x + (int)(bb_w/2);
  int bb_ry = bb_y - 20;
  NVGcolor lab_color = COLOR_WHITE_ALPHA(200);
  NVGcolor uom_color = COLOR_WHITE_ALPHA(200);
  int value_fontSize=30;
  int label_fontSize=15;
  int uom_fontSize = 15;
  int bb_uom_dx =  (int)(bb_w /2 - uom_fontSize*2.5);
  auto lead_one = (*s->sm)["radarState"].getRadarState().getLeadOne();

  //add visual radar relative distance
  if (true) {
    char val_str[16];
    char uom_str[6];
    NVGcolor val_color = COLOR_WHITE_ALPHA(200);
    if (lead_one.getStatus()) {
      //show RED if less than 5 meters
      //show orange if less than 15 meters
      if((int)(lead_one.getDRel()) < 15) {
        val_color = COLOR_ORANGE_ALPHA(200);
      }
      if((int)(lead_one.getDRel()) < 5) {
        val_color = COLOR_RED_ALPHA(200);
      }
      // lead car relative distance is always in meters
      if((float)(lead_one.getDRel()) < 10) {
        snprintf(val_str, sizeof(val_str), "%.1f", (float)(lead_one.getDRel()));
      } else {
        snprintf(val_str, sizeof(val_str), "%d", (int)(lead_one.getDRel()));
      }

    } else {
       snprintf(val_str, sizeof(val_str), "-");
    }
    snprintf(uom_str, sizeof(uom_str), "m");
    bb_ry +=bb_ui_draw_measure(s, val_str, uom_str, "REL DIST",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize, 0);
  }
  //add visual radar relative speed
  if (true) {
    char val_str[16];
    char uom_str[6];
    NVGcolor val_color = COLOR_WHITE_ALPHA(200);
    if (lead_one.getStatus()) {
      //show Orange if negative speed (approaching)
      //show Orange if negative speed faster than 5mph (approaching fast)
      if((int)(lead_one.getVRel()) < 0) {
        val_color = nvgRGBA(255, 188, 3, 200);
      }
      if((int)(lead_one.getVRel()) < -5) {
        val_color = nvgRGBA(255, 0, 0, 200);
      }
      // lead car relative speed is always in meters
      snprintf(val_str, sizeof(val_str), "%d", (int)(round(lead_one.getVRel() * (scene.is_metric ? 3.6 : 2.2369363))));
    } else {
       snprintf(val_str, sizeof(val_str), "-");
    }
    if (scene.is_metric) {
      snprintf(uom_str, sizeof(uom_str), "km/h");;
    } else {
      snprintf(uom_str, sizeof(uom_str), "mi/h");
    }
    bb_ry +=bb_ui_draw_measure(s, val_str, uom_str, "REL SPD",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize, 0);
  }
  //add steering angle
  if (true) {
    char val_str[16];
    char uom_str[6];
    //std::string angle_val = std::to_string(int(scene.angleSteers*10)/10) + "°";
    NVGcolor val_color = COLOR_GREEN_ALPHA(200);
    //show Orange if more than 30 degrees
    //show red if  more than 50 degrees
    if(((int)(scene.angleSteers) < -90) || ((int)(scene.angleSteers) > 90)) {
      val_color = COLOR_ORANGE_ALPHA(200);
    }
    if(((int)(scene.angleSteers) < -180) || ((int)(scene.angleSteers) > 180)) {
      val_color = COLOR_RED_ALPHA(200);
    }
    // steering is in degrees
    snprintf(val_str, sizeof(val_str), "%.1f",(scene.angleSteers));
    snprintf(uom_str, sizeof(uom_str), "%.1f",(scene.desired_angle_steers));

    bb_ry +=bb_ui_draw_measure(s, val_str, uom_str, "STR ANG",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize, 0);
  }

  //add steerratio from lateralplan
  if (true) {
    char val_str[16];
    char uom_str[6];
    NVGcolor val_color = COLOR_WHITE_ALPHA(200);
    if (scene.controls_state.getEnabled()) {
      snprintf(val_str, sizeof(val_str), "%.2f",(scene.steerRatio));
    } else {
       snprintf(val_str, sizeof(val_str), "-");
    }
    snprintf(uom_str, sizeof(uom_str), "");
    bb_ry +=bb_ui_draw_measure(s, val_str, uom_str, "SteerRatio",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize, 0);
  }

  // //engine rpm
  // if (scene.engine_rpm < 9998) {
  //   //char val_str[16];
  //   char uom_str[6];
  //   std::string engine_rpm_val = std::to_string(int(scene.engine_rpm));
  //   NVGcolor val_color = COLOR_WHITE_ALPHA(200);
  //   if(scene.engine_rpm > 2500) {
  //     val_color = nvgRGBA(255, 188, 3, 200);
  //   }
  //   if(scene.engine_rpm > 3500) {
  //     val_color = nvgRGBA(255, 0, 0, 200);
  //   }
  //   snprintf(uom_str, sizeof(uom_str), "%d", scene.gear_step);
  //   bb_ry +=bb_ui_draw_measure(s, engine_rpm_val.c_str(), uom_str, "ENG RPM",
  //       bb_rx, bb_ry, bb_uom_dx,
  //       val_color, lab_color, uom_color,
  //       value_fontSize, label_fontSize, uom_fontSize, 1);
  // }

  // //cruise gap
  // if (scene.longitudinal_control && scene.radar_long_helper < 2) {
  //   char val_str[16];
  //   char uom_str[6];
  //   NVGcolor val_color = COLOR_WHITE_ALPHA(200);
  //   if (scene.controls_state.getEnabled()) {
  //     if (scene.cruise_gap == scene.dynamic_tr_mode) {
  //       snprintf(val_str, sizeof(val_str), "AUT");
  //       snprintf(uom_str, sizeof(uom_str), "%.2f",(scene.dynamic_tr_value));
  //     } else {
  //       snprintf(val_str, sizeof(val_str), "%d",(scene.cruise_gap));
  //       snprintf(uom_str, sizeof(uom_str), "S");
  //     }
  //   } else {
  //     snprintf(val_str, sizeof(val_str), "-");
  //     snprintf(uom_str, sizeof(uom_str), "");
  //   }
  //   bb_ry +=bb_ui_draw_measure(s, val_str, uom_str, "Cruise Gap",
  //       bb_rx, bb_ry, bb_uom_dx,
  //       val_color, lab_color, uom_color,
  //       value_fontSize, label_fontSize, uom_fontSize, 0);
  // }

  if (0 < scene.gear_step && scene.gear_step < 9) {
    //char val_str[16];
    //char uom_str[6];
    std::string main_val = "";
    if (scene.charge_meter > 0) {
      main_val = std::to_string(int(scene.charge_meter)) + "%";  
    } else {
      main_val = "S " + std::to_string(int(scene.gear_step));
    }
    std::string gap = "";
    NVGcolor val_color = COLOR_YELLOW_ALPHA(230);
    NVGcolor uom_color2 = COLOR_WHITE_ALPHA(200);
    if (scene.cruise_gap == 1) {
      if (scene.gap_by_speed_on) {
        uom_color2 = nvgRGBA(0, 180, 255, 220);
      } else {
        uom_color2 = COLOR_RED_ALPHA(240);
      }
      gap = "■";
    } else if (scene.cruise_gap == 2) {
      if (scene.gap_by_speed_on) {
        uom_color2 = nvgRGBA(0, 180, 255, 220);
      } else {
        uom_color2 = COLOR_OCHRE_ALPHA(220);
      }
      gap = "■■";
    } else if (scene.cruise_gap == 3) {
      if (scene.gap_by_speed_on) {
        uom_color2 = nvgRGBA(0, 180, 255, 220);
      } else {
        uom_color2 = COLOR_GREEN_ALPHA(200);
      }
      gap = "■■■";
    } else {
      if (scene.gap_by_speed_on) {
        uom_color2 = nvgRGBA(0, 180, 255, 220);
      } else {
        uom_color2 = COLOR_WHITE_ALPHA(200);
      }
      gap = "■■■■";
    }
    bb_ry +=bb_ui_draw_measure(s, main_val.c_str(), gap.c_str(), scene.charge_meter>0?"MAIN BAT":"GEAR",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color2,
        value_fontSize, label_fontSize, uom_fontSize, 2);
  }

  //finally draw the frame
  nvgBeginPath(s->vg);
  nvgRoundedRect(s->vg, bb_x, bb_y, bb_w, bb_ry - bb_y + 45, 20);
  nvgStrokeColor(s->vg, COLOR_WHITE_ALPHA(80));
  nvgStrokeWidth(s->vg, 6);
  nvgStroke(s->vg);
}

//BB END: functions added for the display of various items

static void bb_ui_draw_UI(UIState *s) {
  const int bb_dml_w = 180;
  const int bb_dml_x = bdr_s;
  const int bb_dml_y = bdr_s + 220;

  const int bb_dmr_w = 180;
  const int bb_dmr_x = s->fb_w - bb_dmr_w - bdr_s;
  const int bb_dmr_y = bdr_s + 220;

  bb_ui_draw_measures_left(s, bb_dml_x, bb_dml_y, bb_dml_w);
  bb_ui_draw_measures_right(s, bb_dmr_x, bb_dmr_y-20, bb_dmr_w);
}

// show speedlimit value
static void draw_safetysign(UIState *s) {
  const int diameter = 185;
  const int diameter2 = 170;
  const int diameter3 = 202;
  int s_center_x = bdr_s + 305 + (s->scene.display_maxspeed_time>0 ? 184 : 0);
  const int s_center_y = bdr_s + 100;
  
  int d_center_x = s_center_x;
  const int d_center_y = s_center_y + 155;
  const int d_width = 220;
  const int d_height = 70;
  int opacity = 0;

  const Rect rect_s = {s_center_x - diameter/2, s_center_y - diameter/2, diameter, diameter};
  const Rect rect_si = {s_center_x - diameter2/2, s_center_y - diameter2/2, diameter2, diameter2};
  const Rect rect_so = {s_center_x - diameter3/2, s_center_y - diameter3/2, diameter3, diameter3};
  const Rect rect_d = {d_center_x - d_width/2, d_center_y - d_height/2, d_width, d_height};
  char safetySpeed[16];
  char safetyDist[32];
  int safety_speed = s->scene.limitSpeedCamera;
  float safety_dist = s->scene.limitSpeedCameraDist;
  float maxspeed = round(s->scene.controls_state.getVCruise());
  //int safety_speed = s->scene.liveNaviData.opkrspeedlimit;
  //float safety_dist = s->scene.liveNaviData.opkrspeedlimitdist;
  int sl_opacity = 0;
  if (s->scene.sl_decel_off) {
    sl_opacity = 3;
  } else if (s->scene.osm_off_spdlimit) {
    sl_opacity = 2;
  } else {
    sl_opacity = 1;
  }

  snprintf(safetySpeed, sizeof(safetySpeed), "%d", safety_speed);
  if (maxspeed != 255.0) {
    if (s->scene.is_metric) {
      if (safety_dist < 1000) {
        snprintf(safetyDist, sizeof(safetyDist), "%.0fm", safety_dist);
      } else if (safety_dist < 10000) {
        snprintf(safetyDist, sizeof(safetyDist), "%.2fkm", safety_dist/1000);
      } else {
        snprintf(safetyDist, sizeof(safetyDist), "%.1fkm", safety_dist/1000);
      }
      opacity = safety_dist>600 ? 0 : (600 - safety_dist) * 0.425;
    } else {
      if (safety_dist < 1000) {
        snprintf(safetyDist, sizeof(safetyDist), "%.0fyd", safety_dist);
      } else if (safety_dist < 10000) {
        snprintf(safetyDist, sizeof(safetyDist), "%.2fmi", safety_dist/1000);
      } else {
        snprintf(safetyDist, sizeof(safetyDist), "%.1fmi", safety_dist/1000);
      }
      opacity = safety_dist>600 ? 0 : (600 - safety_dist) * 0.425;
    }
  }

  if (safety_speed > 21) {
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    if (s->scene.speedlimit_signtype) {
      ui_fill_rect(s->vg, rect_si, COLOR_WHITE_ALPHA(200/sl_opacity), 16.);
      ui_draw_rect(s->vg, rect_s, COLOR_BLACK_ALPHA(200/sl_opacity), 9, 17.);
      ui_draw_rect(s->vg, rect_so, COLOR_WHITE_ALPHA(200/sl_opacity), 6, 20.);
      ui_draw_text(s, rect_s.centerX(), rect_s.centerY()-55, "SPEED", 55, COLOR_BLACK_ALPHA(200/sl_opacity), "sans-bold");
      ui_draw_text(s, rect_s.centerX(), rect_s.centerY()-20, "LIMIT", 55, COLOR_BLACK_ALPHA(200/sl_opacity), "sans-bold");
    } else {
      ui_fill_rect(s->vg, rect_si, COLOR_WHITE_ALPHA(200/sl_opacity), diameter2/2);
      ui_draw_rect(s->vg, rect_s, COLOR_RED_ALPHA(200/sl_opacity), 20, diameter/2);
    }

    if (safety_speed < 100) {
      if (s->scene.speedlimit_signtype) {
        ui_draw_text(s, rect_s.centerX(), rect_s.centerY()+35, safetySpeed, 140, COLOR_BLACK_ALPHA(200/sl_opacity), "sans-bold");
      } else {
        ui_draw_text(s, rect_s.centerX(), rect_s.centerY(), safetySpeed, 160, COLOR_BLACK_ALPHA(200/sl_opacity), "sans-bold");
      }
    } else {
      if (s->scene.speedlimit_signtype) {
        ui_draw_text(s, rect_s.centerX(), rect_s.centerY()+35, safetySpeed, 115, COLOR_BLACK_ALPHA(200/sl_opacity), "sans-bold");
      } else {
        ui_draw_text(s, rect_s.centerX(), rect_s.centerY(), safetySpeed, 115, COLOR_BLACK_ALPHA(200/sl_opacity), "sans-bold");
      }
    }
    if (safety_dist != 0) {
      ui_fill_rect(s->vg, rect_d, COLOR_RED_ALPHA(opacity/sl_opacity), 20.);
      ui_draw_rect(s->vg, rect_d, COLOR_WHITE_ALPHA(200/sl_opacity), 8, 20);
      nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
      ui_draw_text(s, rect_d.centerX(), rect_d.centerY(), safetyDist, 78, COLOR_WHITE_ALPHA(200/sl_opacity), "sans-bold");
    }
  } else if ((s->scene.mapSignCam == 195 || s->scene.mapSignCam == 197) && safety_speed == 0 && safety_dist != 0 && s->scene.navi_select == 1) {
    ui_fill_rect(s->vg, rect_si, COLOR_WHITE_ALPHA(200/sl_opacity), diameter2/2);
    ui_draw_rect(s->vg, rect_s, COLOR_RED_ALPHA(200/sl_opacity), 20, diameter/2);
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    ui_draw_text(s, rect_s.centerX(), rect_s.centerY(), "VAR\nSEC", 108, COLOR_BLACK_ALPHA(200/sl_opacity), "sans-bold");
    if (safety_dist != 0) {
      ui_fill_rect(s->vg, rect_d, COLOR_RED_ALPHA(opacity/sl_opacity), 20.);
      ui_draw_rect(s->vg, rect_d, COLOR_WHITE_ALPHA(200/sl_opacity), 8, 20);
      nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
      ui_draw_text(s, rect_d.centerX(), rect_d.centerY(), safetyDist, 78, COLOR_WHITE_ALPHA(200/sl_opacity), "sans-bold");
    }
  }
}

static void draw_compass(UIState *s) {
  //draw compass by opkr
  if (s->scene.gpsAccuracyUblox != 0.00) {
    const int compass_size = 140;
    const int compass_x = s->fb_w - compass_size - 35;
    const int compass_y = 1080 - compass_size - 35;
    const int from_center = 58;
    const Rect rect = {compass_x, compass_y, compass_size, compass_size};
    char degree[64];
    snprintf(degree, sizeof(degree), "%.0f", s->scene.bearingUblox);
    ui_draw_rect(s->vg, rect, COLOR_WHITE_ALPHA(0), 0, 0);
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    ui_draw_text(s, rect.centerX()+from_center, rect.centerY(), "E", 35, COLOR_WHITE_ALPHA(200), "sans-bold");
    ui_draw_text(s, rect.centerX()-from_center, rect.centerY(), "W", 35, COLOR_WHITE_ALPHA(200), "sans-bold");
    ui_draw_text(s, rect.centerX(), rect.centerY()+from_center, "S", 35, COLOR_WHITE_ALPHA(200), "sans-bold");
    ui_draw_text(s, rect.centerX(), rect.centerY()-from_center, "N", 35, COLOR_WHITE_ALPHA(200), "sans-bold");
    if (337.5 < s->scene.bearingUblox || s->scene.bearingUblox <= 22.5) {
      ui_draw_text(s, rect.centerX(), rect.centerY()-16, "N", 65, COLOR_GREEN_ALPHA(200), "sans-bold");
    } else if (s->scene.bearingUblox <= 67.5) {
      ui_draw_text(s, rect.centerX(), rect.centerY()-16, "NE", 58, COLOR_GREEN_ALPHA(200), "sans-bold");
    } else if (s->scene.bearingUblox <= 112.5) {
      ui_draw_text(s, rect.centerX(), rect.centerY()-16, "E", 65, COLOR_GREEN_ALPHA(200), "sans-bold");
    } else if (s->scene.bearingUblox <= 157.5) {
      ui_draw_text(s, rect.centerX(), rect.centerY()-16, "SE", 58, COLOR_GREEN_ALPHA(200), "sans-bold");
    } else if (s->scene.bearingUblox <= 202.5) {
      ui_draw_text(s, rect.centerX(), rect.centerY()-16, "S", 65, COLOR_GREEN_ALPHA(200), "sans-bold");
    } else if (s->scene.bearingUblox <= 247.5) {
      ui_draw_text(s, rect.centerX(), rect.centerY()-16, "SW", 58, COLOR_GREEN_ALPHA(200), "sans-bold");
    } else if (s->scene.bearingUblox <= 292.5) {
      ui_draw_text(s, rect.centerX(), rect.centerY()-16, "W", 65, COLOR_GREEN_ALPHA(200), "sans-bold");
    } else if (s->scene.bearingUblox <= 337.5) {
      ui_draw_text(s, rect.centerX(), rect.centerY()-16, "NW", 58, COLOR_GREEN_ALPHA(200), "sans-bold");
    }
    ui_draw_text(s, rect.centerX(), rect.centerY()+18, degree, 48, COLOR_WHITE_ALPHA(200), "sans-bold");
    float niddle_rotation = s->scene.bearingUblox/180*3.141592;
    nvgSave(s->vg);
    nvgTranslate(s->vg, compass_x+compass_size/2, compass_y+compass_size/2);
    nvgRotate(s->vg, niddle_rotation);
    nvgFontFace(s->vg, "sans-bold");
    nvgFontSize(s->vg, 78);
    nvgFillColor(s->vg, COLOR_RED_ALPHA(200));
    nvgText(s->vg, 0, -43, "^", NULL);
    nvgRestore(s->vg);
  }
}

static void draw_navi_button(UIState *s) {
  int btn_w = 140;
  int btn_h = 140;
  int btn_x1 = s->fb_w - btn_w - 355;
  int btn_y = 1080 - btn_h - 35;
  int btn_xc1 = btn_x1 + (btn_w/2);
  int btn_yc = btn_y + (btn_h/2);
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
  nvgBeginPath(s->vg);
  nvgRoundedRect(s->vg, btn_x1, btn_y, btn_w, btn_h, 100);
  nvgStrokeColor(s->vg, nvgRGBA(255,255,255,80));
  nvgStrokeWidth(s->vg, 6);
  nvgStroke(s->vg);
  nvgFontSize(s->vg, 55);
  if (s->scene.map_is_running || s->scene.liveENaviData.eopkrconalive) {
    NVGcolor fillColor = nvgRGBA(0,0,255,80);
    nvgFillColor(s->vg, fillColor);
    nvgFill(s->vg);
  }
  nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
  if (s->scene.mapbox_running) {
    nvgFontSize(s->vg, 50);
    nvgText(s->vg,btn_xc1,btn_yc-17,"MAP",NULL);
    nvgFontSize(s->vg, 48);
    nvgText(s->vg,btn_xc1,btn_yc+17,"Search",NULL);
  } else {
    if (s->scene.liveENaviData.eopkrconalive) {
      nvgText(s->vg,btn_xc1,btn_yc,"TMAP",NULL);
    } else {
      nvgText(s->vg,btn_xc1,btn_yc,"NAVI",NULL);
    }
  }
}

static void draw_laneless_button(UIState *s) {
  int btn_w = 140;
  int btn_h = 140;
  int btn_x1 = s->fb_w - btn_w - 195;
  int btn_y = 1080 - btn_h - 35;
  int btn_xc1 = btn_x1 + (btn_w/2);
  int btn_yc = btn_y + (btn_h/2);
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
  nvgBeginPath(s->vg);
  nvgRoundedRect(s->vg, btn_x1, btn_y, btn_w, btn_h, 100);
  nvgStrokeColor(s->vg, nvgRGBA(255,255,255,80));
  nvgStrokeWidth(s->vg, 6);
  nvgStroke(s->vg);
  nvgFontSize(s->vg, 55);
  if (!s->scene.lateralPlan.lanelessModeStatus) {
    nvgBeginPath(s->vg);
    nvgMoveTo(s->vg, btn_xc1-21, btn_yc-57);
    nvgLineTo(s->vg, btn_xc1-31, btn_yc-57);
    nvgLineTo(s->vg, btn_xc1-36, btn_yc-9);
    nvgLineTo(s->vg, btn_xc1-26, btn_yc-9);
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
    nvgFill(s->vg);
    nvgBeginPath(s->vg);
    nvgMoveTo(s->vg, btn_xc1-28, btn_yc+5);
    nvgLineTo(s->vg, btn_xc1-38, btn_yc+5);
    nvgLineTo(s->vg, btn_xc1-43, btn_yc+50);
    nvgLineTo(s->vg, btn_xc1-33, btn_yc+50);
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
    nvgFill(s->vg);
    nvgBeginPath(s->vg);
    nvgMoveTo(s->vg, btn_xc1+21, btn_yc-57);
    nvgLineTo(s->vg, btn_xc1+31, btn_yc-57);
    nvgLineTo(s->vg, btn_xc1+36, btn_yc-9);
    nvgLineTo(s->vg, btn_xc1+26, btn_yc-9);
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
    nvgFill(s->vg);
    nvgBeginPath(s->vg);
    nvgMoveTo(s->vg, btn_xc1+28, btn_yc+5);
    nvgLineTo(s->vg, btn_xc1+38, btn_yc+5);
    nvgLineTo(s->vg, btn_xc1+43, btn_yc+50);
    nvgLineTo(s->vg, btn_xc1+33, btn_yc+50);
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
    nvgFill(s->vg);
  }
  nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
  if (s->scene.laneless_mode == 0) {
    nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
    nvgFontSize(s->vg, 46);
    nvgText(s->vg,btn_xc1,btn_yc-42,"L",NULL);
    nvgText(s->vg,btn_xc1,btn_yc-14,"A",NULL);
    nvgText(s->vg,btn_xc1,btn_yc+13,"N",NULL);
    nvgText(s->vg,btn_xc1,btn_yc+41,"E",NULL);
  } else if (s->scene.laneless_mode == 1) {
    nvgFontSize(s->vg, 50);
    nvgText(s->vg,btn_xc1,btn_yc-17,"LANE",NULL);
    nvgText(s->vg,btn_xc1,btn_yc+17,"LESS",NULL);
  } else if (s->scene.laneless_mode == 2) {
    if (!s->scene.lateralPlan.lanelessModeStatus) {
      nvgFillColor(s->vg, nvgRGBA(0,255,0,150));
    } else {
      nvgFillColor(s->vg, nvgRGBA(0,150,255,150));
    }
    nvgFontSize(s->vg, 46);
    nvgText(s->vg,btn_xc1,btn_yc-42,"A",NULL);
    nvgText(s->vg,btn_xc1,btn_yc-14,"U",NULL);
    nvgText(s->vg,btn_xc1,btn_yc+13,"T",NULL);
    nvgText(s->vg,btn_xc1,btn_yc+41,"O",NULL);
  }
  nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
}

static void ui_draw_vision_header(UIState *s) {
  NVGpaint gradient = nvgLinearGradient(s->vg, 0, header_h - (header_h / 2.5), 0, header_h,
                                        nvgRGBAf(0, 0, 0, 0.45), nvgRGBAf(0, 0, 0, 0));
  ui_fill_rect(s->vg, {0, 0, s->fb_w , header_h}, gradient);

  if (s->scene.comma_stock_ui != 1) {
    if ((*s->sm)["carState"].getCarState().getCruiseButtons() == 1 || (*s->sm)["carState"].getCarState().getCruiseButtons() == 2) {
      s->scene.display_maxspeed_time = 100;
      ui_draw_vision_maxspeed(s);
    } else if (s->scene.display_maxspeed_time > 0) {
      s->scene.display_maxspeed_time--;
      ui_draw_vision_maxspeed(s);
    }
    ui_draw_vision_cruise_speed(s);
  } else {
    ui_draw_vision_maxspeed_org(s);
  }
  ui_draw_vision_speed(s);
  ui_draw_vision_event(s);
  if (s->scene.comma_stock_ui != 1) {
    bb_ui_draw_UI(s);
    if (s->scene.controls_state.getEnabled()) {
      ui_draw_standstill(s);
    }
    if (s->scene.navi_select == 0 || s->scene.navi_select == 1 || s->scene.navi_select == 2 || s->scene.mapbox_running) {
      draw_safetysign(s);
    } else if (s->scene.navi_select == 3 && (s->scene.mapSign != 20 && s->scene.mapSign != 21)) {
      draw_safetysign(s);
    }
    draw_compass(s);
    if (s->scene.navi_select == 0 || s->scene.navi_select == 1 || s->scene.navi_select == 2 || s->scene.navi_select == 3 || s->scene.mapbox_running) {
      draw_navi_button(s);
    }
    if (s->scene.end_to_end) {
      draw_laneless_button(s);
    }
  }
}

//blind spot warning by OPKR
static void ui_draw_blindspot_mon(UIState *s) {
  UIScene &scene = s->scene;
  const int width = 100;
  const int height = s->fb_h;

  const int left_x = 0;
  const int left_y = 0;
  const int right_x = s->fb_w - width;
  const int right_y = 0;

  const Rect rect_l = {left_x, left_y, width, height};
  const Rect rect_r = {right_x, right_y, width, height};

  int car_valid_status = 0;
  bool car_valid_left = scene.leftblindspot;
  bool car_valid_right = scene.rightblindspot;
  int car_valid_alpha;
  if (scene.nOpkrBlindSpotDetect) {
    if (scene.car_valid_status_changed != car_valid_status) {
      scene.blindspot_blinkingrate = 114;
      scene.car_valid_status_changed = car_valid_status;
    }
    if (car_valid_left || car_valid_right) {
      if (!car_valid_left && car_valid_right) {
        car_valid_status = 1;
      } else if (car_valid_left && !car_valid_right) {
        car_valid_status = 2;
      } else if (car_valid_left && car_valid_right) {
        car_valid_status = 3;
      } else {
        car_valid_status = 0;
      }
      scene.blindspot_blinkingrate -= 6;
      if(scene.blindspot_blinkingrate<0) scene.blindspot_blinkingrate = 120;
      if (scene.blindspot_blinkingrate>=60) {
        car_valid_alpha = 150;
      } else {
        car_valid_alpha = 0;
      }
    } else {
      scene.blindspot_blinkingrate = 120;
    }
    if(car_valid_left) {
      ui_fill_rect(s->vg, rect_l, COLOR_ORANGE_ALPHA(car_valid_alpha), 0);
    }
    if(car_valid_right) {
      ui_fill_rect(s->vg, rect_r, COLOR_ORANGE_ALPHA(car_valid_alpha), 0);
    }
  }
}

// draw date/time/streetname
void draw_datetime_osm_info_text(UIState *s) {
  int rect_w = 600;
  int rect_x = s->fb_w/2 - rect_w/2;
  const int rect_h = 60;
  const int rect_y = !s->scene.animated_rpm?0:(s->fb_h-rect_h);
  char dayofweek[50];

  // Get local time to display
  time_t t = time(NULL);
  struct tm tm = *localtime(&t);
  char now[50];
  if (tm.tm_wday == 0) {
    strcpy(dayofweek, "SUN");
  } else if (tm.tm_wday == 1) {
    strcpy(dayofweek, "MON");
  } else if (tm.tm_wday == 2) {
    strcpy(dayofweek, "TUE");
  } else if (tm.tm_wday == 3) {
    strcpy(dayofweek, "WED");
  } else if (tm.tm_wday == 4) {
    strcpy(dayofweek, "THU");
  } else if (tm.tm_wday == 5) {
    strcpy(dayofweek, "FRI");
  } else if (tm.tm_wday == 6) {
    strcpy(dayofweek, "SAT");
  }

  const std::string road_name = s->scene.liveMapData.ocurrentRoadName;
  const std::string ref_ = s->scene.liveMapData.oref;
  std::string text_out = "";
  if (s->scene.top_text_view == 1) {
    snprintf(now,sizeof(now),"%02d-%02d %s %02d:%02d:%02d", tm.tm_mon + 1, tm.tm_mday, dayofweek, tm.tm_hour, tm.tm_min, tm.tm_sec);
    std::string str(now);
    text_out = str;
  } else if (s->scene.top_text_view == 2) {
    snprintf(now,sizeof(now),"%02d-%02d %s", tm.tm_mon + 1, tm.tm_mday, dayofweek);
    std::string str(now);
    text_out = str;
  } else if (s->scene.top_text_view == 3) {
    snprintf(now,sizeof(now),"%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
    std::string str(now);
    text_out = str;
  } else if (s->scene.top_text_view == 4 && s->scene.osm_enabled) {
    snprintf(now,sizeof(now),"%02d-%02d %s %02d:%02d:%02d ", tm.tm_mon + 1, tm.tm_mday, dayofweek, tm.tm_hour, tm.tm_min, tm.tm_sec);
    std::string str(now);
    text_out = str + road_name + ref_;
  } else if (s->scene.top_text_view == 5 && s->scene.osm_enabled) {
    snprintf(now,sizeof(now),"%02d-%02d %s ", tm.tm_mon + 1, tm.tm_mday, dayofweek);
    std::string str(now);
    text_out = str + road_name + ref_;
  } else if (s->scene.top_text_view == 6 && s->scene.osm_enabled) {
    snprintf(now,sizeof(now),"%02d:%02d:%02d ", tm.tm_hour, tm.tm_min, tm.tm_sec);
    std::string str(now);
    text_out = str + road_name + ref_;
  } else if (s->scene.top_text_view == 7 && s->scene.osm_enabled) {
    text_out = road_name;
  }
  float tw = nvgTextBounds(s->vg, 0, 0, text_out.c_str(), nullptr, nullptr);
  rect_w = tw*2;
  rect_x = s->fb_w/2 - rect_w/2;
  nvgBeginPath(s->vg);
  nvgRoundedRect(s->vg, rect_x, rect_y, rect_w, rect_h, 15);
  nvgStrokeWidth(s->vg, 0);
  nvgStroke(s->vg);
  nvgFillColor(s->vg, COLOR_BLACK_ALPHA(60));
  nvgFill(s->vg);
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_TOP);
  ui_draw_text(s, s->fb_w/2, rect_y, text_out.c_str(), s->scene.mapbox_running?33:57, COLOR_WHITE_ALPHA(190), "KaiGenGothicKR-Medium");
}

// live camera offset adjust by OPKR
static void ui_draw_live_tune_panel(UIState *s) {
  const int width = 160;
  const int height = 160;
  const int x_start_pos_l = s->fb_w/2 - width*2;
  const int x_start_pos_r = s->fb_w/2 + width*2;
  const int y_pos = 750;
  //left symbol_above
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x_start_pos_l, y_pos - 175);
  nvgLineTo(s->vg, x_start_pos_l - width + 30, y_pos + height/2 - 175);
  nvgLineTo(s->vg, x_start_pos_l, y_pos + height - 175);
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, nvgRGBA(255,153,153,150));
  nvgFill(s->vg);
  //right symbol above
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x_start_pos_r, y_pos - 175);
  nvgLineTo(s->vg, x_start_pos_r + width - 30, y_pos + height/2 - 175);
  nvgLineTo(s->vg, x_start_pos_r, y_pos + height - 175);
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, nvgRGBA(255,153,153,150));
  nvgFill(s->vg);
  //left symbol
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x_start_pos_l, y_pos);
  nvgLineTo(s->vg, x_start_pos_l - width + 30, y_pos + height/2);
  nvgLineTo(s->vg, x_start_pos_l, y_pos + height);
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, nvgRGBA(171,242,0,150));
  nvgFill(s->vg);
  //right symbol
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x_start_pos_r, y_pos);
  nvgLineTo(s->vg, x_start_pos_r + width - 30, y_pos + height/2);
  nvgLineTo(s->vg, x_start_pos_r, y_pos + height);
  nvgClosePath(s->vg);

  nvgFillColor(s->vg, COLOR_WHITE_ALPHA(150));
  nvgFill(s->vg);

  //param value
  nvgFontSize(s->vg, 170);
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);


  int live_tune_panel_list = s->scene.live_tune_panel_list;
  int lateralControlMethod = s->scene.lateralControlMethod;

  char const* szTuneName = nullptr;
  char szTuneParam[100];
  int list_menu = live_tune_panel_list - (s->scene.list_count);

  if (live_tune_panel_list == 0) {
    sprintf(szTuneParam, "%+0.3f", s->scene.cameraOffset*0.001);
    szTuneName = "CameraOffset";
  } else if (live_tune_panel_list == 1) {
    sprintf(szTuneParam, "%+0.3f", s->scene.pathOffset*0.001);
    szTuneName = "PathOffset";
  } else if (lateralControlMethod == 0) {  // 0.PID

    if ( list_menu == 0 ) {
      sprintf(szTuneParam, "%0.2f", s->scene.pidKp*0.01);
      szTuneName = "Pid: Kp";
    } else if (list_menu == 1 ) {
      sprintf(szTuneParam, "%0.3f", s->scene.pidKi*0.001);
      szTuneName = "Pid: Ki";
    } else if (list_menu == 2 ) {
      sprintf(szTuneParam, "%0.2f", s->scene.pidKd*0.01);
      szTuneName = "Pid: Kd";
    } else if (list_menu == 3 ) {
      sprintf(szTuneParam, "%0.5f", s->scene.pidKf*0.00001);
      szTuneName = "Pid: Kf";
    }
  } else if (lateralControlMethod == 1) {         // 1.INDI

    if ( list_menu == 0 ) {
      sprintf(szTuneParam, "%0.1f", s->scene.indiInnerLoopGain*0.1);
      szTuneName = "INDI: ILGain";
    } else if ( list_menu == 1 ) {
      sprintf(szTuneParam, "%0.1f", s->scene.indiOuterLoopGain*0.1);
      szTuneName = "INDI: OLGain";
    } else if ( list_menu == 2 ) {
      sprintf(szTuneParam, "%0.1f", s->scene.indiTimeConstant*0.1);
      szTuneName = "INDI: TConst";
    } else if ( list_menu == 3 ) {
      sprintf(szTuneParam, "%0.1f", s->scene.indiActuatorEffectiveness*0.1);
      szTuneName = "INDI: ActEffct";
    }
  } else if (lateralControlMethod == 2) {       // 2.LQR

    if ( list_menu == 0 ) {
      sprintf(szTuneParam, "%0.0f", s->scene.lqrScale*1.0);
      szTuneName = "LQR: Scale";
    } else if ( list_menu == 1) {
      sprintf(szTuneParam, "%0.3f", s->scene.lqrKi*0.001);
      szTuneName = "LQR: Ki";
    } else if ( list_menu == 2 ) {
      sprintf(szTuneParam, "%0.5f", s->scene.lqrDcGain*0.00001);
      szTuneName = "LQR: DcGain";
    }
  } else if (lateralControlMethod == 3) {     // 3.TORQUE
    float max_lat_accel = s->scene.torqueMaxLatAccel * 0.1;

    if ( list_menu == 0 ) {
      sprintf(szTuneParam, "%0.1f>%0.2f", s->scene.torqueKp*0.1, (s->scene.torqueKp*0.1)/max_lat_accel);
      szTuneName = "TORQUE: Kp";
    } else if ( list_menu == 1 ) {
      sprintf(szTuneParam, "%0.1f>%0.2f", s->scene.torqueKf*0.1, (s->scene.torqueKf*0.1)/max_lat_accel);
      szTuneName = "TORQUE: Kf";
    } else if ( list_menu == 2 ) {
      sprintf(szTuneParam, "%0.1f>%0.2f", s->scene.torqueKi*0.1, (s->scene.torqueKi*0.1)/max_lat_accel);
      szTuneName = "TORQUE: Ki";
    } else if ( list_menu == 3 ) {
      sprintf(szTuneParam, "%0.1f", s->scene.torqueMaxLatAccel*0.1);
      szTuneName = "TORQUE: MaxL";
    } else if ( list_menu == 4 ) {
      sprintf(szTuneParam, "%0.3f", s->scene.torqueFriction*0.001);
      szTuneName = "TORQUE: Fric";
    }
  } else if (lateralControlMethod == 4) {     // 4.MULTI
    float max_lat_accel = s->scene.torqueMaxLatAccel * 0.1;
    if ( list_menu == 0 ) {
      sprintf(szTuneParam, "%0.2f", s->scene.pidKp*0.01);
      szTuneName = "Pid: Kp";
    } else if (list_menu == 1 ) {
      sprintf(szTuneParam, "%0.3f", s->scene.pidKi*0.001);
      szTuneName = "Pid: Ki";
    } else if (list_menu == 2 ) {
      sprintf(szTuneParam, "%0.2f", s->scene.pidKd*0.01);
      szTuneName = "Pid: Kd";
    } else if (list_menu == 3 ) {
      sprintf(szTuneParam, "%0.5f", s->scene.pidKf*0.00001);
      szTuneName = "Pid: Kf";
    } else if ( list_menu == 4 ) {
      sprintf(szTuneParam, "%0.1f", s->scene.indiInnerLoopGain*0.1);
      szTuneName = "INDI: ILGain";
    } else if ( list_menu == 5 ) {
      sprintf(szTuneParam, "%0.1f", s->scene.indiOuterLoopGain*0.1);
      szTuneName = "INDI: OLGain";
    } else if ( list_menu == 6 ) {
      sprintf(szTuneParam, "%0.1f", s->scene.indiTimeConstant*0.1);
      szTuneName = "INDI: TConst";
    } else if ( list_menu == 7 ) {
      sprintf(szTuneParam, "%0.1f", s->scene.indiActuatorEffectiveness*0.1);
      szTuneName = "INDI: ActEffct";
    } else if ( list_menu == 8 ) {
      sprintf(szTuneParam, "%0.0f", s->scene.lqrScale*1.0);
      szTuneName = "LQR: Scale";
    } else if ( list_menu == 9) {
      sprintf(szTuneParam, "%0.3f", s->scene.lqrKi*0.001);
      szTuneName = "LQR: Ki";
    } else if ( list_menu == 10 ) {
      sprintf(szTuneParam, "%0.5f", s->scene.lqrDcGain*0.00001);
      szTuneName = "LQR: DcGain";
    } else if ( list_menu == 11 ) {
      sprintf(szTuneParam, "%0.1f>%0.2f", s->scene.torqueKp*0.1, (s->scene.torqueKp*0.1)/max_lat_accel);
      szTuneName = "TORQUE: Kp";
    } else if ( list_menu == 12 ) {
      sprintf(szTuneParam, "%0.1f>%0.2f", s->scene.torqueKf*0.1, (s->scene.torqueKf*0.1)/max_lat_accel);
      szTuneName = "TORQUE: Kf";
    } else if ( list_menu == 13 ) {
      sprintf(szTuneParam, "%0.1f>%0.2f", s->scene.torqueKi*0.1, (s->scene.torqueKi*0.1)/max_lat_accel);
      szTuneName = "TORQUE: Ki";
    } else if ( list_menu == 14 ) {
      sprintf(szTuneParam, "%0.1f", s->scene.torqueMaxLatAccel*0.1);
      szTuneName = "TORQUE: MaxL";
    } else if ( list_menu == 15 ) {
      sprintf(szTuneParam, "%0.3f", s->scene.torqueFriction*0.001);
      szTuneName = "TORQUE: Fric";
    }
  }

  if (szTuneName) {
    ui_print(s, s->fb_w/2, y_pos + height/2, "%s", szTuneParam);
    nvgFontSize(s->vg, 120);
    ui_print(s, s->fb_w/2, y_pos - 95, szTuneName);
  }

  nvgFillColor(s->vg, nvgRGBA(171,242,0,150));
  nvgFill(s->vg);
}

static void ui_draw_auto_hold(UIState *s) {
  int y_pos = 0;
  if (s->scene.steer_warning && (s->scene.car_state.getVEgo() < 0.1 || s->scene.stand_still) && s->scene.car_state.getSteeringAngleDeg() < 90) {
    y_pos = 500;
  } else {
    y_pos = 740;
  }
  const int width = 500;
  const Rect rect = {s->fb_w/2 - width/2, y_pos, width, 145};
  NVGcolor color = COLOR_BLACK_ALPHA(50);
  ui_fill_rect(s->vg, rect, color, 30.);
  ui_draw_rect(s->vg, rect, COLOR_WHITE_ALPHA(50), 10, 20.);
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
  ui_draw_text(s, rect.centerX(), rect.centerY(), "AUTO HOLD", 100, COLOR_GREEN_ALPHA(150), "sans-bold");
}

// rpm animation by opkr
static void ui_draw_rpm_animation(UIState *s) {
  float center_x = (float)s->fb_w/2.0f;
  float center_y = 185.0f;
  float radius_i = 140.0f;
  float radius_o = 185.0f;
  char rpm_str[6];
  float max_rpm = (float)s->scene.max_animated_rpm;
  float rpm = fmin(s->scene.engine_rpm, max_rpm);
  //float rpm = 3600.0f;
  // yp = y0 + ((y1-y0)/(x1-x0)) * (xp - x0),  yp = interp(xp, [x0, x1], [y0, y1])
  float rpm_to_deg = floor(9.0f + ((27.0f-9.0f)/(max_rpm-0.0f)) * (rpm - 0.0f)); // min:9, max:27
  float target1 = (float)(NVG_PI/12.0f)*9.0f;
  float target2 = (float)(NVG_PI/12.0f)*10.0f;

  for (int count = 9; count < (int)rpm_to_deg; ++count) {
    if (rpm < 1.0f) break;
    nvgBeginPath(s->vg);
    nvgArc(s->vg, center_x, center_y, radius_i, target1, target2, NVG_CW);
    nvgArc(s->vg, center_x, center_y, radius_o, target2, target1, NVG_CCW);
    nvgClosePath(s->vg);
    nvgStrokeWidth(s->vg, 3);
    nvgStrokeColor(s->vg, COLOR_BLACK_ALPHA(100));
    nvgStroke(s->vg);
    if (count < 12) {
      nvgFillColor(s->vg, nvgRGBA(25,127,54,200));
    } else if (count < 15) {
      nvgFillColor(s->vg, nvgRGBA(34,177,76,200));
    } else if (count < 18) {
      nvgFillColor(s->vg, nvgRGBA(0,255,0,200));
    } else if (count < 21) {
      nvgFillColor(s->vg, nvgRGBA(255,201,14,200));
    } else if (count < 24) {
      nvgFillColor(s->vg, nvgRGBA(255,127,39,200));
    } else if (count < 28) {
      nvgFillColor(s->vg, nvgRGBA(255,0,0,200));
    }
    nvgFill(s->vg);
    target1 = target2;
    target2 = (float)(NVG_PI/12.0f)*((float)count+2.0f);
  }
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
  ui_draw_text(s, center_x, center_y+110, s->scene.is_metric?"KPH":"MPH", 65, s->scene.brakeLights?COLOR_RED_ALPHA(200):COLOR_WHITE_ALPHA(200), "sans-semibold");
  snprintf(rpm_str, sizeof(rpm_str), "%.0f", s->scene.engine_rpm);
  ui_draw_text(s, center_x, center_y-110, s->scene.engine_rpm>1?rpm_str:"", 55, COLOR_WHITE_ALPHA(200), "sans-semibold");
}

// grid line by opkr for mounting device appropriate position  BF:Back and Forth Angle, RL:Right and Left Angle
static void ui_draw_grid(UIState *s) {
  NVGcolor color = COLOR_WHITE_ALPHA(230);
  nvgBeginPath(s->vg);
  nvgStrokeWidth(s->vg, 3);
  nvgStrokeColor(s->vg, color);
  for (int i = 0; i < 8; i++) {
    nvgMoveTo(s->vg, s->fb_w/2 + (i*120), 0);
    nvgLineTo(s->vg, s->fb_w/2 + (i*120) , s->fb_h);
    nvgMoveTo(s->vg, s->fb_w/2 - (i*120), 0);
    nvgLineTo(s->vg, s->fb_w/2 - (i*120) , s->fb_h);
  }
  for (int i = 0; i < 5; i++) {
    nvgMoveTo(s->vg, 0, s->fb_h/2 + (i*108));
    nvgLineTo(s->vg, s->fb_w, s->fb_h/2 + (i*108));
    nvgMoveTo(s->vg, 0, s->fb_h/2 - (i*108));
    nvgLineTo(s->vg, s->fb_w, s->fb_h/2 - (i*108));
  }
  nvgStroke(s->vg);
}

static void ui_draw_vision(UIState *s) {
  const UIScene *scene = &s->scene;
  // Draw augmented elements
  if (scene->world_objects_visible) {
    ui_draw_world(s);
  }
  // Set Speed, Current Speed, Status/Events
  ui_draw_vision_header(s);
  if ((*s->sm)["controlsState"].getControlsState().getAlertSize() == cereal::ControlsState::AlertSize::NONE) {
    ui_draw_vision_face(s);
    if (scene->comma_stock_ui != 1) {
      ui_draw_blindspot_mon(s);
    }
  }
  if (scene->live_tune_panel_enable) {
    ui_draw_live_tune_panel(s);
  }
  if (scene->top_text_view > 0 && scene->comma_stock_ui != 1) {
    draw_datetime_osm_info_text(s);
  }
  if (scene->brakeHold && scene->comma_stock_ui != 1) {
    ui_draw_auto_hold(s);
  }
  if (s->scene.animated_rpm && scene->comma_stock_ui != 1) {
    ui_draw_rpm_animation(s);
  }
  if (scene->cal_view) {
    ui_draw_grid(s);
  }
  if (s->scene.stop_line && scene->comma_stock_ui != 1) {
    ui_draw_stop_sign(s);
  }
}

void ui_draw(UIState *s, int w, int h) {
  // Update intrinsics matrix after possible wide camera toggle change
  if (s->fb_w != w || s->fb_h != h) {
    ui_resize(s, w, h);
  }
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  nvgBeginFrame(s->vg, s->fb_w, s->fb_h, 1.0f);
  ui_draw_vision(s);
  dashcam(s);
  nvgEndFrame(s->vg);
  glDisable(GL_BLEND);
}

void ui_draw_image(const UIState *s, const Rect &r, const char *name, float alpha) {
  nvgBeginPath(s->vg);
  NVGpaint imgPaint = nvgImagePattern(s->vg, r.x, r.y, r.w, r.h, 0, s->images.at(name), alpha);
  nvgRect(s->vg, r.x, r.y, r.w, r.h);
  nvgFillPaint(s->vg, imgPaint);
  nvgFill(s->vg);
}

void ui_draw_rect(NVGcontext *vg, const Rect &r, NVGcolor color, int width, float radius) {
  nvgBeginPath(vg);
  radius > 0 ? nvgRoundedRect(vg, r.x, r.y, r.w, r.h, radius) : nvgRect(vg, r.x, r.y, r.w, r.h);
  nvgStrokeColor(vg, color);
  nvgStrokeWidth(vg, width);
  nvgStroke(vg);
}

static inline void fill_rect(NVGcontext *vg, const Rect &r, const NVGcolor *color, const NVGpaint *paint, float radius) {
  nvgBeginPath(vg);
  radius > 0 ? nvgRoundedRect(vg, r.x, r.y, r.w, r.h, radius) : nvgRect(vg, r.x, r.y, r.w, r.h);
  if (color) nvgFillColor(vg, *color);
  if (paint) nvgFillPaint(vg, *paint);
  nvgFill(vg);
}
void ui_fill_rect(NVGcontext *vg, const Rect &r, const NVGcolor &color, float radius) {
  fill_rect(vg, r, &color, nullptr, radius);
}
void ui_fill_rect(NVGcontext *vg, const Rect &r, const NVGpaint &paint, float radius) {
  fill_rect(vg, r, nullptr, &paint, radius);
}

void ui_nvg_init(UIState *s) {
  // on EON, we enable MSAA
  s->vg = Hardware::EON() ? nvgCreate(0) : nvgCreate(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
  assert(s->vg);

  // init fonts
  std::pair<const char *, const char *> fonts[] = {
      {"sans-regular", "../assets/fonts/opensans_regular.ttf"},
      {"sans-semibold", "../assets/fonts/opensans_semibold.ttf"},
      {"sans-bold", "../assets/fonts/opensans_bold.ttf"},
      {"KaiGenGothicKR-Normal", "../assets/addon/font/KaiGenGothicKR-Normal.ttf"},
      {"KaiGenGothicKR-Medium", "../assets/addon/font/KaiGenGothicKR-Medium.ttf"},
      {"KaiGenGothicKR-Bold", "../assets/addon/font/KaiGenGothicKR-Bold.ttf"},
  };
  for (auto [name, file] : fonts) {
    int font_id = nvgCreateFont(s->vg, name, file);
    assert(font_id >= 0);
  }

  // init images
  std::vector<std::pair<const char *, const char *>> images = {
    {"wheel", "../assets/img_chffr_wheel.png"},
    {"driver_face", "../assets/img_driver_face.png"},
    {"speed_bump", "../assets/addon/img/img_speed_bump.png"},
  };
  for (auto [name, file] : images) {
    s->images[name] = nvgCreateImage(s->vg, file, 1);
    assert(s->images[name] != 0);
  }
}

void ui_resize(UIState *s, int width, int height) {
  s->fb_w = width;
  s->fb_h = height;

  auto intrinsic_matrix = s->wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix;
  float zoom = ZOOM / intrinsic_matrix.v[0];
  if (s->wide_camera) {
    zoom *= 0.5;
  }

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  s->car_space_transform.reset();
  s->car_space_transform.translate(width / 2, height / 2 + y_offset)
      .scale(zoom, zoom)
      .translate(-intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);
}
