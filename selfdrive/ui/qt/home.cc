#include "selfdrive/ui/qt/home.h"

#include <QDateTime>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QVBoxLayout>
#include <QProcess> // opkr
#include <QSoundEffect> // opkr

#include "selfdrive/common/params.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/drive_stats.h"
#include "selfdrive/ui/qt/widgets/prime.h"

// HomeWindow: the container for the offroad and onroad UIs

HomeWindow::HomeWindow(QWidget* parent) : QWidget(parent) {
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->setMargin(0);
  main_layout->setSpacing(0);

  sidebar = new Sidebar(this);
  main_layout->addWidget(sidebar);
  QObject::connect(this, &HomeWindow::update, sidebar, &Sidebar::updateState);
  QObject::connect(sidebar, &Sidebar::openSettings, this, &HomeWindow::openSettings);

  slayout = new QStackedLayout();
  main_layout->addLayout(slayout);

  home = new OffroadHome();
  slayout->addWidget(home);

  onroad = new OnroadWindow(this);
  slayout->addWidget(onroad);

  QObject::connect(this, &HomeWindow::update, onroad, &OnroadWindow::updateStateSignal);
  QObject::connect(this, &HomeWindow::offroadTransitionSignal, onroad, &OnroadWindow::offroadTransitionSignal);

  driver_view = new DriverViewWindow(this);
  connect(driver_view, &DriverViewWindow::done, [=] {
    showDriverView(false);
  });
  slayout->addWidget(driver_view);
  setAttribute(Qt::WA_NoSystemBackground);
}

void HomeWindow::showSidebar(bool show) {
  sidebar->setVisible(show);
}

void HomeWindow::offroadTransition(bool offroad) {
  sidebar->setVisible(offroad);
  if (offroad) {
    slayout->setCurrentWidget(home);
  } else {
    slayout->setCurrentWidget(onroad);
  }
  emit offroadTransitionSignal(offroad);
}

void HomeWindow::showDriverView(bool show) {
  if (show) {
    emit closeSettings();
    slayout->setCurrentWidget(driver_view);
  } else {
    slayout->setCurrentWidget(home);
  }
  sidebar->setVisible(show == false);
}


int HomeWindow::clip(int &x, int lo, int hi)
{
  int  nMin = hi;

  if (hi > x) nMin = x;
  if (lo > nMin) {
    x = lo;
  } else {
    x = nMin;
  }
  return x;
}
  

void HomeWindow::mousePressCommon(QMouseEvent* e, int nDir) {
  int live_tune_panel_list = QUIState::ui_state.scene.live_tune_panel_list;

  if (live_tune_panel_list == 0) {
    QUIState::ui_state.scene.cameraOffset += 5*nDir;
    clip(QUIState::ui_state.scene.cameraOffset, -1000, 1000);
    QString value = QString::number(QUIState::ui_state.scene.cameraOffset);
    Params().put("CameraOffsetAdj", value.toStdString());
  } else if (live_tune_panel_list == 1) {
    QUIState::ui_state.scene.pathOffset += 5*nDir;
    clip(QUIState::ui_state.scene.pathOffset, -1000, 1000);
    QString value = QString::number(QUIState::ui_state.scene.pathOffset);
    Params().put("PathOffsetAdj", value.toStdString());
  }
}






void HomeWindow::mousePressPID(QMouseEvent* e, int nDir) {
  int nMenuPos = QUIState::ui_state.scene.live_tune_panel_list - QUIState::ui_state.scene.list_count;

  if (nMenuPos == 0) {
    QUIState::ui_state.scene.pidKp += nDir;
    // 50
    clip(QUIState::ui_state.scene.pidKp, 1, 50);
    QString value = QString::number(QUIState::ui_state.scene.pidKp);
    Params().put("PidKp", value.toStdString());
  } else if (nMenuPos == 1) {
    QUIState::ui_state.scene.pidKi += nDir;
    clip(QUIState::ui_state.scene.pidKi, 1, 100);
    // 100
    QString value = QString::number(QUIState::ui_state.scene.pidKi);
    Params().put("PidKi", value.toStdString());
  } else if (nMenuPos == 2) {
    QUIState::ui_state.scene.pidKd += 5*nDir;
    // 300
    clip(QUIState::ui_state.scene.pidKd, 0, 300);
    QString value = QString::number(QUIState::ui_state.scene.pidKd);
    Params().put("PidKd", value.toStdString());
  } else if (nMenuPos == 3) {
    QUIState::ui_state.scene.pidKf += nDir;
    clip(QUIState::ui_state.scene.pidKf, 1, 50);
    // 50
    QString value = QString::number(QUIState::ui_state.scene.pidKf);
    Params().put("PidKf", value.toStdString());
  }
}

void HomeWindow::mousePressINDI(QMouseEvent* e, int nDir) {
  int nMenuPos = QUIState::ui_state.scene.live_tune_panel_list - QUIState::ui_state.scene.list_count;

  if (nMenuPos == 0) {
    QUIState::ui_state.scene.indiInnerLoopGain += nDir;
    clip(QUIState::ui_state.scene.indiInnerLoopGain, 1, 200);
    QString value = QString::number(QUIState::ui_state.scene.indiInnerLoopGain);
    Params().put("InnerLoopGain", value.toStdString());
  } else if (nMenuPos == 1) {
    QUIState::ui_state.scene.indiOuterLoopGain += nDir;
    clip(QUIState::ui_state.scene.indiOuterLoopGain, 1, 200);
    QString value = QString::number(QUIState::ui_state.scene.indiOuterLoopGain);
    Params().put("OuterLoopGain", value.toStdString());
  } else if (nMenuPos == 2) {
    QUIState::ui_state.scene.indiTimeConstant += nDir;
    clip(QUIState::ui_state.scene.indiTimeConstant, 1, 200);
    QString value = QString::number(QUIState::ui_state.scene.indiTimeConstant);
    Params().put("TimeConstant", value.toStdString());
  } else if (nMenuPos == 3) {
    QUIState::ui_state.scene.indiActuatorEffectiveness += nDir;
    clip(QUIState::ui_state.scene.indiActuatorEffectiveness, 1, 200);
    QString value = QString::number(QUIState::ui_state.scene.indiActuatorEffectiveness);
    Params().put("ActuatorEffectiveness", value.toStdString());
  }
}

void HomeWindow::mousePressLQR(QMouseEvent* e, int nDir) {
  int nMenuPos = QUIState::ui_state.scene.live_tune_panel_list - QUIState::ui_state.scene.list_count;

  if (nMenuPos == 0) {
    QUIState::ui_state.scene.lqrScale += 50*nDir;
    clip(QUIState::ui_state.scene.lqrScale, 50, 5000);
    QString value = QString::number(QUIState::ui_state.scene.lqrScale);
    Params().put("Scale", value.toStdString());
  } else if (nMenuPos == 1) {
    QUIState::ui_state.scene.lqrKi += nDir;
    clip(QUIState::ui_state.scene.lqrKi, 1, 100);
    QString value = QString::number(QUIState::ui_state.scene.lqrKi);
    Params().put("LqrKi", value.toStdString());
  } else if (nMenuPos == 2) {
    QUIState::ui_state.scene.lqrDcGain += 5*nDir;
    clip(QUIState::ui_state.scene.lqrDcGain, 5, 500);
    QString value = QString::number(QUIState::ui_state.scene.lqrDcGain);
    Params().put("DcGain", value.toStdString());
  }
}

void HomeWindow::mousePressTORQ(QMouseEvent* e, int nDir) {
  int nMenuPos = QUIState::ui_state.scene.live_tune_panel_list - QUIState::ui_state.scene.list_count;
  int max_lat_accel = QUIState::ui_state.scene.torqueMaxLatAccel;

  if (nMenuPos == 0) {
    QUIState::ui_state.scene.torqueKp += nDir;
    clip(QUIState::ui_state.scene.torqueKp, 1, max_lat_accel);
    QString value = QString::number(QUIState::ui_state.scene.torqueKp);
    Params().put("TorqueKp", value.toStdString());
  } else if (nMenuPos == 1) {
    QUIState::ui_state.scene.torqueKf += nDir;
    clip(QUIState::ui_state.scene.torqueKf, 1, max_lat_accel);
    QString value = QString::number(QUIState::ui_state.scene.torqueKf);
    Params().put("TorqueKf", value.toStdString());
  } else if (nMenuPos == 2) {
    QUIState::ui_state.scene.torqueKi += nDir;
    clip(QUIState::ui_state.scene.torqueKi, 1, max_lat_accel);
    QString value = QString::number(QUIState::ui_state.scene.torqueKi);
    Params().put("TorqueKi", value.toStdString());
  } else if (nMenuPos == 3) {
    QUIState::ui_state.scene.torqueMaxLatAccel += nDir;
    clip(QUIState::ui_state.scene.torqueMaxLatAccel, 1, 50);
    QString value = QString::number(QUIState::ui_state.scene.torqueMaxLatAccel);
    Params().put("TorqueMaxLatAccel", value.toStdString());
  } else if (nMenuPos == 4) {
    QUIState::ui_state.scene.torqueFriction += 5*nDir;
    clip(QUIState::ui_state.scene.torqueFriction, 0, 300);
    QString value = QString::number(QUIState::ui_state.scene.torqueFriction);
    Params().put("TorqueFriction", value.toStdString());
  }
}

void HomeWindow::mousePressMULTI(QMouseEvent* e, int nDir) {
  int nMenuPos = QUIState::ui_state.scene.live_tune_panel_list - QUIState::ui_state.scene.list_count;
  int max_lat_accel = QUIState::ui_state.scene.torqueMaxLatAccel;
  if (nMenuPos == 0) {
    QUIState::ui_state.scene.pidKp += nDir;
    // 50
    clip(QUIState::ui_state.scene.pidKp, 1, 50);
    QString value = QString::number(QUIState::ui_state.scene.pidKp);
    Params().put("PidKp", value.toStdString());
  } else if (nMenuPos == 1) {
    QUIState::ui_state.scene.pidKi += nDir;
    clip(QUIState::ui_state.scene.pidKi, 1, 100);
    // 100
    QString value = QString::number(QUIState::ui_state.scene.pidKi);
    Params().put("PidKi", value.toStdString());
  } else if (nMenuPos == 2) {
    QUIState::ui_state.scene.pidKd += 5*nDir;
    // 300
    clip(QUIState::ui_state.scene.pidKd, 0, 300);
    QString value = QString::number(QUIState::ui_state.scene.pidKd);
    Params().put("PidKd", value.toStdString());
  } else if (nMenuPos == 3) {
    QUIState::ui_state.scene.pidKf += nDir;
    clip(QUIState::ui_state.scene.pidKf, 1, 50);
    // 50
    QString value = QString::number(QUIState::ui_state.scene.pidKf);
    Params().put("PidKf", value.toStdString());
  } else if (nMenuPos == 4) {
    QUIState::ui_state.scene.indiInnerLoopGain += nDir;
    clip(QUIState::ui_state.scene.indiInnerLoopGain, 1, 200);
    QString value = QString::number(QUIState::ui_state.scene.indiInnerLoopGain);
    Params().put("InnerLoopGain", value.toStdString());
  } else if (nMenuPos == 5) {
    QUIState::ui_state.scene.indiOuterLoopGain += nDir;
    clip(QUIState::ui_state.scene.indiOuterLoopGain, 1, 200);
    QString value = QString::number(QUIState::ui_state.scene.indiOuterLoopGain);
    Params().put("OuterLoopGain", value.toStdString());
  } else if (nMenuPos == 6) {
    QUIState::ui_state.scene.indiTimeConstant += nDir;
    clip(QUIState::ui_state.scene.indiTimeConstant, 1, 200);
    QString value = QString::number(QUIState::ui_state.scene.indiTimeConstant);
    Params().put("TimeConstant", value.toStdString());
  } else if (nMenuPos == 7) {
    QUIState::ui_state.scene.indiActuatorEffectiveness += nDir;
    clip(QUIState::ui_state.scene.indiActuatorEffectiveness, 1, 200);
    QString value = QString::number(QUIState::ui_state.scene.indiActuatorEffectiveness);
    Params().put("ActuatorEffectiveness", value.toStdString());
  } else if (nMenuPos == 8) {
    QUIState::ui_state.scene.lqrScale += 50*nDir;
    clip(QUIState::ui_state.scene.lqrScale, 50, 5000);
    QString value = QString::number(QUIState::ui_state.scene.lqrScale);
    Params().put("Scale", value.toStdString());
  } else if (nMenuPos == 9) {
    QUIState::ui_state.scene.lqrKi += nDir;
    clip(QUIState::ui_state.scene.lqrKi, 1, 100);
    QString value = QString::number(QUIState::ui_state.scene.lqrKi);
    Params().put("LqrKi", value.toStdString());
  } else if (nMenuPos == 10) {
    QUIState::ui_state.scene.lqrDcGain += 5*nDir;
    clip(QUIState::ui_state.scene.lqrDcGain, 5, 500);
    QString value = QString::number(QUIState::ui_state.scene.lqrDcGain);
    Params().put("DcGain", value.toStdString());
  } else if (nMenuPos == 11) {
    QUIState::ui_state.scene.torqueKp += nDir;
    clip(QUIState::ui_state.scene.torqueKp, 1, max_lat_accel);
    QString value = QString::number(QUIState::ui_state.scene.torqueKp);
    Params().put("TorqueKp", value.toStdString());
  } else if (nMenuPos == 12) {
    QUIState::ui_state.scene.torqueKf += nDir;
    clip(QUIState::ui_state.scene.torqueKf, 1, max_lat_accel);
    QString value = QString::number(QUIState::ui_state.scene.torqueKf);
    Params().put("TorqueKf", value.toStdString());
  } else if (nMenuPos == 13) {
    QUIState::ui_state.scene.torqueKi += nDir;
    clip(QUIState::ui_state.scene.torqueKi, 1, max_lat_accel);
    QString value = QString::number(QUIState::ui_state.scene.torqueKi);
    Params().put("TorqueKi", value.toStdString());
  } else if (nMenuPos == 14) {
    QUIState::ui_state.scene.torqueMaxLatAccel += nDir;
    clip(QUIState::ui_state.scene.torqueMaxLatAccel, 1, 50);
    QString value = QString::number(QUIState::ui_state.scene.torqueMaxLatAccel);
    Params().put("TorqueMaxLatAccel", value.toStdString());
  } else if (nMenuPos == 15) {
    QUIState::ui_state.scene.torqueFriction += 5*nDir;
    clip(QUIState::ui_state.scene.torqueFriction, 0, 300);
    QString value = QString::number(QUIState::ui_state.scene.torqueFriction);
    Params().put("TorqueFriction", value.toStdString());
  }
}

void HomeWindow::mousePressEvent(QMouseEvent* e) 
{
  //float max_lat_accel = QUIState::ui_state.scene.torqueMaxLatAccel;

  printf( "mousePressEvent = (%d,%d)\n", e->x(), e->y() );
  // OPKR add map
  if (QUIState::ui_state.scene.started && map_overlay_btn.ptInRect(e->x(), e->y())) {
    QSoundEffect effect1;
    effect1.setSource(QUrl::fromLocalFile("/data/openpilot/selfdrive/assets/addon/sound/click.wav"));
    //effect1.setLoopCount(1);
    //effect1.setLoopCount(QSoundEffect::Infinite);
    float volume1 = 0.5;
    if (QUIState::ui_state.scene.nVolumeBoost < 0) {
      volume1 = 0.0;
    } else if (QUIState::ui_state.scene.nVolumeBoost > 1) {
      volume1 = QUIState::ui_state.scene.nVolumeBoost * 0.01;
    }
    effect1.setVolume(volume1);
    effect1.play();
    if (!QUIState::ui_state.scene.mapbox_running) {
      QProcess::execute("am start --activity-task-on-home com.opkr.maphack/com.opkr.maphack.MainActivity");
    } else if (QUIState::ui_state.scene.mapbox_running && !QUIState::ui_state.scene.map_on_top && QUIState::ui_state.scene.map_on_overlay) {
      Params().remove("NavDestination");
    } else {
      QProcess::execute("pkill com.android.chrome");
      QProcess::execute("rm -rf /data/data/com.android.chrome/app_tabs/0");
    }
    QUIState::ui_state.scene.map_on_top = false;
    QUIState::ui_state.scene.map_on_overlay = true;
    return;
  }
  if (QUIState::ui_state.scene.started && !sidebar->isVisible() && !QUIState::ui_state.scene.map_on_top && map_btn.ptInRect(e->x(), e->y()) && !QUIState::ui_state.scene.mapbox_running) {
    QSoundEffect effect2;
    effect2.setSource(QUrl::fromLocalFile("/data/openpilot/selfdrive/assets/addon/sound/click.wav"));
    //effect1.setLoopCount(1);
    //effect1.setLoopCount(QSoundEffect::Infinite);
    float volume2 = 0.5;
    if (QUIState::ui_state.scene.nVolumeBoost < 0) {
      volume2 = 0.0;
    } else if (QUIState::ui_state.scene.nVolumeBoost > 1) {
      volume2 = QUIState::ui_state.scene.nVolumeBoost * 0.01;
    }
    effect2.setVolume(volume2);
    effect2.play();
    QUIState::ui_state.scene.map_is_running = !QUIState::ui_state.scene.map_is_running;
    if (QUIState::ui_state.scene.map_is_running) {
      if (QUIState::ui_state.scene.navi_select == 1) {
        QProcess::execute("am start com.mnsoft.mappyobn/com.mnsoft.mappy.MainActivity");
      } else if (QUIState::ui_state.scene.navi_select == 2) {
        QProcess::execute("am start com.thinkware.inaviair/com.thinkware.inaviair.UIActivity");
      } else if (QUIState::ui_state.scene.navi_select == 3) {
        QProcess::execute("am start com.waze/com.waze.MainActivity");
      }
      QUIState::ui_state.scene.map_on_top = true;
      QUIState::ui_state.scene.map_is_running = true;
      QUIState::ui_state.scene.map_on_overlay = false;
      Params().putBool("OpkrMapEnable", true);
    } else {
      if (QUIState::ui_state.scene.navi_select == 1) {
        QProcess::execute("pkill com.mnsoft.mappyobn");
      } else if (QUIState::ui_state.scene.navi_select == 2) {
        QProcess::execute("pkill com.thinkware.inaviair");
      } else if (QUIState::ui_state.scene.navi_select == 3) {
        QProcess::execute("pkill com.waze");
      }
      QUIState::ui_state.scene.map_on_top = false;
      QUIState::ui_state.scene.map_on_overlay = false;
      QUIState::ui_state.scene.map_is_running = false;
      Params().putBool("OpkrMapEnable", false);
    }
    return;
  }
  if (QUIState::ui_state.scene.started && !sidebar->isVisible() && !QUIState::ui_state.scene.map_on_top && mapbox_btn.ptInRect(e->x(), e->y()) && QUIState::ui_state.scene.mapbox_running) {
    QSoundEffect effect4;
    effect4.setSource(QUrl::fromLocalFile("/data/openpilot/selfdrive/assets/addon/sound/click.wav"));
    //effect1.setLoopCount(1);
    //effect1.setLoopCount(QSoundEffect::Infinite);
    float volume2 = 0.5;
    if (QUIState::ui_state.scene.nVolumeBoost < 0) {
      volume2 = 0.0;
    } else if (QUIState::ui_state.scene.nVolumeBoost > 1) {
      volume2 = QUIState::ui_state.scene.nVolumeBoost * 0.01;
    }
    effect4.setVolume(volume2);
    effect4.play();
    QProcess::execute("am start -n com.android.chrome/org.chromium.chrome.browser.ChromeTabbedActivity -d \"http://localhost:8082\" --activity-clear-task");
    QUIState::ui_state.scene.map_on_top = true;
    QUIState::ui_state.scene.map_on_overlay = false;
    return;
  }
  if (QUIState::ui_state.scene.started && QUIState::ui_state.scene.map_is_running && map_return_btn.ptInRect(e->x(), e->y()) && !QUIState::ui_state.scene.mapbox_running) {
    QSoundEffect effect3;
    effect3.setSource(QUrl::fromLocalFile("/data/openpilot/selfdrive/assets/addon/sound/click.wav"));
    //effect1.setLoopCount(1);
    //effect1.setLoopCount(QSoundEffect::Infinite);
    float volume3 = 0.5;
    if (QUIState::ui_state.scene.nVolumeBoost < 0) {
      volume3 = 0.0;
    } else if (QUIState::ui_state.scene.nVolumeBoost > 1) {
      volume3 = QUIState::ui_state.scene.nVolumeBoost * 0.01;
    }
    effect3.setVolume(volume3);
    effect3.play();
    if (QUIState::ui_state.scene.navi_select == 1) {
      QProcess::execute("am start --activity-task-on-home com.mnsoft.mappyobn/com.mnsoft.mappy.MainActivity");
    } else if (QUIState::ui_state.scene.navi_select == 2) {
      QProcess::execute("am start --activity-task-on-home com.thinkware.inaviair/com.thinkware.inaviair.UIActivity");
    } else if (QUIState::ui_state.scene.navi_select == 3) {
      QProcess::execute("am start --activity-task-on-home com.waze/com.waze.MainActivity");
    }
    QUIState::ui_state.scene.map_on_top = true;
    QUIState::ui_state.scene.map_on_overlay = false;
    return;
  }
  // OPKR REC
  if (QUIState::ui_state.scene.started && !sidebar->isVisible() && !QUIState::ui_state.scene.map_on_top && QUIState::ui_state.scene.comma_stock_ui != 1 && rec_btn.ptInRect(e->x(), e->y()) && !QUIState::ui_state.scene.mapbox_running) {
    QUIState::ui_state.scene.touched = true;
    return;
  }
  // Laneless mode
  if (QUIState::ui_state.scene.started && !sidebar->isVisible() && !QUIState::ui_state.scene.map_on_top && QUIState::ui_state.scene.end_to_end && QUIState::ui_state.scene.comma_stock_ui != 1 && laneless_btn.ptInRect(e->x(), e->y()) && !QUIState::ui_state.scene.mapbox_running) {
    QUIState::ui_state.scene.laneless_mode = QUIState::ui_state.scene.laneless_mode + 1;
    if (QUIState::ui_state.scene.laneless_mode > 2) {
      QUIState::ui_state.scene.laneless_mode = 0;
    }
    if (QUIState::ui_state.scene.laneless_mode == 0) {
      Params().put("LanelessMode", "0", 1);
    } else if (QUIState::ui_state.scene.laneless_mode == 1) {
      Params().put("LanelessMode", "1", 1);
    } else if (QUIState::ui_state.scene.laneless_mode == 2) {
      Params().put("LanelessMode", "2", 1);
    }
    return;
  }
  // Monitoring mode
  if (QUIState::ui_state.scene.started && !sidebar->isVisible() && !QUIState::ui_state.scene.map_on_top && monitoring_btn.ptInRect(e->x(), e->y()) && !QUIState::ui_state.scene.mapbox_running) {
    QUIState::ui_state.scene.monitoring_mode = !QUIState::ui_state.scene.monitoring_mode;
    if (QUIState::ui_state.scene.monitoring_mode) {
      Params().putBool("OpkrMonitoringMode", true);
    } else {
      Params().putBool("OpkrMonitoringMode", false);
    }
    return;
  }
  // Stock UI Toggle
  if (QUIState::ui_state.scene.started && !sidebar->isVisible() && !QUIState::ui_state.scene.map_on_top && stockui_btn.ptInRect(e->x(), e->y())) {
    QUIState::ui_state.scene.comma_stock_ui = QUIState::ui_state.scene.comma_stock_ui + 1;
    if (QUIState::ui_state.scene.do_not_disturb_mode > 0) {
      if (QUIState::ui_state.scene.comma_stock_ui > 2) {
        QUIState::ui_state.scene.comma_stock_ui = 0;
      }
    } else {
      if (QUIState::ui_state.scene.comma_stock_ui > 1 ) {
        QUIState::ui_state.scene.comma_stock_ui = 0;
      }
    }

    if (QUIState::ui_state.scene.comma_stock_ui == 0) {
      Params().put("CommaStockUI", "0", 1);
    } else if (QUIState::ui_state.scene.comma_stock_ui == 1) {
      Params().put("CommaStockUI", "1", 1);
    } else if (QUIState::ui_state.scene.comma_stock_ui == 2) {
      Params().put("CommaStockUI", "2", 1);
      QUIState::ui_state.scene.touched2 = true;
      QTimer::singleShot(500, []() { QUIState::ui_state.scene.touched2 = false; });
    }
    return;
  }
  // LiveTune UI Toggle
  if (QUIState::ui_state.scene.started && !sidebar->isVisible() && !QUIState::ui_state.scene.map_on_top && tuneui_btn.ptInRect(e->x(), e->y()) && !QUIState::ui_state.scene.mapbox_running) {
    QUIState::ui_state.scene.opkr_livetune_ui = !QUIState::ui_state.scene.opkr_livetune_ui;
    if (QUIState::ui_state.scene.opkr_livetune_ui) {
      Params().putBool("OpkrLiveTunePanelEnable", true);
      QUIState::ui_state.scene.live_tune_panel_enable = true;
    } else {
      Params().putBool("OpkrLiveTunePanelEnable", false);
      QUIState::ui_state.scene.live_tune_panel_enable = false;
    }
    return;
  }
  // SpeedLimit Decel on/off Toggle
  if (QUIState::ui_state.scene.started && !sidebar->isVisible() && !QUIState::ui_state.scene.map_on_top && speedlimit_btn.ptInRect(e->x(), e->y())) {
    QUIState::ui_state.scene.sl_decel_off = !QUIState::ui_state.scene.sl_decel_off;
    if (QUIState::ui_state.scene.sl_decel_off) {
      Params().putBool("SpeedLimitDecelOff", true);
    } else {
      Params().putBool("SpeedLimitDecelOff", false);
    }
    return;
  }
  // opkr live ui tune
  if (QUIState::ui_state.scene.live_tune_panel_enable && QUIState::ui_state.scene.started && !sidebar->isVisible() && !QUIState::ui_state.scene.map_on_top) {
    int nBtnDir = 0;

    if (livetunepanel_left_btn.ptInRect(e->x(), e->y())) {
      nBtnDir = -1;
    } else if (livetunepanel_right_btn.ptInRect(e->x(), e->y())) {
      nBtnDir = 1;
    } else if (livetunepanel_left_above_btn.ptInRect(e->x(), e->y())) {
      QUIState::ui_state.scene.live_tune_panel_list -= 1;
      int nLoop = 2;

      if (QUIState::ui_state.scene.live_tune_panel_list >= 0) return;
      if (QUIState::ui_state.scene.lateralControlMethod == 2) {  // 2. LQR
        nLoop = 2;
      } else if (QUIState::ui_state.scene.lateralControlMethod == 3) { // 3. TORQ
        nLoop = 4;
      } else if (QUIState::ui_state.scene.lateralControlMethod < 2) {
        nLoop = 3;
      } else if (QUIState::ui_state.scene.lateralControlMethod == 4) {
        nLoop = 15;
      }

      QUIState::ui_state.scene.live_tune_panel_list = QUIState::ui_state.scene.list_count + nLoop;
      //clip( QUIState::ui_state.scene.live_tune_panel_list, 1, 0 );
      return;
    } else if (livetunepanel_right_above_btn.ptInRect(e->x(), e->y())) {
      QUIState::ui_state.scene.live_tune_panel_list += 1;
      int nLoop = QUIState::ui_state.scene.list_count;

      if (QUIState::ui_state.scene.lateralControlMethod == 2) { // 2. LQR
         nLoop = nLoop + 3;
      } else if (QUIState::ui_state.scene.lateralControlMethod == 3) { // 3. TORQ
        nLoop = nLoop + 5;
      } else if (QUIState::ui_state.scene.lateralControlMethod < 2) { // 0. PID,  1. INDI
        nLoop = nLoop + 4;
      } else if (QUIState::ui_state.scene.lateralControlMethod == 4) { // 4. MULTI
        nLoop = nLoop + 16;
      }

      if(QUIState::ui_state.scene.live_tune_panel_list < nLoop) return;
      QUIState::ui_state.scene.live_tune_panel_list = 0;
      return;
    }

    if (nBtnDir) {
      mousePressCommon(e, nBtnDir);
      if (QUIState::ui_state.scene.lateralControlMethod == 0) {  // 0. PID
        mousePressPID(e, nBtnDir);
      } else if (QUIState::ui_state.scene.lateralControlMethod == 1) {  // 1. INDI
        mousePressINDI(e, nBtnDir);
      } else if (QUIState::ui_state.scene.lateralControlMethod == 2) {  // 2. LQR
        mousePressLQR(e, nBtnDir);
      } else if (QUIState::ui_state.scene.lateralControlMethod == 3) {  // 3. TORQ
        mousePressTORQ(e, nBtnDir);
      } else if (QUIState::ui_state.scene.lateralControlMethod == 4) {  // 4. MULTI
        mousePressMULTI(e, nBtnDir);
      }
      return;
    }
  }

  // Handle sidebar collapsing
  if (onroad->isVisible() && (!sidebar->isVisible() || e->x() > sidebar->width())) {
    sidebar->setVisible(!sidebar->isVisible() && !onroad->isMapVisible());
    QUIState::ui_state.sidebar_view = !QUIState::ui_state.sidebar_view;
  }
  
  if (QUIState::ui_state.scene.started && QUIState::ui_state.scene.autoScreenOff != -3) {
    QUIState::ui_state.scene.touched2 = true;
    QTimer::singleShot(500, []() { QUIState::ui_state.scene.touched2 = false; });
  }

  if (QUIState::ui_state.scene.monitoring_mode) {
    Params().putBool("OpkrWakeUp", false);
  }
}

// OffroadHome: the offroad home page

OffroadHome::OffroadHome(QWidget* parent) : QFrame(parent) {
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(40, 40, 40, 45);

  // top header
  QHBoxLayout* header_layout = new QHBoxLayout();
  header_layout->setContentsMargins(15, 15, 15, 0);
  header_layout->setSpacing(16);

  date = new QLabel();
  header_layout->addWidget(date, 1, Qt::AlignHCenter | Qt::AlignLeft);

  update_notif = new QPushButton(tr("UPDATE"));
  update_notif->setVisible(false);
  update_notif->setStyleSheet("background-color: #364DEF;");
  QObject::connect(update_notif, &QPushButton::clicked, [=]() { center_layout->setCurrentIndex(1); });
  header_layout->addWidget(update_notif, 0, Qt::AlignHCenter | Qt::AlignRight);

  alert_notif = new QPushButton();
  alert_notif->setVisible(false);
  alert_notif->setStyleSheet("background-color: #E22C2C;");
  QObject::connect(alert_notif, &QPushButton::clicked, [=] { center_layout->setCurrentIndex(2); });
  header_layout->addWidget(alert_notif, 0, Qt::AlignHCenter | Qt::AlignRight);

  header_layout->addWidget(new QLabel("OPKR"), 0, Qt::AlignHCenter | Qt::AlignRight);

  main_layout->addLayout(header_layout);

  // main content
  main_layout->addSpacing(25);
  center_layout = new QStackedLayout();

  QWidget* statsAndSetupWidget = new QWidget(this);
  QHBoxLayout* statsAndSetup = new QHBoxLayout(statsAndSetupWidget);
  statsAndSetup->setMargin(0);
  statsAndSetup->setSpacing(30);
  statsAndSetup->addWidget(new DriveStats, 1);
  statsAndSetup->addWidget(new SetupWidget);

  center_layout->addWidget(statsAndSetupWidget);

  // add update & alerts widgets
  update_widget = new UpdateAlert();
  QObject::connect(update_widget, &UpdateAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(update_widget);
  alerts_widget = new OffroadAlert();
  QObject::connect(alerts_widget, &OffroadAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(alerts_widget);

  main_layout->addLayout(center_layout, 1);

  // set up refresh timer
  timer = new QTimer(this);
  timer->callOnTimeout(this, &OffroadHome::refresh);

  setStyleSheet(R"(
    * {
     color: white;
    }
    OffroadHome {
      background-color: black;
    }
    OffroadHome > QPushButton {
      padding: 15px 30px;
      border-radius: 5px;
      font-size: 40px;
      font-weight: 500;
    }
    OffroadHome > QLabel {
      font-size: 55px;
    }
  )");
}

void OffroadHome::showEvent(QShowEvent *event) {
  refresh();
  timer->start(10 * 1000);
}

void OffroadHome::hideEvent(QHideEvent *event) {
  timer->stop();
}

void OffroadHome::refresh() {
  date->setText(QDateTime::currentDateTime().toString("dddd, MMMM d, hh:mm"));

  bool updateAvailable = update_widget->refresh();
  int alerts = alerts_widget->refresh();

  // pop-up new notification
  int idx = center_layout->currentIndex();
  if (!updateAvailable && !alerts) {
    idx = 0;
  } else if (updateAvailable && (!update_notif->isVisible() || (!alerts && idx == 2))) {
    idx = 1;
  } else if (alerts && (!alert_notif->isVisible() || (!updateAvailable && idx == 1))) {
    idx = 2;
  }
  center_layout->setCurrentIndex(idx);

  update_notif->setVisible(updateAvailable);
  alert_notif->setVisible(alerts);
  if (alerts) {
    alert_notif->setText(QString::number(alerts) + (alerts > 1 ? tr(" ALERTS") : tr(" ALERT")));
  }
}
