#include "selfdrive/ui/qt/offroad/settings.h"

#include <cassert>
#include <cmath>
#include <string>

#include <QDebug>
#include <QProcess> // opkr
#include <QDateTime> // opkr
#include <QTimer> // opkr
#include <QFileInfo> // opkr

#ifndef QCOM
#include "selfdrive/ui/qt/offroad/networking.h"
#endif

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_settings.h"
#endif

#include "selfdrive/common/params.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/widgets/ssh_keys.h"
#include "selfdrive/ui/qt/widgets/toggle.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"

#include "selfdrive/ui/qt/widgets/opkr.h"
#include "selfdrive/ui/qt/widgets/steerWidget.h"


TogglesPanel::TogglesPanel(SettingsWindow *parent) : ListWidget(parent) {
  // param, title, desc, icon
  std::vector<std::tuple<QString, QString, QString, QString>> toggles{
    {
      "OpenpilotEnabledToggle",
      tr("Enable openpilot"),
      tr("Use the openpilot system for adaptive cruise control and lane keep driver assistance. Your attention is required at all times to use this feature. Changing this setting takes effect when the car is powered off."),
      "../assets/offroad/icon_openpilot.png",
    },
    {
      "IsLdwEnabled",
      tr("Enable Lane Departure Warnings"),
      tr("Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line without a turn signal activated while driving over 31 mph (50 km/h)."),
      "../assets/offroad/icon_warning.png",
    },
    {
      "IsRHD",
      tr("Enable Right-Hand Drive"),
      tr("Allow openpilot to obey left-hand traffic conventions and perform driver monitoring on right driver seat."),
      "../assets/offroad/icon_openpilot_mirrored.png",
    },
    {
      "IsMetric",
      tr("Use Metric System"),
      tr("Display speed in km/h instead of mph."),
      "../assets/offroad/icon_metric.png",
    },
    {
      "RecordFront",
      tr("Record and Upload Driver Camera"),
      tr("Upload data from the driver facing camera and help improve the driver monitoring algorithm."),
      "../assets/offroad/icon_monitoring.png",
    },
    {
      "EndToEndToggle",
      "\U0001f96c"+tr("Enable Lane selector Mode")+"\U0001f96c",
      tr("Activate lane selection mode. Lane Mode, Lane Less, AUTO can be selected and switched on the screen."),
      "../assets/offroad/icon_road.png",
    },
#ifdef ENABLE_MAPS
    {
      "NavSettingTime24h",
      tr("Show ETA in 24h format"),
      tr("Use 24h format instead of am/pm"),
      "../assets/offroad/icon_metric.png",
    },
#endif

    {
      "OpkrEnableDriverMonitoring",
      tr("Enable Driver Monitoring"),
      tr("Use the driver monitoring function."),
      "../assets/offroad/icon_shell.png",
    },
    {
      "OpkrEnableLogger",
      tr("Enable Driving Log Record"),
      tr("Record the driving log locally for data analysis. Only loggers are activated and not uploaded to the server."),
      "../assets/offroad/icon_shell.png",
    },
    {
      "OpkrEnableUploader",
      tr("Enable Sending Log to Server"),
      tr("Activate the upload process to transmit system logs and other driving data to the server. Upload it only off-road."),
      "../assets/offroad/icon_shell.png",
    },
  };

  Params params;

  if (params.getBool("DisableRadar_Allow")) {
    toggles.push_back({
      "DisableRadar",
      tr("openpilot Longitudinal Control"),
      tr("openpilot will disable the car's radar and will take over control of gas and brakes. Warning: this disables AEB!"),
      "../assets/offroad/icon_speed_limit.png",
    });
  }

  for (auto &[param, title, desc, icon] : toggles) {
    auto toggle = new ParamControl(param, title, desc, icon, this);
    //bool locked = params.getBool((param + "Lock").toStdString());
    //toggle->setEnabled(true);
    //connect(parent, &SettingsWindow::offroadTransition, toggle, &ParamControl::setEnabled);
    addItem(toggle);
  }
}

DevicePanel::DevicePanel(SettingsWindow *parent) : ListWidget(parent) {
  setSpacing(50);
  addItem(new LabelControl(tr("Dongle ID"), getDongleId().value_or(tr("N/A"))));
  addItem(new LabelControl(tr("Serial"), params.get("HardwareSerial").c_str()));

  addItem(new OpenpilotView());

  // offroad-only buttons

  auto dcamBtn = new ButtonControl(tr("Driver Camera"), tr("PREVIEW"),
                                   tr("Preview the driver facing camera to ensure that driver monitoring has good visibility. (vehicle must be off)"));
  connect(dcamBtn, &ButtonControl::clicked, [=]() { emit showDriverView(); });
  addItem(dcamBtn);

  if (!params.getBool("Passive")) {
    auto retrainingBtn = new ButtonControl(tr("Review Training Guide"), tr("REVIEW"), tr("Review the rules, features, and limitations of openpilot"));
    connect(retrainingBtn, &ButtonControl::clicked, [=]() {
      if (ConfirmationDialog::confirm(tr("Are you sure you want to review the training guide?"), this)) {
        emit reviewTrainingGuide();
      }
    });
    addItem(retrainingBtn);
  }

  if (Hardware::TICI()) {
    auto regulatoryBtn = new ButtonControl(tr("Regulatory"), tr("VIEW"), "");
    connect(regulatoryBtn, &ButtonControl::clicked, [=]() {
      const std::string txt = util::read_file("../assets/offroad/fcc.html");
      RichTextDialog::alert(QString::fromStdString(txt), this);
    });
    addItem(regulatoryBtn);
  }

  auto resetCalibBtn = new ButtonControl(tr("Reset Calibration"), tr("RESET"), " ");
  connect(resetCalibBtn, &ButtonControl::showDescription, this, &DevicePanel::updateCalibDescription);
  connect(resetCalibBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset calibration?"), this)) {
      params.remove("CalibrationParams");
      params.remove("LiveParameters");
      params.putBool("OnRoadRefresh", true);
      QTimer::singleShot(3000, [this]() {
        params.putBool("OnRoadRefresh", false);
      });
    }
  });
  addItem(resetCalibBtn);

  auto translateBtn = new ButtonControl(tr("Change Language"), tr("CHANGE"), "");
  connect(translateBtn, &ButtonControl::clicked, [=]() {
    QMap<QString, QString> langs = getSupportedLanguages();
    QString currentLang = QString::fromStdString(Params().get("LanguageSetting"));
    QString selection = MultiOptionDialog::getSelection(tr("Select a language"), langs.keys(), langs.key(currentLang), this);
    if (!selection.isEmpty()) {
      // put language setting, exit Qt UI, and trigger fast restart
      Params().put("LanguageSetting", langs[selection].toStdString());
    }
  });
  addItem(translateBtn);

  QObject::connect(parent, &SettingsWindow::offroadTransition, [=](bool offroad) {
    for (auto btn : findChildren<ButtonControl *>()) {
      btn->setEnabled(offroad);
    }
    resetCalibBtn->setEnabled(true);
    translateBtn->setEnabled(true);
  });

  // power buttons
  QHBoxLayout *power_layout = new QHBoxLayout();
  power_layout->setSpacing(30);

  QPushButton *refresh_btn = new QPushButton(tr("Refresh"));
  refresh_btn->setObjectName("refresh_btn");
  power_layout->addWidget(refresh_btn);
  QObject::connect(refresh_btn, &QPushButton::clicked, this, &DevicePanel::refresh);

  QPushButton *reboot_btn = new QPushButton(tr("Reboot"));
  reboot_btn->setObjectName("reboot_btn");
  power_layout->addWidget(reboot_btn);
  QObject::connect(reboot_btn, &QPushButton::clicked, this, &DevicePanel::reboot);

  QPushButton *poweroff_btn = new QPushButton(tr("Power Off"));
  poweroff_btn->setObjectName("poweroff_btn");
  power_layout->addWidget(poweroff_btn);
  QObject::connect(poweroff_btn, &QPushButton::clicked, this, &DevicePanel::poweroff);

  setStyleSheet(R"(
    QPushButton {
      height: 120px;
      border-radius: 15px;
    }
    #refresh_btn { background-color: #83c744; }
    #refresh_btn:pressed { background-color: #c7deb1; }
    #reboot_btn { background-color: #ed8e3b; }
    #reboot_btn:pressed { background-color: #f0bf97; }
    #poweroff_btn { background-color: #E22C2C; }
    #poweroff_btn:pressed { background-color: #FF2424; }
  )");
  addItem(power_layout);
}

void DevicePanel::updateCalibDescription() {
  QString desc =
      tr("openpilot requires the device to be mounted within 4° left or right and "
         "within 5° up or 8° down. openpilot is continuously calibrating, resetting is rarely required.");
  std::string calib_bytes = Params().get("CalibrationParams");
  if (!calib_bytes.empty()) {
    try {
      AlignedBuffer aligned_buf;
      capnp::FlatArrayMessageReader cmsg(aligned_buf.align(calib_bytes.data(), calib_bytes.size()));
      auto calib = cmsg.getRoot<cereal::Event>().getLiveCalibration();
      if (calib.getCalStatus() != 0) {
        double pitch = calib.getRpyCalib()[1] * (180 / M_PI);
        double yaw = calib.getRpyCalib()[2] * (180 / M_PI);
        desc += tr(" Your device is pointed %1° %2 and %3° %4.")
                    .arg(QString::number(std::abs(pitch), 'g', 1), pitch > 0 ? tr("down") : tr("up"),
                         QString::number(std::abs(yaw), 'g', 1), yaw > 0 ? tr("left") : tr("right"));
      }
    } catch (kj::Exception) {
      qInfo() << "invalid CalibrationParams";
    }
  }
  qobject_cast<ButtonControl *>(sender())->setDescription(desc);
}

void DevicePanel::refresh() {
  if (QUIState::ui_state.status == UIStatus::STATUS_DISENGAGED) {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to refresh?"), this)) {
      // Check engaged again in case it changed while the dialog was open
      if (QUIState::ui_state.status == UIStatus::STATUS_DISENGAGED) {
        Params().putBool("OnRoadRefresh", true);
        QTimer::singleShot(3000, []() {
          Params().putBool("OnRoadRefresh", false);
        });
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Disengage to Refresh"), this);
  }
}

void DevicePanel::reboot() {
  if (QUIState::ui_state.status == UIStatus::STATUS_DISENGAGED) {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reboot?"), this)) {
      // Check engaged again in case it changed while the dialog was open
      if (QUIState::ui_state.status == UIStatus::STATUS_DISENGAGED) {
        Params().putBool("DoReboot", true);
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Disengage to Reboot"), this);
  }
}

void DevicePanel::poweroff() {
  if (QUIState::ui_state.status == UIStatus::STATUS_DISENGAGED) {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to power off?"), this)) {
      // Check engaged again in case it changed while the dialog was open
      if (QUIState::ui_state.status == UIStatus::STATUS_DISENGAGED) {
        Params().putBool("DoShutdown", true);
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Disengage to Power Off"), this);
  }
}

SoftwarePanel::SoftwarePanel(QWidget* parent) : ListWidget(parent) {
  gitRemoteLbl = new LabelControl(tr("Git Remote"));
  gitBranchLbl = new LabelControl(tr("Git Branch"));
  gitCommitLbl = new LabelControl(tr("Git Commit"));
  osVersionLbl = new LabelControl(tr("OS Version"));
  versionLbl = new LabelControl(tr("Fork"));
  lastUpdateLbl = new LabelControl(tr("Last Update Check"), "", "");
  updateBtn = new ButtonControl(tr("Check for Updates"), "");
  connect(updateBtn, &ButtonControl::clicked, [=]() {
    if (params.getBool("IsOffroad")) {
      fs_watch->addPath(QString::fromStdString(params.getParamPath("LastUpdateTime")));
      fs_watch->addPath(QString::fromStdString(params.getParamPath("UpdateFailedCount")));
    }
    std::system("/data/openpilot/selfdrive/assets/addon/script/gitcommit.sh");
    std::system("date '+%F %T' > /data/params/d/LastUpdateTime");
    QString last_ping = QString::fromStdString(params.get("LastAthenaPingTime"));
    QString desc = "";
    QString commit_local = QString::fromStdString(Params().get("GitCommit").substr(0, 10));
    QString commit_remote = QString::fromStdString(Params().get("GitCommitRemote").substr(0, 10));
    QString empty = "";
    desc += tr("LOCAL: %1  REMOTE: %2%3%4 ").arg(commit_local, commit_remote, empty, empty);
    if (!last_ping.length()) {
      desc += tr("Network connection is missing or unstable. Check the connection.");
      ConfirmationDialog::alert(desc, this);
    } else if (commit_local == commit_remote) {
      desc += tr("Local and remote match. No update required.");
      ConfirmationDialog::alert(desc, this);
    } else {
      if (QFileInfo::exists("/data/OPKR_Updates.txt")) {
        QFileInfo fileInfo;
        fileInfo.setFile("/data/OPKR_Updates.txt");
        const std::string txt = util::read_file("/data/OPKR_Updates.txt");
        if (UpdateInfoDialog::confirm(desc + "\n" + QString::fromStdString(txt), this)) {
          if (ConfirmationDialog::confirm(tr("Device will be updated and rebooted. Do you want to proceed?"), this)) {std::system("/data/openpilot/selfdrive/assets/addon/script/gitpull.sh");}
        }
      } else {
        QString cmd1 = "wget https://raw.githubusercontent.com/openpilotkr/openpilot/"+QString::fromStdString(params.get("GitBranch"))+"/OPKR_Updates.txt -O /data/OPKR_Updates.txt";
        QProcess::execute(cmd1);
        QTimer::singleShot(2000, []() {});
        if (QFileInfo::exists("/data/OPKR_Updates.txt")) {
          QFileInfo fileInfo;
          fileInfo.setFile("/data/OPKR_Updates.txt");
          const std::string txt = util::read_file("/data/OPKR_Updates.txt");
          if (UpdateInfoDialog::confirm(desc + "\n" + QString::fromStdString(txt), this)) {
            if (ConfirmationDialog::confirm(tr("Device will be updated and rebooted. Do you want to proceed?"), this)) {std::system("/data/openpilot/selfdrive/assets/addon/script/gitpull.sh");}
          }
        }
      }
    }
  });


  auto uninstallBtn = new ButtonControl(tr("Uninstall %1").arg(getBrand()), tr("UNINSTALL"));
  connect(uninstallBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to uninstall?"), this)) {
      params.putBool("DoUninstall", true);
    }
  });
  connect(parent, SIGNAL(offroadTransition(bool)), uninstallBtn, SLOT(setEnabled(bool)));

  QWidget *widgets[] = {osVersionLbl, versionLbl, gitRemoteLbl, gitBranchLbl, lastUpdateLbl, updateBtn};
  for (QWidget* w : widgets) {
    addItem(w);
  }

  addItem(new GitHash());
 // addItem(new GitPullOnBootToggle());

  addItem(new CPresetWidget());

  addItem(new CGitGroup());




  addItem(new CUtilWidget(this));

  addItem(uninstallBtn);
  fs_watch = new QFileSystemWatcher(this);
  QObject::connect(fs_watch, &QFileSystemWatcher::fileChanged, [=](const QString path) {
    if (path.contains("UpdateFailedCount") && std::atoi(params.get("UpdateFailedCount").c_str()) > 0) {
      lastUpdateLbl->setText(tr("failed to fetch update"));
      updateBtn->setText(tr("CHECK"));
      updateBtn->setEnabled(true);
    } else if (path.contains("LastUpdateTime")) {
      updateLabels();
    }
  });
}

void SoftwarePanel::showEvent(QShowEvent *event) {
  updateLabels();
}

void SoftwarePanel::updateLabels() {
  QString lastUpdate = "";
  QString tm = QString::fromStdString(params.get("LastUpdateTime").substr(0, 19));
  if (tm != "") {
    lastUpdate = timeAgo(QDateTime::fromString(tm, "yyyy-MM-dd HH:mm:ss"));
  }

  versionLbl->setText("OPKR");
  lastUpdateLbl->setText(lastUpdate);
  updateBtn->setText(tr("CHECK"));
  updateBtn->setEnabled(true);
  gitRemoteLbl->setText(QString::fromStdString(params.get("GitRemote").substr(19)));
  gitBranchLbl->setText(QString::fromStdString(params.get("GitBranch")));
  gitCommitLbl->setText(QString::fromStdString(params.get("GitCommit")).left(10));
  osVersionLbl->setText(QString::fromStdString(Hardware::get_os_version()).trimmed());
}

C2NetworkPanel::C2NetworkPanel(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->setContentsMargins(50, 0, 50, 0);

  ListWidget *list = new ListWidget();
  list->setSpacing(30);
  // wifi + tethering buttons
#ifdef QCOM
  auto wifiBtn = new ButtonControl(tr("Wi-Fi Settings"), tr("OPEN"));
  QObject::connect(wifiBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_wifi(); });
  list->addItem(wifiBtn);

  auto tetheringBtn = new ButtonControl(tr("Tethering Settings"), tr("OPEN"));
  QObject::connect(tetheringBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_tethering(); });
  list->addItem(tetheringBtn);
#endif
  ipaddress = new LabelControl(tr("IP Address"), "");
  list->addItem(ipaddress);

  list->addItem(new HotspotOnBootToggle());

  // SSH key management
  list->addItem(new SshToggle());
  list->addItem(new SshControl());
  list->addItem(new SshLegacyToggle());

  layout->addWidget(list);
  layout->addStretch(1);
}

void C2NetworkPanel::showEvent(QShowEvent *event) {
  ipaddress->setText(getIPAddress());
}

QString C2NetworkPanel::getIPAddress() {
  std::string result = util::check_output("ifconfig wlan0");
  if (result.empty()) return "";

  const std::string inetaddrr = "inet addr:";
  std::string::size_type begin = result.find(inetaddrr);
  if (begin == std::string::npos) return "";

  begin += inetaddrr.length();
  std::string::size_type end = result.find(' ', begin);
  if (end == std::string::npos) return "";

  return result.substr(begin, end - begin).c_str();
}

QWidget *network_panel(QWidget *parent) {
#ifdef QCOM
  return new C2NetworkPanel(parent);
#else
  return new Networking(parent);
#endif
}

UIPanel::UIPanel(QWidget *parent) : QFrame(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->setContentsMargins(50, 0, 50, 0);
  layout->setSpacing(30);

  // OPKR
  layout->addWidget(new AutoShutdown());
  layout->addWidget(new ForceShutdown());
  layout->addWidget(new VolumeControl());
  layout->addWidget(new BrightnessControl());
  layout->addWidget(new AutoScreenOff());
  layout->addWidget(new BrightnessOffControl());
  layout->addWidget(new DoNotDisturbMode());  
  layout->addWidget(new GetOffAlert());
  layout->addWidget(new BatteryChargingControlToggle());
  layout->addWidget(new ChargingMin());
  layout->addWidget(new ChargingMax());
  layout->addWidget(new DrivingRecordToggle());
  layout->addWidget(new RecordCount());
  layout->addWidget(new RecordQuality());
  const char* record_del = "rm -f /storage/emulated/0/videos/*";
  auto recorddelbtn = new ButtonControl(tr("Delete All Recorded Files"), tr("RUN"));
  QObject::connect(recorddelbtn, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm(tr("Delete all saved recorded files. Do you want to proceed?"), this)){
      std::system(record_del);
    }
  });
  layout->addWidget(recorddelbtn);
  const char* realdata_del = "rm -rf /data/media/0/realdata/*";
  auto realdatadelbtn = new ButtonControl(tr("Delete All Driving Logs"), tr("RUN"));
  QObject::connect(realdatadelbtn, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm(tr("Delete all saved driving logs. Do you want to proceed?"), this)){
      std::system(realdata_del);
    }
  });
  layout->addWidget(realdatadelbtn);
  layout->addWidget(new MonitoringMode());
  layout->addWidget(new MonitorEyesThreshold());
  layout->addWidget(new NormalEyesThreshold());
  layout->addWidget(new BlinkThreshold());
  layout->addWidget(new OPKRNaviSelect());
  layout->addWidget(new ExternalDeviceIP());
  layout->addWidget(new RunNaviOnBootToggle());
  layout->addWidget(new OPKRServerSelect());
  layout->addWidget(new OPKRServerAPI());
  layout->addWidget(new MapboxEnabledToggle());
  layout->addWidget(new OPKRMapboxStyle());
  layout->addWidget(new GoogleMapEnabledToggle());
  layout->addWidget(new OPKRTopTextView());
  layout->addWidget(new RPMAnimatedToggle());
  layout->addWidget(new RPMAnimatedMaxValue());
  layout->addWidget(new ShowStopLineToggle());
  layout->addWidget(new HoldForSettingToggle());
  layout->addWidget(new RTShieldToggle());
  layout->addWidget(new OSMOfflineUseToggle());
}
DrivingPanel::DrivingPanel(QWidget *parent) : QFrame(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->setContentsMargins(50, 0, 50, 0);
  layout->setSpacing(30);
  // OPKR
  layout->addWidget(new AutoResumeToggle());
  layout->addWidget(new RESCountatStandstill());
  layout->addWidget(new CruiseGapAdjustToggle());
  layout->addWidget(new CruiseGapBySpdOn());
  layout->addWidget(new CruiseGapBySpd());
  layout->addWidget(new StandstillResumeAltToggle());
  layout->addWidget(new DepartChimeAtResume());
  layout->addWidget(new VariableCruiseToggle());
  layout->addWidget(new VariableCruiseLevel());
  layout->addWidget(new CruiseSetwithRoadLimitSpeed());
  layout->addWidget(new CruiseSetwithRoadLimitSpeedOffset());
  layout->addWidget(new CruisemodeSelInit());
  layout->addWidget(new LaneChangeSpeed());
  layout->addWidget(new LaneChangeDelay());
  layout->addWidget(new LCTimingFactorUD());
  layout->addWidget(new LCTimingFactor());
  layout->addWidget(new LeftCurvOffset());
  layout->addWidget(new RightCurvOffset());
  layout->addWidget(new BlindSpotDetectToggle());

  layout->addWidget(new CSteerWidget());
  layout->addWidget(new SteerAngleCorrection());
  layout->addWidget(new TurnSteeringDisableToggle());
  layout->addWidget(new CruiseOverMaxSpeedToggle());
  layout->addWidget(new OSMEnabledToggle());
  layout->addWidget(new OSMSpeedLimitEnabledToggle());
  layout->addWidget(new StockNaviSpeedToggle());
  layout->addWidget(new SpeedLimitOffset());
  layout->addWidget(new OSMCustomSpeedLimitUD());
  layout->addWidget(new OSMCustomSpeedLimit());
  layout->addWidget(new SpeedLimitSignType());
  layout->addWidget(new CamDecelDistAdd());
  layout->addWidget(new CurvDecelSelect());
  layout->addWidget(new VCurvSpeedUD());
  layout->addWidget(new VCurvSpeed());
  layout->addWidget(new OCurvSpeedUD());
  layout->addWidget(new OCurvSpeed());
  layout->addWidget(new SpeedBumpDecelToggle());
  layout->addWidget(new OPKREarlyStoppingToggle());
  layout->addWidget(new AutoEnabledToggle());
  layout->addWidget(new AutoEnableSpeed());
  layout->addWidget(new CruiseAutoResToggle());
  layout->addWidget(new RESChoice());
  layout->addWidget(new AutoResCondition());
  layout->addWidget(new AutoResLimitTime());
  layout->addWidget(new AutoRESDelay());
  layout->addWidget(new LaneWidth());
  layout->addWidget(new SpeedLaneWidthUD());
  layout->addWidget(new SpeedLaneWidth());
  layout->addWidget(new RoutineDriveOnToggle());
  layout->addWidget(new RoutineDriveOption());
  layout->addWidget(new CloseToRoadEdgeToggle());
  layout->addWidget(new OPKREdgeOffset());
  layout->addWidget(new ToAvoidLKASFaultToggle());
  layout->addWidget(new ToAvoidLKASFault());
  layout->addWidget(new SpeedCameraOffsetToggle());
}

DeveloperPanel::DeveloperPanel(QWidget *parent) : QFrame(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->setContentsMargins(50, 0, 50, 0);
  layout->setSpacing(30);

  // OPKR
  layout->addWidget(new DebugUiOneToggle());
  layout->addWidget(new DebugUiTwoToggle());
  layout->addWidget(new DebugUiThreeToggle());
  layout->addWidget(new OPKRDebug());
  layout->addWidget(new ShowErrorToggle());
  layout->addWidget(new LongLogToggle());
  layout->addWidget(new PrebuiltToggle());
  layout->addWidget(new FPTwoToggle());
  layout->addWidget(new WhitePandaSupportToggle());
  layout->addWidget(new BattLessToggle());
  layout->addWidget(new ComIssueToggle());
  layout->addWidget(new LDWSToggle());
  layout->addWidget(new GearDToggle());
  layout->addWidget(new SteerWarningFixToggle());
  layout->addWidget(new IgnoreCanErroronISGToggle());
  layout->addWidget(new FCA11MessageToggle());
  layout->addWidget(new UFCModeEnabledToggle());
  layout->addWidget(new StockLKASEnabledatDisenagedStatusToggle());
  layout->addWidget(new C2WithCommaPowerToggle());
  layout->addWidget(new JoystickModeToggle());
  layout->addWidget(new NoSmartMDPSToggle());
  layout->addWidget(new UserSpecificFeature());
  layout->addWidget(new TimeZoneSelectCombo());

  layout->addWidget(horizontal_line());
  layout->addWidget(new CarSelectCombo());

  layout->addWidget(new CPandaGroup());
}

TuningPanel::TuningPanel(QWidget *parent) : QFrame(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);

  layout->setContentsMargins(50, 0, 50, 0);
  layout->setSpacing(30);

  // OPKR
  layout->addWidget(new LabelControl(tr("〓〓〓〓〓〓〓〓【 TUNING 】〓〓〓〓〓〓〓〓"), ""));
  layout->addWidget(new CameraOffset());
  layout->addWidget(new PathOffset());
  layout->addWidget(horizontal_line());

  layout->addWidget(new SteerActuatorDelay());

  layout->addWidget(new TireStiffnessFactor());
  layout->addWidget(new SteerThreshold());
  layout->addWidget(new SteerLimitTimer());

  layout->addWidget(new LiveSteerRatioToggle());
  layout->addWidget(new LiveSRPercent());
  layout->addWidget(new SRBaseControl());
  layout->addWidget(new SRMaxControl());

  layout->addWidget(horizontal_line());
  layout->addWidget(new VariableSteerMaxToggle());
  layout->addWidget(new SteerMax());
  layout->addWidget(new VariableSteerDeltaToggle());
  layout->addWidget(new SteerDeltaUp());
  layout->addWidget(new SteerDeltaDown());

  layout->addWidget(horizontal_line());
  layout->addWidget(new ToAvoidLKASFaultBeyondToggle());
  layout->addWidget(new DesiredCurvatureLimit());

  layout->addWidget(horizontal_line());

  //layout->addWidget(new LabelControl("〓〓〓〓〓〓〓〓【 CONTROL 】〓〓〓〓〓〓〓〓", ""));
 // layout->addWidget(new LateralControl());
  layout->addWidget(new LiveTunePanelToggle());

  layout->addWidget(new CLateralControlGroup());
  layout->addWidget(horizontal_line());
  layout->addWidget(new CLongControlGroup());

}

void SettingsWindow::showEvent(QShowEvent *event) {
  panel_widget->setCurrentIndex(0);
  nav_btns->buttons()[0]->setChecked(true);
}

SettingsWindow::SettingsWindow(QWidget *parent) : QFrame(parent) {

  // setup two main layouts
  sidebar_widget = new QWidget;
  QVBoxLayout *sidebar_layout = new QVBoxLayout(sidebar_widget);
  sidebar_layout->setMargin(0);
  panel_widget = new QStackedWidget();
  panel_widget->setStyleSheet(R"(
    border-radius: 30px;
    background-color: #292929;
  )");

  // close button
  QPushButton *close_btn = new QPushButton("×");
  close_btn->setStyleSheet(R"(
    QPushButton {
      font-size: 140px;
      padding-bottom: 20px;
      font-weight: bold;
      border 1px grey solid;
      border-radius: 50px;
      background-color: #292929;
      font-weight: 400;
    }
    QPushButton:pressed {
      background-color: #3B3B3B;
    }
  )");
  close_btn->setFixedSize(220, 130);
  sidebar_layout->addSpacing(5);
  sidebar_layout->addWidget(close_btn, 0, Qt::AlignCenter);
  QObject::connect(close_btn, &QPushButton::clicked, this, &SettingsWindow::closeSettings);

  // setup panels
  DevicePanel *device = new DevicePanel(this);
  SoftwarePanel *software = new SoftwarePanel(this);
  QObject::connect(device, &DevicePanel::reviewTrainingGuide, this, &SettingsWindow::reviewTrainingGuide);
  QObject::connect(device, &DevicePanel::showDriverView, this, &SettingsWindow::showDriverView);
  QObject::connect(software, &SoftwarePanel::closeSettings, this, &SettingsWindow::closeSettings);

  QList<QPair<QString, QWidget *>> panels = {
    {tr("Device"), device},
    {tr("Network"), network_panel(this)},
    {tr("Toggles"), new TogglesPanel(this)},
    {tr("Software"), software},
    {tr("UIMenu"), new UIPanel(this)},
    {tr("Driving"), new DrivingPanel(this)},
    {tr("Developer"), new DeveloperPanel(this)},
    {tr("Tuning"), new TuningPanel(this)},
  };

  sidebar_layout->addSpacing(45);

#ifdef ENABLE_MAPS
  auto map_panel = new MapPanel(this);
  panels.push_back({tr("Navigation"), map_panel});
  QObject::connect(map_panel, &MapPanel::closeSettings, this, &SettingsWindow::closeSettings);
#endif

  const int padding = panels.size() > 3 ? 0 : 15;

  nav_btns = new QButtonGroup(this);
  for (auto &[name, panel] : panels) {
    QPushButton *btn = new QPushButton(name);
    btn->setCheckable(true);
    btn->setChecked(nav_btns->buttons().size() == 0);
    btn->setStyleSheet(QString(R"(
      QPushButton {
        color: grey;
        border: none;
        background: none;
        font-size: 54px;
        font-weight: 500;
        padding-top: %1px;
        padding-bottom: %1px;
      }
      QPushButton:checked {
        color: white;
      }
      QPushButton:pressed {
        color: #ADADAD;
      }
    )").arg(padding));

    nav_btns->addButton(btn);
    sidebar_layout->addWidget(btn, 0, Qt::AlignRight);

    const int lr_margin = name != tr("Network") ? 50 : 0;  // Network panel handles its own margins
    panel->setContentsMargins(lr_margin, 25, lr_margin, 25);

    ScrollView *panel_frame = new ScrollView(panel, this);
    panel_widget->addWidget(panel_frame);

    QObject::connect(btn, &QPushButton::clicked, [=, w = panel_frame]() {
      btn->setChecked(true);
      panel_widget->setCurrentWidget(w);
    });
  }
  sidebar_layout->setContentsMargins(50, 50, 100, 50);

  // main settings layout, sidebar + main panel
  QHBoxLayout *main_layout = new QHBoxLayout(this);

  sidebar_widget->setFixedWidth(500);
  main_layout->addWidget(sidebar_widget);
  main_layout->addWidget(panel_widget);

  setStyleSheet(R"(
    * {
      color: white;
      font-size: 50px;
    }
    SettingsWindow {
      background-color: black;
    }
  )");
}

void SettingsWindow::hideEvent(QHideEvent *event) {
#ifdef QCOM
  HardwareEon::close_activities();
#endif
}
