#pragma once

#include <QFrame>
#include <QMap>
#include <QTimer>

#include "selfdrive/common/params.h"
#include "selfdrive/ui/ui.h"

typedef QPair<QPair<QString, QString>, QColor> ItemStatus;
Q_DECLARE_METATYPE(ItemStatus);

class Sidebar : public QFrame {
  Q_OBJECT
  Q_PROPERTY(ItemStatus connectStatus MEMBER connect_status NOTIFY valueChanged);
  Q_PROPERTY(ItemStatus pandaStatus MEMBER panda_status NOTIFY valueChanged);
  Q_PROPERTY(ItemStatus tempStatus MEMBER temp_status NOTIFY valueChanged);
  Q_PROPERTY(QString netType MEMBER net_type NOTIFY valueChanged);
  Q_PROPERTY(int netStrength MEMBER net_strength NOTIFY valueChanged);
  Q_PROPERTY(QString iPAddress MEMBER wifi_IPAddress NOTIFY valueChanged);
  Q_PROPERTY(QString connectName MEMBER connect_Name NOTIFY valueChanged);
  Q_PROPERTY(QString bATStatus MEMBER bat_Status NOTIFY valueChanged);
  Q_PROPERTY(int bATPercent MEMBER bat_Percent NOTIFY valueChanged);
  Q_PROPERTY(bool bATLess MEMBER bat_Less NOTIFY valueChanged);
  Q_PROPERTY(QString rSRP MEMBER rsrp NOTIFY valueChanged);

public:
  explicit Sidebar(QWidget* parent = 0);

signals:
  void openSettings();
  void valueChanged();

public slots:
  void updateState(const UIState &s);

private:
  // opkr
  quint64 mLastPressTime = 0;
  bool trig_settings = false;
  static const quint64 MY_LONG_PRESS_THRESHOLD = 350;

protected:
  void paintEvent(QPaintEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void drawMetric(QPainter &p, const QPair<QString, QString> &label, QColor c, int y);

  QImage home_img, settings_img;
  const QMap<cereal::DeviceState::NetworkType, QString> network_type = {
    {cereal::DeviceState::NetworkType::NONE, "--"},
    {cereal::DeviceState::NetworkType::WIFI, "WiFi"},
    {cereal::DeviceState::NetworkType::ETHERNET, "ETH"},
    {cereal::DeviceState::NetworkType::CELL2_G, "2G"},
    {cereal::DeviceState::NetworkType::CELL3_G, "3G"},
    {cereal::DeviceState::NetworkType::CELL4_G, "LTE"},
    {cereal::DeviceState::NetworkType::CELL5_G, "5G"}
  };

  const QRect settings_btn = QRect(50, 35, 200, 117);
  const QRect overlay_btn = QRect(0, 465, 150, 150);
  const QColor good_color = QColor(255, 255, 255);
  const QColor warning_color = QColor(218, 202, 37);
  const QColor danger_color = QColor(201, 34, 49);

  Params params;
  ItemStatus connect_status, panda_status, temp_status;
  QString net_type;
  int net_strength = 0;
  // opkr
  QString wifi_IPAddress = "--";
  QString connect_Name = "---";
  QString bat_Status = "DisCharging";
  int bat_Percent = 0;
  bool bat_Less = false;
  QString rsrp = "--";

  // atom
  const QMap<int, QImage> battery_imgs = {
    {0, QImage("../assets/addon/img/battery.png")},
    {1, QImage("../assets/addon/img/battery_charging.png")},
  };  
};
