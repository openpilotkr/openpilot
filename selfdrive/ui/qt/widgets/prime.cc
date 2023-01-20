#include "selfdrive/ui/qt/widgets/prime.h"

#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <QrCode.hpp>

#include "selfdrive/common/params.h"
#include "selfdrive/ui/qt/request_repeater.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"

using qrcodegen::QrCode;

PairingQRWidget::PairingQRWidget(QWidget* parent) : QWidget(parent) {
  qrCode = new QLabel;
  qrCode->setScaledContents(true);
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->addWidget(qrCode, 0, Qt::AlignCenter);

  QTimer* timer = new QTimer(this);
  timer->start(5 * 60 * 1000);
  connect(timer, &QTimer::timeout, this, &PairingQRWidget::refresh);
}

void PairingQRWidget::showEvent(QShowEvent *event) {
  refresh();
}

void PairingQRWidget::refresh() {
  Params params;
  QString IMEI = QString::fromStdString(params.get("IMEI"));
  QString serial = QString::fromStdString(params.get("HardwareSerial"));

  if (std::min(IMEI.length(), serial.length()) <= 5) {
    qrCode->setText(tr("Error getting serial: contact support"));
    qrCode->setWordWrap(true);
    qrCode->setStyleSheet(R"(font-size: 48px;)");
    return;
  }
  QString pairToken = CommaApi::create_jwt({{"pair", true}});

  QString qrString = IMEI + "--" + serial + "--" + pairToken;
  this->updateQrCode(qrString);
}

void PairingQRWidget::updateQrCode(const QString &text) {
  QrCode qr = QrCode::encodeText(text.toUtf8().data(), QrCode::Ecc::LOW);
  qint32 sz = qr.getSize();
  // make the image larger so we can have a white border
  QImage im(sz + 2, sz + 2, QImage::Format_RGB32);
  QRgb black = qRgb(0, 0, 0);
  QRgb white = qRgb(255, 255, 255);

  for (int y = 0; y < sz + 2; y++) {
    for (int x = 0; x < sz + 2; x++) {
      im.setPixel(x, y, white);
    }
  }
  for (int y = 0; y < sz; y++) {
    for (int x = 0; x < sz; x++) {
      im.setPixel(x + 1, y + 1, qr.getModule(x, y) ? black : white);
    }
  }
  // Integer division to prevent anti-aliasing
  int approx500 = (500 / (sz + 2)) * (sz + 2);
  qrCode->setPixmap(QPixmap::fromImage(im.scaled(approx500, approx500, Qt::KeepAspectRatio, Qt::FastTransformation), Qt::MonoOnly));
  qrCode->setFixedSize(approx500, approx500);
}

PrimeUserWidget::PrimeUserWidget(QWidget* parent) : QWidget(parent) {
  mainLayout = new QVBoxLayout(this);
  mainLayout->setMargin(30);

  QLabel* commaPrime = new QLabel("OPKR");
  mainLayout->addWidget(commaPrime, 0, Qt::AlignCenter);
  mainLayout->addSpacing(15);
  QPixmap hkgpix("../assets/addon/img/hkg.png");
  QLabel *hkg = new QLabel();
  hkg->setPixmap(hkgpix.scaledToWidth(470, Qt::SmoothTransformation));
  hkg->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
  mainLayout->addWidget(hkg, 0, Qt::AlignCenter);

  opusername = new QLabel();
  opusername->setStyleSheet("font-size: 55px;"); // TODO: fit width
  //mainLayout->addWidget(opusername, 0, Qt::AlignTop);

  //mainLayout->addSpacing(10);

  QLabel* commaPoints = new QLabel("COMMA POINTS");
  commaPoints->setStyleSheet(R"(
    color: #b8b8b8;
  )");
  //mainLayout->addWidget(commaPoints, 0, Qt::AlignTop);

  points = new QLabel();
  //mainLayout->addWidget(points, 0, Qt::AlignTop);

  setStyleSheet(R"(
    QLabel {
      font-size: 70px;
      font-weight: 500;
    }
  )");
  setStyleSheet(R"(
    PrimeUserWidget {
      background-color: #333333;
      border-radius: 10px;
    }
  )");

  // set up API requests
  QString OPKR_SERVER = QString::fromStdString(Params().get("OPKRServer"));
  QString TARGET_SERVER = "";
  if (OPKR_SERVER == "0") {
    TARGET_SERVER = util::getenv("API_HOST", "http://opkr.tk:3000").c_str();
  } else if (OPKR_SERVER == "1") {
    TARGET_SERVER = util::getenv("API_HOST", "https://api.commadotai.com").c_str();
  } else if (OPKR_SERVER == "2") {
    TARGET_SERVER = "http://" + QString::fromStdString(Params().get("OPKRServerAPI"));
  } else {
    TARGET_SERVER = util::getenv("API_HOST", "http://opkr.tk:3000").c_str();
  }

  // set up API requests
  if (auto dongleId = getDongleId()) {
    QString url = TARGET_SERVER + "/v1/devices/" + *dongleId + "/owner";
    RequestRepeater *repeater = new RequestRepeater(this, url, "ApiCache_Owner", 12);
    QObject::connect(repeater, &RequestRepeater::requestDone, this, &PrimeUserWidget::replyFinished);
  }
}

void PrimeUserWidget::replyFinished(const QString &response) {
  QJsonDocument doc = QJsonDocument::fromJson(response.toUtf8());
  if (doc.isNull()) {
    qDebug() << "JSON Parse failed on getting points";
    return;
  }

  QJsonObject json = doc.object();
  QString points_str = QString::number(json["points"].toInt());
  QString username_str = json["opusername"].toString();
  if (username_str.length()) {
    username_str = "@" + username_str;
  }

  opusername->setText(username_str);
  points->setText(points_str);
}

PrimeAdWidget::PrimeAdWidget(QWidget* parent) : QFrame(parent) {
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setMargin(30);
  main_layout->setSpacing(15);

  main_layout->addWidget(new QLabel("OPKR"), 1, Qt::AlignCenter);

  QPixmap hkgpix("../assets/addon/img/hkg.png");
  QLabel *hkg = new QLabel();
  hkg->setPixmap(hkgpix.scaledToWidth(430, Qt::SmoothTransformation));
  hkg->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
  main_layout->addWidget(hkg, 0, Qt::AlignCenter);

  setStyleSheet(R"(
    PrimeAdWidget {
      border-radius: 10px;
      background-color: #333333;
    }
  )");
}


SetupWidget::SetupWidget(QWidget* parent) : QFrame(parent) {
  mainLayout = new QStackedWidget;

  // Unpaired, registration prompt layout

  QWidget* finishRegistration = new QWidget;
  QVBoxLayout* finishRegistationLayout = new QVBoxLayout(finishRegistration);
  finishRegistationLayout->setMargin(0);
  finishRegistationLayout->setSpacing(0);

  QLabel* opkr = new QLabel("OPKR");
  opkr->setStyleSheet("font-size: 85px;"); // TODO: fit width
  finishRegistationLayout->addWidget(opkr, 0, Qt::AlignCenter);
  finishRegistationLayout->addSpacing(35);
  QPixmap hkgpix("../assets/addon/img/hkg.png");
  QLabel *hkg = new QLabel();
  hkg->setPixmap(hkgpix.scaledToWidth(450, Qt::SmoothTransformation));
  hkg->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
  finishRegistationLayout->addWidget(hkg, 0, Qt::AlignCenter);

  QPushButton* finishButton = new QPushButton(tr("Show QR Code"));
  finishButton->setFixedHeight(100);
  finishButton->setStyleSheet(R"(
    border-radius: 30px;
    font-size: 45px;
    font-weight: 500;
    background: #585858;
  )");
  finishRegistationLayout->addWidget(finishButton);
  QObject::connect(finishButton, &QPushButton::clicked, this, &SetupWidget::showQrCode);

  mainLayout->addWidget(finishRegistration);

  // Pairing QR code layout

  QWidget* q = new QWidget;
  QVBoxLayout* qrLayout = new QVBoxLayout(q);

  qrLayout->addSpacing(40);
  QLabel* qrLabel = new QLabel(tr("Scan Device!"));
  qrLabel->setWordWrap(true);
  qrLabel->setAlignment(Qt::AlignHCenter);
  qrLabel->setStyleSheet(R"(
    font-size: 45px;
    font-weight: 400;
  )");
  qrLayout->addWidget(qrLabel, 0, Qt::AlignTop);

  qrLayout->addWidget(new PairingQRWidget, 1);

  mainLayout->addWidget(q);

  primeAd = new PrimeAdWidget;
  mainLayout->addWidget(primeAd);

  primeUser = new PrimeUserWidget;
  mainLayout->addWidget(primeUser);

  mainLayout->setCurrentWidget(primeAd);

  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->addWidget(mainLayout);

  setStyleSheet(R"(
    SetupWidget {
      background-color: #333333;
      border-radius: 10px;
    }
    * {
      font-size: 90px;
      font-weight: 500;
      border-radius: 10px;
    }
  )");

  // Retain size while hidden
  QSizePolicy sp_retain = sizePolicy();
  sp_retain.setRetainSizeWhenHidden(true);
  setSizePolicy(sp_retain);

  // set up API requests
  QString OPKR_SERVER = QString::fromStdString(Params().get("OPKRServer"));
  QString TARGET_SERVER = "";
  if (OPKR_SERVER == "0") {
    TARGET_SERVER = util::getenv("API_HOST", "http://opkr.tk:3000").c_str();
  } else if (OPKR_SERVER == "1") {
    TARGET_SERVER = util::getenv("API_HOST", "https://api.commadotai.com").c_str();
  } else if (OPKR_SERVER == "2") {
    TARGET_SERVER = "http://" + QString::fromStdString(Params().get("OPKRServerAPI"));
  } else {
    TARGET_SERVER = util::getenv("API_HOST", "http://opkr.tk:3000").c_str();
  }

  if (auto dongleId = getDongleId()) {
    QString url = TARGET_SERVER + "/v1.1/devices/" + *dongleId + "/";
    RequestRepeater* repeater = new RequestRepeater(this, url, "ApiCache_Device", 10);

    QObject::connect(repeater, &RequestRepeater::requestDone, this, &SetupWidget::replyFinished);
  }
  hide(); // Only show when first request comes back
}

void SetupWidget::showQrCode() {
  showQr = true;
  mainLayout->setCurrentIndex(1);
}

void SetupWidget::replyFinished(const QString &response, bool success) {
  show();
  if (!success) return;

  QJsonDocument doc = QJsonDocument::fromJson(response.toUtf8());
  if (doc.isNull()) {
    qDebug() << "JSON Parse failed on getting pairing and prime status";
    return;
  }

  QJsonObject json = doc.object();
  if (!json["is_paired"].toBool() && !showQr) {
    mainLayout->setCurrentIndex(0);
  } else if (!json["is_paired"].toBool() && showQr) {
    mainLayout->setCurrentIndex(1);
  } else {
    bool prime = json["prime"].toBool();

    if (prime) {
      mainLayout->setCurrentWidget(primeUser);
    } else {
      mainLayout->setCurrentWidget(primeAd);
    }
  }
}
