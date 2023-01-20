#include <array>

#include <QLabel>
#include <QPixmap>
#include <QProgressBar>
#include <QSocketNotifier>
#include <QVariantAnimation>
#include <QWidget>
#include <QHostAddress>
#include <QNetworkInterface>
#include <QAbstractSocket>
#include <QElapsedTimer>

constexpr int spinner_fps = 30;
constexpr QSize spinner_size = QSize(360, 360);

class TrackWidget : public QWidget  {
  Q_OBJECT
public:
  TrackWidget(QWidget *parent = nullptr);

private:
  void paintEvent(QPaintEvent *event) override;
  std::array<QPixmap, spinner_fps> track_imgs;
  QVariantAnimation m_anim;
};

class Spinner : public QWidget {
  Q_OBJECT

public:
  explicit Spinner(QWidget *parent = 0);

private:
  QLabel *text;
  QProgressBar *progress_bar;
  QSocketNotifier *notifier;
  QLabel *ip_label;
  QString device_ip = "";
  QLabel *bt_label;
  QElapsedTimer btElapsed;

public slots:
  void update(int n);
};
