#pragma once

#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QString>
#include <QVBoxLayout>
#include <QWidget>
#include <QListWidget>

#include "selfdrive/ui/qt/widgets/keyboard.h"
#include "selfdrive/common/params.h"


class QDialogBase : public QDialog {
  Q_OBJECT

protected:
  QDialogBase(QWidget *parent);
  bool eventFilter(QObject *o, QEvent *e) override;

public slots:
  int exec() override;
};

class InputDialog : public QDialogBase {
  Q_OBJECT

public:
  explicit InputDialog(const QString &title, QWidget *parent, const QString &subtitle = "", bool secret = false);
  static QString getText(const QString &title, QWidget *parent, const QString &substitle = "",
                         bool secret = false, int minLength = -1, const QString &defaultText = "");
  QString text();
  void setMessage(const QString &message, bool clearInputField = true);
  void setMinLength(int length);
  void show();

private:
  int minLength;
  QLineEdit *line;
  Keyboard *k;
  QLabel *label;
  QLabel *sublabel;
  QVBoxLayout *main_layout;
  QPushButton *eye_btn;

private slots:
  void handleEnter();

signals:
  void cancel();
  void emitText(const QString &text);
};

class ConfirmationDialog : public QDialogBase {
  Q_OBJECT

public:
  explicit ConfirmationDialog(const QString &prompt_text, const QString &confirm_text,
                              const QString &cancel_text, QWidget* parent);
  static bool alert(const QString &prompt_text, QWidget *parent);
  static bool confirm(const QString &prompt_text, QWidget *parent);
};

// larger ConfirmationDialog for rich text
class RichTextDialog : public QDialogBase {
  Q_OBJECT

public:
  explicit RichTextDialog(const QString &prompt_text, const QString &btn_text, QWidget* parent);
  static bool alert(const QString &prompt_text, QWidget *parent);
};

class MultiOptionDialog : public QDialogBase {
  Q_OBJECT

public:
  explicit MultiOptionDialog(const QString &prompt_text, const QStringList &l, const QString &current, QWidget *parent);
  static QString getSelection(const QString &prompt_text, const QStringList &l, const QString &current, QWidget *parent);
  QString selection;
};

// UpdateInfoDialog
class UpdateInfoDialog : public QDialogBase {
  Q_OBJECT

public:
  explicit UpdateInfoDialog(const QString &prompt_text, const QString &confirm_text,
                            const QString &cancel_text, const QString &detail_text, QWidget* parent);
  static bool confirm(const QString &prompt_text, QWidget *parent);
};

// Git Pull Cancel
class GitPullCancel : public QDialogBase {
  Q_OBJECT

public:
  explicit GitPullCancel(const QString &confirm_text,
                            const QString &cancel_text, QWidget* parent);
  static bool confirm(QWidget *parent);

private:
  Params params;
};