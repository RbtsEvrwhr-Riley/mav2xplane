#ifndef LINKDIALOG_H
#define LINKDIALOG_H

#include <QGroupBox>
#include <QColor>

class QLineEdit;
class QRadioButton;
class QPushButton;
class Indicator;
class QComboBox;


class ConnectionWidget : public QGroupBox
{
    Q_OBJECT
public:
    /* type = 0 - ethernet, 1 - com */
    ConnectionWidget(int type, QWidget *parent = 0);
    virtual ~ConnectionWidget(){}

    void setColor(const QColor &c);
    void setIP(const QString &ip);

    QString getName() const;
    quint16 getPort() const;
    int getBaud() const;


private slots:
    void updatePortList();

private:
    QLineEdit *ipEdit;
    QComboBox *comBox;
    QPushButton *btnApply;
    QComboBox *bauds;
    QLineEdit *port;
    Indicator *indr;
    int mType;
};

#endif // LINKDIALOG_H
