#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "QGC/XPlaneLink.h"
#include <QSpinBox>
#include <QLabel>
#include <QTableWidget>
#include "indicator/indicator.h"

class ConnectionWidget;
class QGridLayout;
class QCheckBox;
class QPushButton;
class QComboBox;


class MainWindow : public QMainWindow
{
    Q_OBJECT

       bool px4_connected, xplane_connected;
       QGCXPlaneLink *link;
       UAS *uas;

       ConnectionWidget *xplane_connection, *px4_connection;
       QWidget *centralWidget;
	   QWidget *channelWindow;
	   QTableWidget *servoTable,*chanTable;
       QGridLayout *grid;
       QPushButton *startBtn, *enableHilBtn, *openChannelWindowButton;//*armBtn, *takeoffBtn, *enableHilBtn, *setModeBtn;
       QComboBox *modeBox;

       QString remoteHost, portName;

       Indicator *armIndicator;
       QLabel *mode;
       bool armed;
       bool connected;
       bool hilEnabled;


public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();
	
signals:
	void changeFlightMode(QString text);
	void takeoff();
	void toggleArmed(bool armed);

private slots:
	void saveChannels();
	void updateChanQGCLabel(int row, int col);
	void updateServoQGCLabel(int row, int col);
	void closeChannels();
    void onXplaneConnected(QString host);
    void onPx4Connected();
    void onStartClicked();
	void onOpenChannelWindowClicked();
    void onPortError(QString error);
    void onArmClicked();
    void onTakeoffClicked();
    void setMode();

    void armStayChanged(int armed);
    void flightModeChanged(QString mode);

    void enableHil();
    void stopSimulation();
    void enableButtons(bool enable);

};

#endif // MAINWINDOW_H
