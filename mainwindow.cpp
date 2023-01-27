#include "mainwindow.h"
#include "connectionwidget/connectionwidget.h"
#include <QGridLayout>
#include <QPushButton>
#include <QCheckBox>
#include <QDebug>
#include <QMessageBox>
#include <QComboBox>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QSettings>
/*
const QStringList px4modes = {"Manual",
                              "Altitude",
                              "Position",
                              "Mission",
                              "Hold",
                              "Takeoff",
                              "Land",
                              "Return",
                              "Acro",
                              "Offboard",
                              "Stabilized",
                              "Rattitude",
                              "Follow Me",
                              "Return to Groundstation",
                              "Ready",
                              "Simple"};
*/
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),px4_connected(true),xplane_connected(true),link(nullptr),hilEnabled(false)
{  
    centralWidget = new QWidget();
    grid = new QGridLayout;
    xplane_connection = new ConnectionWidget(0);
    xplane_connection->setColor(Qt::red);    

    px4_connection = new ConnectionWidget(1);
    px4_connection->setColor(Qt::red);

    grid->addWidget(px4_connection,0,0,1,1);
    grid->addWidget(xplane_connection,0,1,1,1);

    startBtn = new QPushButton(tr("Start"));
    startBtn->setEnabled(true);
    connect(startBtn,&QPushButton::clicked,this,&MainWindow::onStartClicked);

    QGroupBox *vbox = new QGroupBox;
    QVBoxLayout *l = new QVBoxLayout;
	/*
    armBtn = new QPushButton(tr("ARM"));
    connect(armBtn,&QPushButton::clicked, this, &MainWindow::onArmClicked);


    armIndicator = new Indicator("ARM STATUS");
    l->addWidget(armIndicator);
    l->addWidget(armBtn);*/
    vbox->setLayout(l);

    //grid->addWidget(vbox,1,0,2,1);
/*
    QGroupBox *box = new QGroupBox;

    QGridLayout *grid2 = new QGridLayout;

    mode = new QLabel("FLIGHT MODE");
    mode->setAlignment(Qt::AlignCenter);

    grid2->addWidget(mode,0,0,1,2);

    takeoffBtn = new QPushButton(tr("Takeoff"));
    connect(takeoffBtn,&QPushButton::clicked, this, &MainWindow::onTakeoffClicked);

    grid2->addWidget(takeoffBtn,2,0,1,2);

    modeBox = new QComboBox;
    modeBox->addItems(px4modes);
	*/
    grid->addWidget(startBtn,5,0,1,3);

    enableHilBtn = new QPushButton(tr("Enable HIL"));

    connect(enableHilBtn,SIGNAL(released()),this,SLOT(enableHil()));
    grid->addWidget(enableHilBtn,4,0,1,3);

/*    setModeBtn = new QPushButton(tr("Set mode"));
    connect(setModeBtn,SIGNAL(released()),this,SLOT(setMode()));
    grid2->addWidget(setModeBtn,1,0,1,1);

    grid2->addWidget(modeBox,1,1,1,1);

    box->setLayout(grid2);
*/
    //grid->addWidget(box,2,1,1,1);

    centralWidget->setLayout(grid);
    setCentralWidget(centralWidget);
	
	openChannelWindowButton = new QPushButton(tr("Configure MAVLink Channels"));
	connect(openChannelWindowButton, &QPushButton::clicked, this, &MainWindow::onOpenChannelWindowClicked);
	grid->addWidget(openChannelWindowButton, 1,0,1,2);

    armed = false;
    connected = false;

    uas = nullptr;
	link = new QGCXPlaneLink();
    enableButtons(false);
}

MainWindow::~MainWindow()
{
    delete grid;
    if(link)
    {
        link->deleteLater();
    }
}

void MainWindow::enableHil()
{
    if(uas)
        uas->setHilMode(hilEnabled = !hilEnabled);
    enableHilBtn->setText((hilEnabled)?("Disable HIL"):("Enable HIL"));
}

void MainWindow::setMode()
{
	/*
    if(uas)
        uas->setFlightMode(modeBox->currentText());
	*/
	emit changeFlightMode(modeBox->currentText());
}

void MainWindow::armStayChanged(int armed)
{
	/*
    (armed)?armIndicator->setColor(Qt::green):armIndicator->setColor(Qt::red);
    (armed)?armIndicator->setText("ARMED"):armIndicator->setText("DISARMED");
    (armed)?armBtn->setText("DISARM"):armBtn->setText("ARM");*/
}

void MainWindow::flightModeChanged(QString mode)
{
    this->mode->setText(mode);
}

void MainWindow::onPx4Connected()
{
    px4_connection->setColor(Qt::green);
}

void MainWindow::onXplaneConnected(QString host)
{
	//qDebug("Got an xplane connection");
    if(host.contains(remoteHost)){
        xplane_connection->setColor(Qt::green);
        link->setRemoteHost(host);
    }
    else xplane_connection->setColor(Qt::red);
}

void MainWindow::onStartClicked()
{
	qDebug("Clicked start/stop");
    if(connected)
        stopSimulation();

    else {
		/*
		TODO: figure out how to fix this because link needs to be an object for init conf
        if(link != nullptr)
        {
            link->terminate();
            link->wait();
            delete link;
        }	
*/
		// TODO these vars should be renamed
		qDebug("Starting UAS class");
		qDebug("%i", px4_connection->getPort());
        uas = new UAS(px4_connection->getName(),px4_connection->getPort());
		QSettings settings("Robots Everywhere", "Mav2Xplane");
		settings.setValue("px4ip", px4_connection->getName());
		settings.setValue("px4port", px4_connection->getPort());
        connect(uas,&UAS::hostConnected,this,&MainWindow::onPx4Connected);
        //connect(uas,&UAS::armStayChanged,this,&MainWindow::armStayChanged);
        //connect(uas,&UAS::flightModeChanged,this,&MainWindow::flightModeChanged);
		
		//connect(this,&MainWindow::toggleArmed,uas,&UAS::setArmed);
		
	    uas->start();
		qDebug("Starting QGCXplaneLink");
        link -> setup(xplane_connection->getName(),xplane_connection->getPort());
		settings.setValue("xplaneip", xplane_connection->getName());
		settings.setValue("xplaneport", xplane_connection->getPort());
        remoteHost = xplane_connection->getName();
        connect(link,&QGCXPlaneLink::hostConnected,this,&MainWindow::onXplaneConnected);
        connect(uas, &UAS::hilControlsChanged,link,&QGCXPlaneLink::updateControls,Qt::QueuedConnection);

        connect(link, &QGCXPlaneLink::hilStateChanged, uas, &UAS::sendHilState,Qt::QueuedConnection);
        connect(link, &QGCXPlaneLink::sensorHilGpsChanged, uas, &UAS::sendHilGps,Qt::QueuedConnection);
        connect(link, &QGCXPlaneLink::sensorHilRawImuChanged,uas, &UAS::sendHilSensors,Qt::QueuedConnection);
        connect(uas, &UAS::hilActuatorControlsChanged,link,&QGCXPlaneLink::updateActuatorControls);		

        link->start();

        enableButtons(true);
    }

    connected = !connected;
    connected?startBtn->setText("Stop"):startBtn->setText("Start");
}

void MainWindow::onPortError(QString error)
{
    QMessageBox msg(QMessageBox::Warning,tr("Error"),error);
    msg.exec();

    px4_connection->setColor(Qt::red);
}

void MainWindow::onArmClicked()
{
    armed = !armed;
	/*
    if(uas)
        uas->setArmed(armed);
	*/
	qDebug("Emitting toggle armed");
	emit toggleArmed(armed);
}

void MainWindow::onTakeoffClicked()
{
	/*
    if(uas)
        uas->takeoff();
	*/
	emit takeoff();
}

void MainWindow::stopSimulation()
{
    link->stop();
    uas->stop();
	link -> wait();
	uas -> wait();

    disconnect(link,&QGCXPlaneLink::hostConnected,this,&MainWindow::onXplaneConnected);
    disconnect(uas, &UAS::hilControlsChanged,link,&QGCXPlaneLink::updateControls);

    disconnect(link, &QGCXPlaneLink::hilStateChanged, uas, &UAS::sendHilState);
    disconnect(link, &QGCXPlaneLink::sensorHilGpsChanged, uas, &UAS::sendHilGps);
    disconnect(link, &QGCXPlaneLink::sensorHilRawImuChanged,uas, &UAS::sendHilSensors);
    disconnect(uas, &UAS::hilActuatorControlsChanged,link,&QGCXPlaneLink::updateActuatorControls);


    px4_connection->setColor(Qt::red);
    xplane_connection->setColor(Qt::red);
    //armIndicator->setColor(Qt::transparent);
    //armIndicator->setText("ARM STATUS");
    //mode->setText("FLIGHT MODE");
    enableButtons(false);
}

void MainWindow::enableButtons(bool enable)
{
    //armBtn->setEnabled(enable);
    enableHilBtn->setEnabled(enable);
    //takeoffBtn->setEnabled(enable);
    //setModeBtn->setEnabled(enable);
   // modeBox->setEnabled(enable);
}

void MainWindow::onOpenChannelWindowClicked()
{
	channelWindow = new QWidget();
	channelWindow -> resize(256,512);
    QVBoxLayout *l = new QVBoxLayout;
	channelWindow->setLayout(l);

	QLabel *throttle = new QLabel(tr("XPlane Throttles"));
	l -> addWidget(throttle);
	QLabel *sixteen = new QLabel(tr("0-15 for throttles, 16 for unused"));
	l -> addWidget(sixteen);
	chanTable = new QTableWidget(16, 2);
	QStringList topLabels;
	topLabels << "MAV Channel" << "QGC Channel";
	chanTable -> setHorizontalHeaderLabels(topLabels);
	l -> addWidget(chanTable);
	QLabel *servos = new QLabel(tr("XPlane Controls"));
	l -> addWidget(servos);
	QStringList servoLabels;
	servoLabels << "Pitch" << "Roll" << "Yaw";
	servoTable = new QTableWidget(3, 2);
	servoTable -> setHorizontalHeaderLabels(topLabels);
	servoTable -> setVerticalHeaderLabels(servoLabels);
	l -> addWidget(servoTable);

	
	QSettings settings("Robots Everywhere", "Mav2Xplane");
	settings.beginGroup("throttles");
	QColor color2(QColor("light grey"));
	connect(chanTable, SIGNAL(cellChanged(int, int)), this, SLOT(updateChanQGCLabel(int, int)));

	for(int x = 0; x < 16; x++)
	{
		chanTable -> setItem(x, 0, new QTableWidgetItem(settings.value(QString::number(x)).toString(), 0));
		chanTable -> item(x, 1) -> setFlags((chanTable -> item(x, 1)->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable));
		chanTable -> item(x, 1) -> setBackground(color2);

	}
	settings.endGroup();
	
	connect(servoTable, SIGNAL(cellChanged(int, int)), this, SLOT(updateServoQGCLabel(int, int)));

	settings.beginGroup("controls");
	for(int x = 0; x < 3; x++)
	{
		servoTable -> setItem(x, 0, new QTableWidgetItem(settings.value(QString::number(x)).toString(), 0));
		servoTable -> item(x, 1) -> setFlags((servoTable -> item(x, 1)->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable));
		servoTable -> item(x, 1) -> setBackground(color2);
	}
	settings.endGroup();
	
	QPushButton *doneButton = new QPushButton(tr("Save"));
	l-> addWidget(doneButton);
	connect(doneButton, &QPushButton::clicked, this, &MainWindow::saveChannels);
	QPushButton *cancelButton = new QPushButton(tr("Cancel"));
	connect(cancelButton, &QPushButton::clicked, this, &MainWindow::closeChannels);
	l-> addWidget(cancelButton);
	channelWindow -> show();
}

void MainWindow::updateChanQGCLabel(int row, int col)
{
	if(col == 0)
	{
		if(!chanTable -> item(row, 0) -> text().isNull() && !chanTable -> item(row, 0) -> text().isEmpty())
		{
			if((chanTable ->item(row, col) -> text().toInt()) < 16)
			{
				chanTable -> setItem(row, 1, new QTableWidgetItem(QString::number((chanTable ->item(row, col) -> text()).toInt() + 1)));
			}
			else
			{
				chanTable -> setItem(row, 1, new QTableWidgetItem(tr("Unused")));
			}
		}
	}
}
void MainWindow::updateServoQGCLabel(int row, int col)
{
	if(col == 0)
	{
		if(!servoTable -> item(row, 0) -> text().isNull() && !servoTable -> item(row, 0) -> text().isEmpty())
		{
			if(servoTable ->item(row, col) -> text().toInt() < 16)
			{
				servoTable -> setItem(row, 1, new QTableWidgetItem(QString::number((servoTable ->item(row, col) -> text()).toInt() + 1)));
			}
			else
			{
				servoTable -> setItem(row, 1, new QTableWidgetItem("Unused"));
			}
		}
	}
}
void MainWindow::saveChannels()
{
	// save them in a preferences file
	QSettings settings("Robots Everywhere", "Mav2Xplane");
	settings.beginGroup("throttles");
	for(long x = 0; x < chanTable -> rowCount(); x++)
	{
		if(!(chanTable -> item(x, 0) -> text().isNull()))
		{
			settings.setValue(QString::number(x), chanTable -> item(x, 0) -> text());
		}
	}
	settings.endGroup();
	settings.beginGroup("controls");
	for(long x = 0; x < servoTable -> rowCount(); x++)
	{
		if(!(servoTable -> item(x, 0) -> text().isNull()))
		{
			settings.setValue(QString::number(x), servoTable -> item(x, 0) -> text());
		}
	}
	settings.endGroup();
	link -> getChannelMapFromPreferences();
	channelWindow -> close();
}
void MainWindow::closeChannels()
{
	channelWindow -> close();
}