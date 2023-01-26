#include <QFormLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QComboBox>
#include <QSerialPortInfo>
#include "connectionwidget.h"
#include "../indicator/indicator.h"
#include <QDebug>
#include <QTimer>
#include <QSettings>

const QStringList baudList = {"57600","115200"};
QStringList availablePorts()
{
    QStringList ports;

    for (QSerialPortInfo port : QSerialPortInfo::availablePorts())
    {
        ports += port.portName() + " (" + port.description() + ")";
    }
    return ports;
}


ConnectionWidget::ConnectionWidget(int type, QWidget *parent)
    : QGroupBox(QObject::tr((type)?"PX4":"Xplane"),parent),mType(type)
{
	formLayout = new QFormLayout;
	comFormLayout = new QFormLayout;
	QSettings settings("Robots Everywhere", "Mav2Xplane");

	if(!type) {
		formLayout = new QFormLayout;

        ipEdit = new QLineEdit;
        ipEdit->setInputMask("000.000.000.000");

		ipEdit->setText(settings.value("xplaneip", "127.0.0.1").toString());
        formLayout->addRow(QObject::tr("IP:"), ipEdit);


        port = new QLineEdit("49005");
		port->setText(settings.value("xplaneport", "49005").toString());

        port->setInputMask("00000");

        formLayout->addRow(QObject::tr("port:"), port);
    }
	else
	{    
		formLayout = new QFormLayout;

		comButtons = new QGroupBox;
		comCom = new QRadioButton(tr("&COM"));
		comUdp = new QRadioButton(tr("&UDP"));
		comUdp -> setChecked(true);
		
		connect(comCom, SIGNAL(clicked()), this, SLOT(checkComRadios()));
		connect(comUdp, SIGNAL(clicked()), this, SLOT(checkComRadios()));
		
		QHBoxLayout *comLayout = new QHBoxLayout;
		comButtons -> setLayout(comLayout);
		comLayout -> addWidget(comCom);
		comLayout -> addWidget(comUdp);
		// ip
		formLayout -> addRow(comButtons);
		
		ipEdit = new QLineEdit;
        ipEdit->setInputMask("000.000.000.000");

        ipEdit->setText("127.0.0.1");
		ipEdit->setText(settings.value("px4ip", "127.0.0.1").toString());

        formLayout->addRow(QObject::tr("IP:"), ipEdit);

        port = new QLineEdit("14560");
		port->setText(settings.value("px4port", "49005").toString());

        port->setInputMask("00000");

        formLayout->addRow(QObject::tr("port:"), port);
		
		// com
		comBox = new QComboBox;
        comBox->addItems(availablePorts());
		
        comFormLayout->addRow(QObject::tr("COM:"),comBox);
		

        bauds = new QComboBox;
        bauds->addItems(baudList);
        bauds->setCurrentIndex(1);
        comFormLayout->addRow(QObject::tr("baud:"),bauds);
		comFormLayout -> itemAt(0) -> widget() -> setVisible(false);
		comFormLayout -> itemAt(1) -> widget() -> setVisible(false);

		comFormLayout -> itemAt(2) -> widget() -> setVisible(false);
		comFormLayout -> itemAt(3) -> widget() -> setVisible(false);

        QTimer::singleShot(10,this,SLOT(updatePortList()));	
	}


    indr = new Indicator(QObject::tr("Link"),this);

    QVBoxLayout *radioLayout = new QVBoxLayout;
    radioLayout->addWidget(indr);

    QVBoxLayout *layout = new QVBoxLayout;	
    layout->addLayout(formLayout);
	layout->addLayout(comFormLayout);
    layout->addLayout(radioLayout);
    layout->addStretch();
    setLayout(layout);
}

void ConnectionWidget::checkComRadios()
{
	if(comCom -> isChecked())
	{
		for(int x = 1; x < formLayout -> rowCount() * 2 - 1; x++)
		{
			formLayout -> itemAt(x) -> widget() -> setVisible(false);
		}
		for(int x = 0; x < comFormLayout -> rowCount() * 2; x++)
		{
			comFormLayout -> itemAt(x) -> widget() -> setVisible(true);
		}
	}
	else
	{
		for(int x = 1; x < formLayout -> rowCount() * 2 - 1; x++)
		{
			formLayout -> itemAt(x) -> widget() -> setVisible(true);
		}
		for(int x = 0; x < comFormLayout -> rowCount() * 2; x++)
		{
			comFormLayout -> itemAt(x) -> widget() -> setVisible(false);
		}
	}
}

QString ConnectionWidget::getName() const
{
    return ipEdit->text();
}

void ConnectionWidget::setColor(const QColor &c)
{
    indr->setColor(c);
}

void ConnectionWidget::setIP(const QString &ip)
{
    ipEdit->setText(ip);
}

void ConnectionWidget::updatePortList()
{
    comBox->clear();
    comBox->addItems(availablePorts());

    QTimer::singleShot(500,this,SLOT(updatePortList()));
}


int ConnectionWidget::getBaud() const
{
    return baudList.at(bauds->currentIndex()).toInt();
}

quint16 ConnectionWidget::getPort() const
{
    return port->text().toInt();
}


