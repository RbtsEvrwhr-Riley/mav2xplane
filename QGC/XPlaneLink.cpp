/*=====================================================================

QGroundControl Open Source Ground Control Station

(c) 2009 - 2011 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>

This file is part of the QGROUNDCONTROL project

    QGROUNDCONTROL is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    QGROUNDCONTROL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with QGROUNDCONTROL. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 * @file QGCXPlaneLink.cc
 *   Implementation of X-Plane interface
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#include <QTimer>
#include <QList>
#include <QDebug>
#include <QMutexLocker>
#include <QNetworkInterface>
#include <QHostInfo>
#include <QCoreApplication>
#include <iostream>
#include "libs/eigen/Eigen/Eigen"
#include <QDateTime>
#include <QSettings>
#include "XPlaneLink.h"
#include "UAS.h"

QGCXPlaneLink::QGCXPlaneLink()
{
	xPlaneVersion = 11; // TODO: Change me

	if(xPlaneVersion > 11)
	{
		throttles = 16;
	}
	else
	{
		throttles = 8;
	}
}
QGCXPlaneLink::QGCXPlaneLink(QString remoteHost, quint16 localPort)
{
	setup(remoteHost, localPort);
}

void QGCXPlaneLink::setup(QString remoteHost, quint16 localPort)
{
	this->remoteHost = remoteHost;
    remotePort = 49000;
    socket = NULL;
    barometerOffsetkPa = -8.0f;
    simUpdateLast = 0;
    _sensorHilEnabled = true;
    _should_exit = false;
    _useHilActuatorControls = true;

    moveToThread(this);

    setTerminationEnabled(false);

    this->localHost = localHost;
    this->localPort = localPort;
	connectState = false;
}

QGCXPlaneLink::~QGCXPlaneLink()
{
    // Tell the thread to exit
    _should_exit = true;

    if (socket) {
        disconnect(socket, &QUdpSocket::readyRead, this, &QGCXPlaneLink::readBytes);
        socket->close();
        socket->deleteLater();
        socket = NULL;
    }
	// it should naturally stop
}

/**
 * @brief Runs the thread
 *
 **/



void QGCXPlaneLink::run()
{
    if (connectState) {
        return;
    }

    socket = new QUdpSocket(this);
    socket->moveToThread(this);
    connectState = socket->bind(localHost, localPort, QAbstractSocket::ReuseAddressHint);
    if (!connectState) {
        socket->deleteLater();
        socket = NULL;
        return;
    }
    QObject::connect(socket, &QUdpSocket::readyRead, this, &QGCXPlaneLink::readBytes,Qt::QueuedConnection);

	//qDebug("Opened socket");

    enableHilActuatorControls(true);

    // _should_exit = false;

    while(!_should_exit) {
        QCoreApplication::processEvents();
        msleep(2);
    }
	//qDebug("Closed xplane connection");
    connectState = false;

    disconnect(socket, &QUdpSocket::readyRead, this, &QGCXPlaneLink::readBytes);

    socket->close();
    socket->deleteLater();
    socket = NULL;
}

void QGCXPlaneLink::enableHilActuatorControls(bool enable)
{
    if (enable != _useHilActuatorControls) {
        _useHilActuatorControls = enable;
    }

    sendDataRef("sim/operation/override/override_control_surfaces", 0.0f);
}

void QGCXPlaneLink::updateActuatorControls(quint64 time, quint64 flags, float* controls, quint8 mode)
{	
    if (!_useHilActuatorControls) {
        qDebug() << "received HIL_ACTUATOR_CONTROLS but not using it";
        return;
    }
	//qDebug() << "received HIL_ACTUATOR_CONTROLS and trying to use it";
	//qDebug() << "P0" << controls[0] << "P1" << controls[1] << "P2" << controls[2] << "P3" << controls[3] << "THR" << controls[4] << "FR" << controls[5] << "FL" << controls[6] << "OTH" << controls[7] << "OTH" << controls[8] << "OTH" << controls[9] << "OTH" << controls[10] << "OTH" << controls[11] << "OTH" << controls[12] << "OTH" << controls[13] << "OTH" << controls[14] << "OTH" << controls[15];
    Q_UNUSED(time);
    Q_UNUSED(flags);
    Q_UNUSED(mode);
    Q_UNUSED(controls[4]);
    Q_UNUSED(controls[5]);
    Q_UNUSED(controls[6]);
    Q_UNUSED(controls[7]);
    Q_UNUSED(controls[8]);
    Q_UNUSED(controls[9]);
    Q_UNUSED(controls[10]);
    Q_UNUSED(controls[11]);
    Q_UNUSED(controls[12]);
    Q_UNUSED(controls[13]);
    Q_UNUSED(controls[14]);
    Q_UNUSED(controls[15]);

#pragma pack(push, 1)
    struct payload {
        char b[5];
        int index;
        float f[16]; // give it 16 we probably don't care
    } p;
#pragma pack(pop)

    p.b[0] = 'D';
    p.b[1] = 'A';
    p.b[2] = 'T';
    p.b[3] = 'A';
    p.b[4] = '\0';

    // Initialize with zeroes
    memset(p.f, 0, sizeof(p.f));

    // direct pass-through, normal fixed-wing.
    p.f[0] = controls[controlMap[0]];        ///< X-Plane Elevator
    p.f[1] = -controls[controlMap[1]];         ///< X-Plane Aileron
    p.f[2] = controls[controlMap[2]];  // THERE IS NO RUDDER ON THIS BIRD       ///< X-Plane Rudder

    // Send to group 8, which equals manual controls
    p.index = 8;
    writeBytes((const char*)&p, sizeof(p));

    // Send throttle to all eight motors
	// QUAD THROTTLE
    p.index = 25;
    p.f[0] = controls[throttleMap[0]]; // right motor
    p.f[1] = controls[throttleMap[1]]; // quads
    p.f[2] = controls[throttleMap[2]];
    p.f[3] = controls[throttleMap[3]];
    p.f[4] = controls[throttleMap[4]];
    p.f[5] = controls[throttleMap[5]];
	p.f[6] = controls[throttleMap[6]];
	p.f[7] = controls[throttleMap[7]];
	if(xPlaneVersion > 11)
	{
		p.f[8] = controls[throttleMap[8]];
		p.f[9] = controls[throttleMap[9]];
		p.f[10] = controls[throttleMap[10]];
		p.f[11] = controls[throttleMap[11]];
		p.f[12] = controls[throttleMap[12]];
		p.f[13] = controls[throttleMap[13]];
		p.f[14] = controls[throttleMap[14]];
		p.f[15] = controls[throttleMap[15]];
	}
    writeBytes((const char*)&p, sizeof(p));
}

void QGCXPlaneLink::updateControls(quint64 time, float rollAilerons, float pitchElevator, float yawRudder, float throttle, quint8 systemMode, quint8 navMode)
{
qDebug() <<"updateControls: "<<  rollAilerons << pitchElevator << yawRudder << throttle << systemMode<<  navMode;
#pragma pack(push, 1)
    struct payload {
        char b[5];
        int index;
        float f[8];
    } p;
#pragma pack(pop)

    p.b[0] = 'D';
    p.b[1] = 'A';
    p.b[2] = 'T';
    p.b[3] = 'A';
    p.b[4] = '\0';

    Q_UNUSED(time);
    Q_UNUSED(systemMode);
    Q_UNUSED(navMode);

    // direct pass-through, normal fixed-wing.
    p.f[0] = -pitchElevator;
    p.f[1] = rollAilerons;
    p.f[2] = yawRudder;



    // Ail / Elevon / Rudder
    p.index = 12;   // XPlane, wing sweep
    writeBytes((const char*)&p, sizeof(p));

    p.index = 8;    // XPlane, joystick? why?
    writeBytes((const char*)&p, sizeof(p));

    p.index = 25;   // Thrust
    memset(p.f, 0, sizeof(p.f));
    p.f[0] = throttle;
    p.f[1] = throttle;
    p.f[2] = throttle;
    p.f[3] = throttle;

    // Throttle
    writeBytes((const char*)&p, sizeof(p));


}

Eigen::Matrix3f euler_to_wRo(double yaw, double pitch, double roll) {
    double c__ = cos(yaw);
    double _c_ = cos(pitch);
    double __c = cos(roll);
    double s__ = sin(yaw);
    double _s_ = sin(pitch);
    double __s = sin(roll);
    double cc_ = c__ * _c_;
    double cs_ = c__ * _s_;
    double sc_ = s__ * _c_;
    double ss_ = s__ * _s_;
    double c_c = c__ * __c;
    double c_s = c__ * __s;
    double s_c = s__ * __c;
    double s_s = s__ * __s;
    double _cc = _c_ * __c;
    double _cs = _c_ * __s;
    double csc = cs_ * __c;
    double css = cs_ * __s;
    double ssc = ss_ * __c;
    double sss = ss_ * __s;
    Eigen::Matrix3f wRo;
    wRo <<
           cc_  , css-s_c,  csc+s_s,
            sc_  , sss+c_c,  ssc-c_s,
            -_s_  ,     _cs,      _cc;
    return wRo;
}

void QGCXPlaneLink::sendDataRef(QString ref, float value)
{
#pragma pack(push, 1)
    struct payload {
        char b[5];
        float value;
        char name[500];
    } dref;
#pragma pack(pop)

    dref.b[0] = 'D';
    dref.b[1] = 'R';
    dref.b[2] = 'E';
    dref.b[3] = 'F';
    dref.b[4] = '0';

    /* Set value */
    dref.value = value;

    /* Fill name with zeroes */
    memset(dref.name, 0, sizeof(dref.name));

    /* Set dref name */

    /* Send command */
    QByteArray ba = ref.toUtf8();
    if (ba.length() > 500) {
        return;
    }

    for (int i = 0; i < ba.length(); i++) {
        dref.name[i] = ba.at(i);
    }
    writeBytes((const char*)&dref, sizeof(dref));
}


void QGCXPlaneLink::writeBytes(const char* data, qint64 size)
{
	//qDebug("Writing to socket");
    if (!data) return;

    // If socket exists and is connected, transmit the data
    if (socket && connectState)
    {
        socket->writeDatagram(data, size, remoteHost, remotePort);
    }
}

/**
 * @brief Read all pending packets from the interface.
 **/

void QGCXPlaneLink::setRemoteHost(QString host)
{
    remoteHost = QHostAddress(host);
}

void QGCXPlaneLink::readBytes()
{
	//qDebug("Got some bytes");
    // Only emit updates on attitude message
    bool emitUpdate = false;
    quint16 fields_changed = 0;

    const qint64 maxLength = 65536;
    char data[maxLength];
    QHostAddress sender;
    quint16 senderPort;

    int s = socket->pendingDatagramSize();

    if (s > maxLength) std::cerr << __FILE__ << __LINE__ << " UDP datagram overflow, allowed to read less bytes than datagram size: " << s << std::endl;
    socket->readDatagram(data, maxLength, &sender, &senderPort);
    if (s > maxLength) {
        std::string headStr = std::string(data, data+5);
        std::cerr << __FILE__ << __LINE__ << " UDP datagram header: " << headStr << std::endl;
    }
	if(!sender.isNull())
	{
		emit hostConnected(sender.toString());
	}

	//qDebug("Host connected");
    // Calculate the number of data segments a 36 bytes
    // XPlane always has 5 bytes header: 'DATA@'
    unsigned nsegs = (s-5)/36;

    //qDebug() << "XPLANE:" << "LEN:" << s << "segs:" << nsegs;

#pragma pack(push, 1)
    struct payload {
        int index;
        float f[8];
    } p;
#pragma pack(pop)

    bool oldConnectionState = xPlaneConnected;

    if (data[0] == 'D' &&
            data[1] == 'A' &&
            data[2] == 'T' &&
            data[3] == 'A')
    {
        xPlaneConnected = true;

        if (oldConnectionState != xPlaneConnected) {
            simUpdateFirst = UAS::groundTimeMilliseconds();
        }

        for (unsigned i = 0; i < nsegs; i++)
        {
            // Get index
            unsigned ioff = (5+i*36);;
            memcpy(&(p), data+ioff, sizeof(p));
			//qDebug() << "packet";
            if (p.index == 3)
            {
                ind_airspeed = p.f[5] * 0.44704f;
                true_airspeed = p.f[6] * 0.44704f;
                groundspeed = p.f[7] * 0.44704;

                //qDebug() << "SPEEDS:" << "airspeed" << true_airspeed << "m/s, groundspeed" << groundspeed << "m/s";
            }
            if (p.index == 4)
            {
				//qDebug() << "VECTORS";
                // WORKAROUND: IF ground speed <<1m/s and altitude-above-ground <1m, do NOT use the X-Plane data, because X-Plane (tested
                // with v10.3 and earlier) delivers yacc=0 and zacc=0 when the ground speed is very low, which gives e.g. wrong readings
                // before launch when waiting on the runway. This might pose a problem for initial state estimation/calibration.
                // Instead, we calculate our own accelerations.
                if (fabsf(groundspeed)<0.1f && alt_agl<1.0)
                {
                    // TODO: Add centrip. acceleration to the current static acceleration implementation.
                    Eigen::Vector3f g(0, 0, -9.80665f);
                    Eigen::Matrix3f R = euler_to_wRo(yaw, pitch, roll);
                    Eigen::Vector3f gr = R.transpose().eval() * g;

                    xacc = gr[0];
                    yacc = gr[1];
                    zacc = gr[2];

                    //qDebug() << "Calculated values" << gr[0] << gr[1] << gr[2];
                }
                else
                {
                    // Accelerometer readings, directly from X-Plane and including centripetal forces.
                    const float one_g = 9.80665f;
                    xacc = p.f[5] * one_g;
                    yacc = p.f[6] * one_g;
                    zacc = -p.f[4] * one_g;

                    //qDebug() << "X-Plane values" << xacc << yacc << zacc;
                }

                fields_changed |= (1 << 0) | (1 << 1) | (1 << 2);
                emitUpdate = true;
            }
            // atmospheric pressure aircraft for XPlane 9 and 10
            else if (p.index == 6)
            {
				//qDebug() << "ENV";
                // inHg to hPa (hecto Pascal / millibar)
                abs_pressure = p.f[0] * 33.863886666718317f;
                temperature = p.f[1];
                fields_changed |= (1 << 9) | (1 << 12);
            }
            // Forward controls from X-Plane to MAV, not very useful
            // better: Connect Joystick to QGroundControl
            //            else if (p.index == 8)
            //            {
            //                ////qDebug() << "MAN:" << p.f[0] << p.f[3] << p.f[7];
            //                man_roll = p.f[0];
            //                man_pitch = p.f[1];
            //                man_yaw = p.f[2];
            //                UAS* uas = dynamic_cast<UAS*>(mav);
            //                if (uas) uas->setManualControlCommands(man_roll, man_pitch, man_yaw, 0.6);
            //            }
            else if ((xPlaneVersion >= 10 && p.index == 16) || (xPlaneVersion == 9 && p.index == 17))
            {
				//qDebug() << "SPEEDS";
                // Cross checked with XPlane flight
                pitchspeed = p.f[0];
                rollspeed = p.f[1];
                yawspeed = p.f[2];
                fields_changed |= (1 << 3) | (1 << 4) | (1 << 5);

                emitUpdate = true;
            }
            else if ((xPlaneVersion >= 10 && p.index == 17) || (xPlaneVersion == 9 && p.index == 18))
            {
                //qDebug() << "HDNG" << "pitch" << p.f[0] << "roll" << p.f[1] << "hding true" << p.f[2] << "hding mag" << p.f[3];
                pitch = p.f[0] / 180.0f * M_PI;
                roll = p.f[1] / 180.0f * M_PI;
                yaw = p.f[2] / 180.0f * M_PI;

                // X-Plane expresses yaw as 0..2 PI
                if (yaw > M_PI) {
                    yaw -= 2.0f * static_cast<float>(M_PI);
                }
                if (yaw < -M_PI) {
                    yaw += 2.0f * static_cast<float>(M_PI);
                }

                float yawmag = p.f[3] / 180.0f * M_PI;

                if (yawmag > M_PI) {
                    yawmag -= 2.0f * static_cast<float>(M_PI);
                }
                if (yawmag < -M_PI) {
                    yawmag += 2.0f * static_cast<float>(M_PI);
                }

                // Normal rotation matrix, but since we rotate the
                // vector [0.25 0 0.45]', we end up with these relevant
                // matrix parts.

                xmag = cos(-yawmag) * 0.25f;
                ymag = sin(-yawmag) * 0.25f;
                zmag = 0.45f;
                fields_changed |= (1 << 6) | (1 << 7) | (1 << 8);

                double cosPhi = cos(roll);
                double sinPhi = sin(roll);
                double cosThe = cos(pitch);
                double sinThe = sin(pitch);
                double cosPsi = cos(0.0);
                double sinPsi = sin(0.0);

                float dcm[3][3];

                dcm[0][0] = cosThe * cosPsi;
                dcm[0][1] = -cosPhi * sinPsi + sinPhi * sinThe * cosPsi;
                dcm[0][2] = sinPhi * sinPsi + cosPhi * sinThe * cosPsi;

                dcm[1][0] = cosThe * sinPsi;
                dcm[1][1] = cosPhi * cosPsi + sinPhi * sinThe * sinPsi;
                dcm[1][2] = -sinPhi * cosPsi + cosPhi * sinThe * sinPsi;

                dcm[2][0] = -sinThe;
                dcm[2][1] = sinPhi * cosThe;
                dcm[2][2] = cosPhi * cosThe;

                Eigen::Matrix3f m = Eigen::Map<Eigen::Matrix3f>((float*)dcm).eval();

                Eigen::Vector3f mag(xmag, ymag, zmag);

                Eigen::Vector3f magbody = m * mag;

                //                //qDebug() << "yaw mag:" << p.f[2] << "x" << xmag << "y" << ymag;
                //                //qDebug() << "yaw mag in body:" << magbody(0) << magbody(1) << magbody(2);

                xmag = magbody(0);
                ymag = magbody(1);
                zmag = magbody(2);

                // Rotate the measurement vector into the body frame using roll and pitch


                emitUpdate = true;
            }

            //            else if (p.index == 19)
            //            {
            //                //qDebug() << "ATT:" << p.f[0] << p.f[1] << p.f[2];
            //            }
            else if (p.index == 20)
            {
                //qDebug() << "LAT/LON/ALT:" << p.f[0] << p.f[1] << p.f[2];
                lat = p.f[0];
                lon = p.f[1];
                alt = p.f[2] * 0.3048f; // convert feet (MSL) to meters
                alt_agl = p.f[3] * 0.3048f; //convert feet (AGL) to meters
            }
            else if (p.index == 21)
            {
				//qDebug() << "21";
                vy = p.f[3];
                vx = -p.f[5];
                // moving 'up' in XPlane is positive, but its negative in NED
                // for us.
                vz = -p.f[4];
            }
            else if (p.index == 12)
            {
                //qDebug() << "AIL/ELEV/RUD" << p.f[0] << p.f[1] << p.f[2];
            }
            else if (p.index == 25)
            {
                //qDebug() << "THROTTLE" << p.f[0] << p.f[1] << p.f[2] << p.f[3];
            }
            else if (p.index == 0)
            {
                //qDebug() << "STATS" << "fgraphics/s" << p.f[0] << "fsim/s" << p.f[2] << "t frame" << p.f[3] << "cpu load" << p.f[4] << "grnd ratio" << p.f[5] << "filt ratio" << p.f[6];
            }
            else if (p.index == 11)
            {
                //qDebug() << "CONTROLS" << "ail" << p.f[0] << "elev" << p.f[1] << "rudder" << p.f[2] << "nwheel" << p.f[3];
            }
            else
            {
                //qDebug() << "UNKNOWN #" << p.index << p.f[0] << p.f[1] << p.f[2] << p.f[3];
            }
        }
    }
    else if (data[0] == 'S' &&
             data[1] == 'N' &&
             data[2] == 'A' &&
             data[3] == 'P')
    {

    }
    else if (data[0] == 'S' &&
             data[1] == 'T' &&
             data[2] == 'A' &&
             data[3] == 'T')
    {

    }
    else
    {
        //qDebug() << "UNKNOWN PACKET:" << data;
    }
	
	//qDebug() << "Checking UAS update";
    // Wait for 0.5s before actually using the data, so that all fields are filled
    if (UAS::groundTimeMilliseconds() - simUpdateFirst < 500) {
        return;
    }
	//qDebug() << "Emit update state";
    // Send updated state
    if (emitUpdate && (UAS::groundTimeMilliseconds() - simUpdateLast) > 2)
    {
        simUpdateHz = simUpdateHz * 0.9f + 0.1f * (1000.0f / (UAS::groundTimeMilliseconds() - simUpdateLast));
        if (UAS::groundTimeMilliseconds() - simUpdateLastText > 2000) {
            //emit statusMessage(tr("Receiving from XPlane at %1 Hz").arg(static_cast<int>(simUpdateHz)));
            // Reset lowpass with current value
            simUpdateHz = (1000.0f / (UAS::groundTimeMilliseconds() - simUpdateLast));
            // Set state
            simUpdateLastText = UAS::groundTimeMilliseconds();
        }

        simUpdateLast = UAS::groundTimeMilliseconds();

        if (_sensorHilEnabled)
        {
			//qDebug("Updating HIL sensors");
            diff_pressure = (ind_airspeed * ind_airspeed * 1.225f) / 2.0f;

            /* tropospheric properties (0-11km) for standard atmosphere */
            const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
            const double a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
            const double g  = 9.80665;	/* gravity constant in m/s/s */
            const double R  = 287.05;	/* ideal gas constant in J/kg/K */

            /* current pressure at MSL in kPa */
            double p1 = 1013.25 / 10.0;

            /* measured pressure in hPa, plus offset to simulate weather effects / offsets */
            double p = abs_pressure / 10.0 + barometerOffsetkPa;

            /*
             * Solve:
             *
             *     /        -(aR / g)     \
             *    | (p / p1)          . T1 | - T1
             *     \                      /
             * h = -------------------------------  + h1
             *                   a
             */
            pressure_alt = (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;

            // set pressure alt to changed
            fields_changed |= (1 << 11);

            emit sensorHilRawImuChanged(UAS::groundTimeUsecs(), xacc, yacc, zacc, rollspeed, pitchspeed, yawspeed,
                                        xmag, ymag, zmag, abs_pressure, diff_pressure / 100.0, pressure_alt, temperature, fields_changed);

            // XXX make these GUI-configurable and add randomness
            int gps_fix_type = 3;
            float eph = 0.3f;
            float epv = 0.6f;
            float vel = sqrt(vx*vx + vy*vy + vz*vz);
            float cog = atan2(vy, vx);
            int satellites = 8;

            emit sensorHilGpsChanged(UAS::groundTimeUsecs(), lat, lon, alt, gps_fix_type, eph, epv, vel, vx, vy, vz, cog, satellites);
        } else {
            emit hilStateChanged(UAS::groundTimeUsecs(), roll, pitch, yaw, rollspeed,
                                 pitchspeed, yawspeed, lat, lon, alt,
                                 vx, vy, vz, ind_airspeed, true_airspeed, xacc, yacc, zacc);
			//qDebug("Updating HIL state");
        }

        // Limit ground truth to 25 Hz
        if (UAS::groundTimeMilliseconds() - simUpdateLastGroundTruth > 40) {
            emit hilGroundTruthChanged(UAS::groundTimeUsecs(), roll, pitch, yaw, rollspeed,
                                       pitchspeed, yawspeed, lat, lon, alt,
                                       vx, vy, vz, ind_airspeed, true_airspeed, xacc, yacc, zacc);

            //simUpdateLastGroundTruth = UAS::groundTimeMilliseconds();
        }
    }

    if (!oldConnectionState && xPlaneConnected)
    {
        //emit statusMessage(tr("Receiving from XPlane."));
    }
}


/**
 * @brief Get the number of bytes to read.
 *
 * @return The number of bytes to read
 **/
qint64 QGCXPlaneLink::bytesAvailable()
{
    return socket->pendingDatagramSize();
}

void QGCXPlaneLink::stop()
{
    _should_exit = true;
}
void QGCXPlaneLink::setThrottleMap(int xChan, int mavChan)
{
	throttleMap[xChan] = mavChan;
}
void QGCXPlaneLink::setControlsMap(int xChan, int mavChan)
{
	controlMap[xChan] = mavChan;
}
/*
int* QGCXPlaneLink::getThrottleMap()
{
	return throttleMap;
}
int* QGCXPlaneLink::getControlsMap()
{
	return controlsMap;
}
*/
/** Loads the channel mappings from a Settings config
**/
void QGCXPlaneLink::getChannelMapFromPreferences()
{
	QSettings settings("Robots Everywhere", "Mav2Xplane");		
	settings.beginGroup("throttles");
	for(int x = 0; x < throttles; x++)
	{
		setThrottleMap(x, settings.value(QString::number(x)).toInt());
	}
	settings.endGroup();
	settings.beginGroup("controls");
	for(int x = 0; x < 3; x++)
	{
		setControlsMap(x, settings.value(QString::number(x)).toInt());
	}
	settings.endGroup();
}
