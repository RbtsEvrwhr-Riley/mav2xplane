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
 * @file QGCXPlaneLink.h
 *   @brief X-Plane simulation link
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#ifndef QGCXPLANESIMULATIONLINK_H
#define QGCXPLANESIMULATIONLINK_H

#include <QString>
#include <QList>
#include <QMap>
#include <QMutex>
#include <QUdpSocket>
#include <QTimer>
#include <QProcess>
#include <QThread>
#include "UAS.h"


class QGCXPlaneLink : public QThread
{
    Q_OBJECT

	int throttles = 8;
	int throttleMap[16] = {4,0,1,2,3,8,8,8,8,8,8,8,8,8,8,8};
	int controlMap[3] = {7, 5, 8};
    bool _sensorHilEnabled;
    bool xPlaneConnected;
    unsigned int xPlaneVersion;
    quint64 simUpdateLast;
    quint64 simUpdateFirst;
    quint64 simUpdateLastText;
    quint64 simUpdateHz;
    quint64 simUpdateLastGroundTruth;
    bool _useHilActuatorControls;

public:
	QGCXPlaneLink();
    QGCXPlaneLink(QString remoteHost, quint16 remotePort);
    ~QGCXPlaneLink();

    bool isConnected();
    qint64 bytesAvailable();
    int getPort() const {
        return localPort;
    }   


    void run();
    void setRemoteHost(QString host);
    void enableHilActuatorControls(bool enable);
    void sendDataRef(QString ref, float value);
	void updateActuatorControls(quint64 time, quint64 flags, float* controls, quint8 mode);
	void setup(QString remoteHost, quint16 localPort);
    //void updateActuatorControls(quint64 time, quint64 flags, float ctl_0, float ctl_1, float ctl_2, float ctl_3, float ctl_4, float ctl_5, float ctl_6, float ctl_7, float ctl_8, float ctl_9, float ctl_10, float ctl_11, float ctl_12, float ctl_13, float ctl_14, float ctl_15, quint8 mode);



public slots:

    /** @brief Send new control states to the simulation */
    void updateControls(quint64 time, float rollAilerons, float pitchElevator, float yawRudder, float throttle, quint8 systemMode, quint8 navMode);
    /** @brief Set the simulator version as text string */


    void readBytes();
    /**
     * @brief Write a number of bytes to the interface.
     *
     * @param data Pointer to the data byte array
     * @param size The size of the bytes array
     **/
    void writeBytes(const char* data, qint64 length);

    void setSensorHILEnabled(bool enabled){_sensorHilEnabled = enabled;}
	
	/**
	* @brief Sets the X-Plane channels 1-16 (by index) to the selected MAVLink channels.
	* @param channels pointer to the array mapping channels
	**/
	void setThrottleMap(int xChan, int mavChan);
	void setControlsMap(int xChan, int mavChan);
	void getChannelMapFromPreferences();
    void stop();

protected:    

    QHostAddress localHost;
    quint16 localPort;
    QHostAddress remoteHost;
    quint16 remotePort;

    QUdpSocket* socket;
    bool connectState,_should_exit;

    float roll, pitch, yaw, rollspeed, pitchspeed, yawspeed;
    double lat, lon, alt, alt_agl;
    float vx, vy, vz, xacc, yacc, zacc;
    float ind_airspeed;
    float true_airspeed;
    float groundspeed;
    float xmag, ymag, zmag, abs_pressure, diff_pressure, pressure_alt, temperature;
    float barometerOffsetkPa;

    float man_roll, man_pitch, man_yaw;

signals:
    void hilStateChanged(quint64 time_us, float roll, float pitch, float yaw, float rollspeed,
                                          float pitchspeed, float yawspeed, double lat, double lon, double alt,
                                          float vx, float vy, float vz, float ind_airspeed, float true_airspeed, float xacc, float yacc, float zacc);

    void hilGroundTruthChanged(quint64 time_us, float roll, float pitch, float yaw, float rollspeed,
                              float pitchspeed, float yawspeed, double lat, double lon, double alt,
                              float vx, float vy, float vz, float ind_airspeed, float true_airspeed, float xacc, float yacc, float zacc);

    void sensorHilGpsChanged(quint64 time_us, double lat, double lon, double alt, int fix_type, float eph, float epv, float vel, float vn, float ve, float vd, float cog, int satellites);

    void sensorHilRawImuChanged(quint64 time_us, float xacc, float yacc, float zacc,
                                                  float xgyro, float ygyro, float zgyro,
                                                  float xmag, float ymag, float zmag,
                                                  float abs_pressure, float diff_pressure,
                                                  float pressure_alt, float temperature,
                                                  quint32 fields_updated);


    void hostConnected(QString hostAddress);

};

#endif // QGCXPLANESIMULATIONLINK_H

