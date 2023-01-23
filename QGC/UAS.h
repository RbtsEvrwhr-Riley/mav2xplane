#ifndef UAS_H
#define UAS_H


#include <QString>
#include <QList>
#include <QMap>
#include <QMutex>
#include <QTimer>
#include <QThread>
#include <QObject>
#include <QDateTime>
#include <QUdpSocket>
#include "mavlink.h"

class UAS : public QThread
{
    Q_OBJECT

    QUdpSocket* socket;
    QHostAddress localHost;
    quint16 localPort = 12345;
    QHostAddress remoteHost;
    quint16 remotePort;
    int _baud;
    bool connectState,_should_exit;
    uint8_t _base_mode;
    uint32_t _custom_mode;
    quint64 lastSendTimeGPS;     ///< Last HIL GPS message sent
    bool _versionNotified;  ///< true: user notified over version issue

public:
    UAS(QString ipaddress,int port);
    ~UAS();
	void run();

    static const char* _manualFlightMode;
    static const char* _acroFlightMode;
    static const char* _stabilizedFlightMode;
    static const char* _rattitudeFlightMode;
    static const char* _altCtlFlightMode;
    static const char* _posCtlFlightMode;
    static const char* _offboardFlightMode;
    static const char* _readyFlightMode;
    static const char* _takeoffFlightMode;
    static const char* _holdFlightMode;
    static const char* _missionFlightMode;
    static const char* _rtlFlightMode;
    static const char* _landingFlightMode;
    static const char* _rtgsFlightMode;
    static const char* _followMeFlightMode;
    static const char* _simpleFlightMode;

protected:
    typedef struct {
        uint8_t     main_mode;
        uint8_t     sub_mode;
        QString     name;       ///< Name for flight mode
        bool        canBeSet;   ///< true: Vehicle can be set to this flight mode
        bool        fixedWing;  /// fixed wing compatible
        bool        multiRotor;  /// multi rotor compatible
    } FlightModeInfo_t;

    QList<FlightModeInfo_t> _flightModeInfoList;

public slots:
    void setHilMode(bool);
    void setArmed(bool);
    void takeoff();
    void requestAutopilotCapabilites(); // Запрос версии автопилота


    /** @brief Send the full HIL state to the MAV */
    void sendHilState(quint64 time_us, float roll, float pitch, float yaw, float rollspeed,
                      float pitchspeed, float yawspeed, double lat, double lon, double alt,
                      float vx, float vy, float vz, float ind_airspeed, float true_airspeed, float xacc, float yacc, float zacc);


    /** @brief RAW sensors for sensor HIL */
    void sendHilSensors(quint64 time_us, float xacc, float yacc, float zacc, float rollspeed, float pitchspeed, float yawspeed,
                        float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, quint32 fields_changed);



    float addZeroMeanNoise(float truth_meas, float noise_var);

    /**
     * @param time_us
     * @param lat
     * @param lon
     * @param alt
     * @param fix_type
     * @param eph
     * @param epv
     * @param vel
     * @param cog course over ground, in radians, -pi..pi
     * @param satellites
     */
    void sendHilGps(quint64 time_us, double lat, double lon, double alt, int fix_type, float eph, float epv, float vel, float vn, float ve, float vd,  float cog, int satellites);

    static quint64 groundTimeMilliseconds()
    {
        return static_cast<quint64>(QDateTime::currentMSecsSinceEpoch());
    }
    static quint64 groundTimeUsecs()
    {
        return groundTimeMilliseconds() * 1000;
    }

    void setFlightMode(QString modeName);
    QString flightMode(uint8_t base_mode, uint32_t custom_mode) const;
    /**
     * @brief Write a number of bytes to the interface.
     *
     * @param data Pointer to the data byte array
     * @param size The size of the bytes array
     **/
    void writeBytes(const uint8_t* data, qint64 length);
    void readData();
	void stop();
signals:
    void hilControlsChanged(quint64 time, float rollAilerons, float pitchElevator, float yawRudder, float throttle, quint8 systemMode, quint8 navMode);
    //void hilActuatorControlsChanged(quint64 time, quint64 flags, float ctl_0, float ctl_1, float ctl_2, float ctl_3, float ctl_4, float ctl_5, float ctl_6, float ctl_7, float ctl_8, float ctl_9, float ctl_10, float ctl_11, float ctl_12, float ctl_13, float ctl_14, float ctl_15, quint8 mode);
	void hilActuatorControlsChanged(quint64 time, quint64 flags, float* controls, quint8 mode);

    void armStayChanged(int armed);
    void flightModeChanged(QString);
	void hostConnected(QString hostAddress);
	//void writeExternal(const uint8_t* data, qint64 size);
	

private slots:
    void writeMessage(mavlink_message_t);
    bool _setFlightMode(const QString flightMode, uint8_t* base_mode=0, uint32_t* custom_mode=0);

    /**
     * @brief _handleAutopilotVersion - обработка пакета с версией автопилота
     * @param message - сообщение от автопилота
     */
    void _handleAutopilotVersion(mavlink_message_t* message);

};

#endif // UAS_H
