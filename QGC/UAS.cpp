#include "UAS.h"
#include <QDateTime>
#include <qmath.h>
#include <QDebug>
#include <QTimer>
#include <QList>
#include <QMutexLocker>
#include <QNetworkInterface>
#include <QHostInfo>
#include <QCoreApplication>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <unistd.h>
#include "px4_custom_mode.h"

const char* UAS::_manualFlightMode =      "Manual";
const char* UAS::_altCtlFlightMode =      "Altitude";
const char* UAS::_posCtlFlightMode =      "Position";
const char* UAS::_missionFlightMode =     "Mission";
const char* UAS::_holdFlightMode =        "Hold";
const char* UAS::_takeoffFlightMode =     "Takeoff";
const char* UAS::_landingFlightMode =     "Land";
const char* UAS::_rtlFlightMode =         "Return";
const char* UAS::_acroFlightMode =        "Acro";
const char* UAS::_offboardFlightMode =    "Offboard";
const char* UAS::_stabilizedFlightMode =  "Stabilized";
const char* UAS::_rattitudeFlightMode =   "Rattitude";
const char* UAS::_followMeFlightMode =    "Follow Me";
const char* UAS::_rtgsFlightMode =        "Return to Groundstation";
const char* UAS::_readyFlightMode =       "Ready";
const char* UAS::_simpleFlightMode =      "Simple";

// we need this bad boy to connect to a remote listening socket in a SITL to send and receive mavlink messages.
UAS::UAS(QString remoteHost, int remotePort)
{
	//qDebug("Setting up socket");
    socket = NULL;
	_should_exit = false;

	this->remoteHost = remoteHost;
    this->remotePort = remotePort;
    connectState = false;
    moveToThread(this);

    setTerminationEnabled(false);

	//qDebug("Setting up struct");
    struct Modes2Name {
        uint8_t     main_mode;
        uint8_t     sub_mode;
        const char* name;       ///< Name for flight mode
        bool        canBeSet;   ///< true: Vehicle can be set to this flight mode
        bool        fixedWing;  /// fixed wing compatible
        bool        multiRotor;  /// multi rotor compatible
    };

    static const struct Modes2Name rgModes2Name[] = {
        //main_mode                         sub_mode                                name                                        canBeSet  FW      MC
    { PX4_CUSTOM_MAIN_MODE_MANUAL,      0,                                      UAS::_manualFlightMode,       true,   true,   true },
    { PX4_CUSTOM_MAIN_MODE_STABILIZED,  0,                                      UAS::_stabilizedFlightMode,   true,   true,   true },
    { PX4_CUSTOM_MAIN_MODE_ACRO,        0,                                      UAS::_acroFlightMode,         true,   true,   true },
    { PX4_CUSTOM_MAIN_MODE_RATTITUDE,   0,                                      UAS::_rattitudeFlightMode,    true,   true,   true },
    { PX4_CUSTOM_MAIN_MODE_ALTCTL,      0,                                      UAS::_altCtlFlightMode,       true,   true,   true },
    { PX4_CUSTOM_MAIN_MODE_POSCTL,      0,                                      UAS::_posCtlFlightMode,       true,   true,   true },
    { PX4_CUSTOM_MAIN_MODE_SIMPLE,      0,                                      UAS::_simpleFlightMode,       true,   false,  true },
    { PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_LOITER,        UAS::_holdFlightMode,         true,   true,   true },
    { PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_MISSION,       UAS::_missionFlightMode,      true,   true,   true },
    { PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_RTL,           UAS::_rtlFlightMode,          true,   true,   true },
    { PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET, UAS::_followMeFlightMode,     true,   false,  true },
    { PX4_CUSTOM_MAIN_MODE_OFFBOARD,    0,                                      UAS::_offboardFlightMode,     true,   false,  true },
    // modes that can't be directly set by the user
    { PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_LAND,          UAS::_landingFlightMode,      false,  true,   true },
    { PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_READY,         UAS::_readyFlightMode,        false,  true,   true },
    { PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_RTGS,          UAS::_rtgsFlightMode,         false,  true,   true },
    { PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,       UAS::_takeoffFlightMode,      false,  true,   true },
};

    // Convert static information to dynamic list. This allows for plugin override class to manipulate list.
    for (size_t i=0; i<sizeof(rgModes2Name)/sizeof(rgModes2Name[0]); i++) {
        const struct Modes2Name* pModes2Name = &rgModes2Name[i];

        FlightModeInfo_t info;

        info.main_mode =    pModes2Name->main_mode;
        info.sub_mode =     pModes2Name->sub_mode;
        info.name =         pModes2Name->name;
        info.canBeSet =     pModes2Name->canBeSet;
        info.fixedWing =    pModes2Name->fixedWing;
        info.multiRotor =   pModes2Name->multiRotor;

        _flightModeInfoList.append(info);
    }

}

void UAS::run()
{
	//qDebug("Starting UAS");
    if (connectState) {
        return;
    }

    socket = new QUdpSocket(this);
    socket->moveToThread(this);
	socket->setSocketOption(QAbstractSocket::ReceiveBufferSizeSocketOption, 65536);
    connectState = socket->bind(QHostAddress::LocalHost, 19999, QAbstractSocket::ReuseAddressHint); // listen on localhost no port, no idea why
    if (!connectState) {
        socket->deleteLater();
        socket = NULL;
        return;
    }

    QObject::connect(socket, &QUdpSocket::readyRead, this, &UAS::readData,Qt::QueuedConnection);
//s	QObject::connect(this, SIGNAL(writeExternal(uint8_t*, qint64)), this, SLOT(writeBytes(uint8_t* data, qint64 size));

    // _should_exit = false;
	bool armed = true;
    while(!_should_exit) {
        QCoreApplication::processEvents();
		//armed = !armed;
		//setArmed(armed);
        msleep(2);
    }
	//qDebug("Exiting");
    connectState = false;

    disconnect(socket, &QUdpSocket::readyRead, this, &UAS::readData);

    socket->close();
    socket->deleteLater();
    socket = NULL;
}


UAS::~UAS()
{    
    setHilMode(false);
    connectState = false;
	//qDebug("Closing if needed");
    if (socket) {
        disconnect(socket, &QUdpSocket::readyRead, this, &UAS::readData);
        socket->close();
        socket->deleteLater();
        socket = NULL;
    }
}

void UAS::sendHilGps(quint64 time_us, double lat, double lon, double alt, int fix_type, float eph, float epv, float vel, float vn, float ve, float vd, float cog, int satellites)
{

	//qDebug("MAV-HIL Send GPS");
    if (UAS::groundTimeMilliseconds() - lastSendTimeGPS < 100)
        return;

    float course = cog;
    // map to 0..2pi
    if (course < 0)
        course += 2.0f * static_cast<float>(M_PI);
    // scale from radians to degrees
    course = (course / M_PI) * 180.0f;

    mavlink_message_t msg;
    mavlink_msg_hil_gps_pack_chan(255, 0, 2,&msg,
                                  time_us, fix_type, lat*1e7, lon*1e7, alt*1e3, eph*1e2, epv*1e2, vel*1e2, vn*1e2, ve*1e2, vd*1e2, course*1e2, satellites, 0, 0);

    lastSendTimeGPS = UAS::groundTimeMilliseconds();
    writeMessage(msg);
}



void UAS::sendHilState(quint64 time_us, float roll, float pitch, float yaw, float rollspeed,
                       float pitchspeed, float yawspeed, double lat, double lon, double alt,
                       float vx, float vy, float vz, float ind_airspeed, float true_airspeed, float xacc, float yacc, float zacc)
{
    float q[4];

    double cosPhi_2 = cos(double(roll) / 2.0);
    double sinPhi_2 = sin(double(roll) / 2.0);
    double cosTheta_2 = cos(double(pitch) / 2.0);
    double sinTheta_2 = sin(double(pitch) / 2.0);
    double cosPsi_2 = cos(double(yaw) / 2.0);
    double sinPsi_2 = sin(double(yaw) / 2.0);
    q[0] = (cosPhi_2 * cosTheta_2 * cosPsi_2 +
            sinPhi_2 * sinTheta_2 * sinPsi_2);
    q[1] = (sinPhi_2 * cosTheta_2 * cosPsi_2 -
            cosPhi_2 * sinTheta_2 * sinPsi_2);
    q[2] = (cosPhi_2 * sinTheta_2 * cosPsi_2 +
            sinPhi_2 * cosTheta_2 * sinPsi_2);
    q[3] = (cosPhi_2 * cosTheta_2 * sinPsi_2 -
            sinPhi_2 * sinTheta_2 * cosPsi_2);

    mavlink_message_t msg;
    mavlink_msg_hil_state_quaternion_pack_chan(255, 0, 2,&msg,
                                               time_us, q, rollspeed, pitchspeed, yawspeed,
                                               lat*1e7f, lon*1e7f, alt*1000, vx*100, vy*100, vz*100, ind_airspeed*100, true_airspeed*100, xacc*1000/9.81, yacc*1000/9.81, zacc*1000/9.81);

    writeMessage(msg);
}

float UAS::addZeroMeanNoise(float truth_meas, float noise_var)
{
    /* Calculate normally distributed variable noise with mean = 0 and variance = noise_var.  Calculated according to
    Box-Muller transform */
    static const float epsilon = std::numeric_limits<float>::min(); //used to ensure non-zero uniform numbers
    static float z0; //calculated normal distribution random variables with mu = 0, var = 1;
    float u1, u2;        //random variables generated from c++ rand();

    /*Generate random variables in range (0 1] */
    do
    {
        //TODO seed rand() with srand(time) but srand(time should be called once on startup)
        //currently this will generate repeatable random noise
        u1 = rand() * (1.0 / RAND_MAX);
        u2 = rand() * (1.0 / RAND_MAX);
    }
    while ( u1 <= epsilon );  //Have a catch to ensure non-zero for log()

    z0 = sqrt(-2.0 * log(u1)) * cos(2.0f * M_PI * u2); //calculate normally distributed variable with mu = 0, var = 1

    //TODO add bias term that changes randomly to simulate accelerometer and gyro bias the exf should handle these
    //as well
    float noise = z0 * sqrt(noise_var); //calculate normally distributed variable with mu = 0, std = var^2

    //Finally gaurd against any case where the noise is not real
    if(std::isfinite(noise)) {
        return truth_meas + noise;
    } else {
        return truth_meas;
    }
}

/*
* @param abs_pressure Absolute Pressure (hPa)
* @param diff_pressure Differential Pressure  (hPa)
*/

void UAS::sendHilSensors(quint64 time_us, float xacc, float yacc, float zacc, float rollspeed, float pitchspeed, float yawspeed,
                         float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, quint32 fields_changed)
{
	//qDebug("MAV Sending HIL sensors");
    float noise_scaler = 0.0001f;
    float xacc_var = noise_scaler * 0.2914f;
    float yacc_var = noise_scaler * 0.2914f;
    float zacc_var = noise_scaler * 0.9577f;
    float rollspeed_var = noise_scaler * 0.8126f;
    float pitchspeed_var = noise_scaler * 0.6145f;
    float yawspeed_var = noise_scaler * 0.5852f;
    float xmag_var = noise_scaler * 0.0786f;
    float ymag_var = noise_scaler * 0.0566f;
    float zmag_var = noise_scaler * 0.0333f;
    float abs_pressure_var = noise_scaler * 0.5604f;
    float diff_pressure_var = noise_scaler * 0.2604f;
    float pressure_alt_var = noise_scaler * 0.5604f;
    float temperature_var = noise_scaler * 0.7290f;


    float xacc_corrupt = addZeroMeanNoise(xacc, xacc_var);
    float yacc_corrupt = addZeroMeanNoise(yacc, yacc_var);
    float zacc_corrupt = addZeroMeanNoise(zacc, zacc_var);
    float rollspeed_corrupt = addZeroMeanNoise(rollspeed,rollspeed_var);
    float pitchspeed_corrupt = addZeroMeanNoise(pitchspeed,pitchspeed_var);
    float yawspeed_corrupt = addZeroMeanNoise(yawspeed,yawspeed_var);
    float xmag_corrupt = addZeroMeanNoise(xmag, xmag_var);
    float ymag_corrupt = addZeroMeanNoise(ymag, ymag_var);
    float zmag_corrupt = addZeroMeanNoise(zmag, zmag_var);
    float abs_pressure_corrupt = addZeroMeanNoise(abs_pressure,abs_pressure_var);
    float diff_pressure_corrupt = addZeroMeanNoise(diff_pressure, diff_pressure_var);
    float pressure_alt_corrupt = addZeroMeanNoise(pressure_alt, pressure_alt_var);
    float temperature_corrupt = addZeroMeanNoise(temperature,temperature_var);



    mavlink_message_t msg;
    mavlink_msg_hil_sensor_pack_chan(255, 0, 2,&msg,
                                     time_us, xacc_corrupt, yacc_corrupt, zacc_corrupt, rollspeed_corrupt, pitchspeed_corrupt,
                                     yawspeed_corrupt, xmag_corrupt, ymag_corrupt, zmag_corrupt, abs_pressure_corrupt,
                                     diff_pressure_corrupt, pressure_alt_corrupt, temperature_corrupt, fields_changed, 0);


    writeMessage(msg);
}
std::string hexStr(const char *data, int len)
{
     std::stringstream ss;
     ss << std::hex;

     for( int i(0) ; i < len; ++i )
         ss << std::setw(2) << std::setfill('0') << (int)data[i];

     return ss.str();
}

void UAS::readData()
{
    const qint64 maxLength = 65536;
    char tempdata[maxLength];
    QHostAddress sender;
    quint16 senderPort;

    int s = socket->pendingDatagramSize(); // this is supposed to be a signed int
	//qDebug() << s;
    if (s > maxLength) std::cerr << __FILE__ << __LINE__ << " UDP datagram overflow, allowed to read less bytes than datagram size: " << s << std::endl;
    socket->readDatagram(tempdata, maxLength, &sender, &senderPort);
    if (s > maxLength) {
        std::string headStr = std::string(tempdata, tempdata+5);
        std::cerr << __FILE__ << __LINE__ << " UDP datagram header: " << headStr << std::endl;
    }
	//qDebug() << "FROM: " << sender.toString();	
	emit hostConnected(sender.toString());
	std::string output = hexStr(tempdata, s);
    mavlink_message_t msg;
    int chan = 0;
    mavlink_status_t status;
	//qDebug() << "Reading a mavlink message" << s;
    for(int iter=0; iter< s; ++iter)	{
        uint8_t msgReceived = mavlink_parse_char(chan, tempdata[iter], &msg, &status);
        if (msgReceived > 0)
        {
            switch(msg.msgid)
            {
            case MAVLINK_MSG_ID_HEARTBEAT : {
				//qDebug("Got heartbeat");
                mavlink_heartbeat_t heartbeat;
                mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                _base_mode =  heartbeat.base_mode;
                _custom_mode = heartbeat.custom_mode;
                emit armStayChanged(_base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY);
                emit flightModeChanged(flightMode(_base_mode,_custom_mode));
                break;
            }

            case MAVLINK_MSG_ID_HIL_CONTROLS:
            {
				qDebug("Got hil controls");
                mavlink_hil_controls_t hil;
                mavlink_msg_hil_controls_decode(&msg, &hil);
                emit hilControlsChanged(hil.time_usec, hil.roll_ailerons, hil.pitch_elevator, hil.yaw_rudder, hil.throttle, hil.mode, hil.nav_mode);

                break;
            }
			case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
            {
				//qDebug("Got hil actuator controls");
                mavlink_hil_actuator_controls_t hil;
                mavlink_msg_hil_actuator_controls_decode(&msg, &hil);
				// less expensive than 16 compares but I'll take better ideas if you got one
				float controls[17];
				std::copy(std::begin(hil.controls), std::end(hil.controls), std::begin(controls));
				controls[16] = 0;
				emit hilActuatorControlsChanged(hil.time_usec, hil.flags,
                                                controls, hil.mode);
                break;
            }
            case MAVLINK_MSG_ID_COMMAND_ACK:
            {
                mavlink_command_ack_t ack;
                mavlink_msg_command_ack_decode(&msg,&ack);
                //qDebug()<<"command: " <<ack.command << "; result: "<<ack.result;
                break;
            }
            case MAVLINK_MSG_ID_STATUSTEXT:
            {
                mavlink_statustext_t status;
                mavlink_msg_statustext_decode(&msg, &status);
                //qDebug() << status.severity << status.text;
                break;
            }
            case MAVLINK_MSG_ID_MISSION_CURRENT:
            {
                mavlink_mission_current_t mission;
                mavlink_msg_mission_current_decode(&msg,&mission);
                //qDebug() << "current mission: " << mission.seq;
                break;
            }
            case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
            {
				//qDebug("Got version");
                _handleAutopilotVersion(&msg);
            }

            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            {
				//qDebug("Got GPS");
                mavlink_global_position_int_t pos;
                mavlink_msg_global_position_int_decode(&msg,&pos);

                //qDebug() << "alt:" << pos.alt/1000.0 << "alt rel:" << pos.relative_alt/1000.0;
                break;
            }
            default:{
				//qDebug() << "Got unknown" << msg.msgid;
            }
            }
        }
		
    }
	//qDebug("Bytehonk %s", output.c_str());
}

void UAS::writeMessage(mavlink_message_t msg)
{
	//qDebug("Writing a mavlink message");
    uint8_t data[4096];
    uint16_t bytesToWrite = mavlink_msg_to_send_buffer(data,&msg);

    writeBytes(data, bytesToWrite);
}

void UAS::setHilMode(bool hilMode)
{

    //qDebug() << "setting hil" << ((hilMode)?"on":"off");
    mavlink_message_t msg;


    uint8_t newBaseMode = _base_mode & ~MAV_MODE_FLAG_DECODE_POSITION_HIL;
    if (hilMode) {
        newBaseMode |= MAV_MODE_FLAG_HIL_ENABLED;
    }

    mavlink_msg_set_mode_pack(255,0,&msg,1,newBaseMode,_custom_mode);

    writeMessage(msg);
}


void UAS::setArmed(bool armed)
{
	//qDebug("Arm called");
    mavlink_message_t msg;
    mavlink_command_long_t cmd;

    cmd.command = (uint16_t)MAV_CMD_COMPONENT_ARM_DISARM;
    cmd.confirmation = 1;
    cmd.param1 = armed ? 1.0f : 0.0f;
    cmd.param2 = 0.0f;
    cmd.param3 = 0.0f;
    cmd.param4 = 0.0f;
    cmd.param5 = 0.0f;
    cmd.param6 = 0.0f;
    cmd.param7 = 0.0f;
    cmd.target_system = 1;
    cmd.target_component = 1;//defaultComponentId();

    mavlink_msg_command_long_encode(255,0, &msg, &cmd);

    writeMessage(msg);
}

void UAS::takeoff()
{
	//qDebug("Takeoff called");
    mavlink_message_t msg;

    mavlink_command_long_t cmd;

    cmd.command = (uint16_t)MAV_CMD_NAV_TAKEOFF;
    cmd.confirmation = 1;
    cmd.param1 = -1.0f;
    cmd.param2 = 0.0f;
    cmd.param3 = 0.0f;
    cmd.param4 = NAN;
    cmd.param5 = NAN;
    cmd.param6 = NAN;
    cmd.param7 = 167.0;
    cmd.target_system = 1;
    cmd.target_component = 1;//vehicle->defaultComponentId();

    mavlink_msg_command_long_encode(255, 0, &msg, &cmd);
    writeMessage(msg);
}

void UAS::requestAutopilotCapabilites()
{
	//qDebug("Requesting autopilot capabilities");
    mavlink_message_t msg;
    mavlink_command_long_t cmd;
    cmd.command = MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
    cmd.param1 = 1;
    mavlink_msg_command_long_encode(255,0,&msg,&cmd);
    writeMessage(msg);
}

void UAS::setFlightMode(QString modeName)
{
	//qDebug("Set Flight Mode %s", modeName);
    uint32_t    custom_mode;
    uint8_t    base_mode;

    if (_setFlightMode(modeName, &base_mode, &custom_mode)) {
        // setFlightMode will only set MAV_MODE_FLAG_CUSTOM_MODE_ENABLED in base_mode, we need to move back in the existing
        // states.
        uint8_t newBaseMode = _base_mode & ~MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE;
        newBaseMode |= base_mode;

        mavlink_message_t msg;
        //qDebug() << newBaseMode << custom_mode;
        mavlink_msg_set_mode_pack(255,
                                  0,
                                  &msg,
                                  1,
                                  newBaseMode,
                                  custom_mode);
        writeMessage(msg);
    }
}

bool UAS::_setFlightMode(const QString flightMode, uint8_t* base_mode, uint32_t* custom_mode) {

    *base_mode = 0;
    *custom_mode = 0;

    for (const FlightModeInfo_t& info : _flightModeInfoList) {
        if (flightMode.compare(info.name) == 0) {
            union px4_custom_mode px4_mode;

            px4_mode.data = 0;
            px4_mode.main_mode = info.main_mode;
            px4_mode.sub_mode = info.sub_mode;

            *base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            *custom_mode = px4_mode.data;

            return true;
        }
    }

    return false;
}

QString UAS::flightMode(uint8_t base_mode, uint32_t custom_mode) const
{
    QString flightMode = "Unknown";

    if (base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
        union px4_custom_mode px4_mode;
        px4_mode.data = custom_mode;

        bool found = false;
        foreach (const FlightModeInfo_t& info, _flightModeInfoList) {
            if (info.main_mode == px4_mode.main_mode && info.sub_mode == px4_mode.sub_mode) {
                flightMode = info.name;
                found = true;
                break;
            }
        }

        if (!found) {
            qWarning() << "Unknown flight mode" << custom_mode;
        }
    } else {
        qWarning() << "PX4 Flight Stack flight mode without custom mode enabled?";
    }
    //qDebug() << "Setting flight mode: " << flightMode;
    return flightMode;
}

void UAS::_handleAutopilotVersion(mavlink_message_t* message)
{

    if (!_versionNotified) {
        bool notifyUser = false;
        int supportedMajorVersion = 1;
        int supportedMinorVersion = 4;
        int supportedPatchVersion = 1;

        mavlink_autopilot_version_t version;
        mavlink_msg_autopilot_version_decode(message, &version);

        if (version.flight_sw_version != 0) {
            int majorVersion, minorVersion, patchVersion;

            majorVersion = (version.flight_sw_version >> (8*3)) & 0xFF;
            minorVersion = (version.flight_sw_version >> (8*2)) & 0xFF;
            patchVersion = (version.flight_sw_version >> (8*1)) & 0xFF;

            if (majorVersion < supportedMajorVersion) {
                notifyUser = true;
            } else if (majorVersion == supportedMajorVersion) {
                if (minorVersion < supportedMinorVersion) {
                    notifyUser = true;
                } else if (minorVersion == supportedMinorVersion) {
                    notifyUser = patchVersion < supportedPatchVersion;
                }
            }
        } else {
            notifyUser = true;
        }

        if (notifyUser)
        {
            _versionNotified = true;
            //qDebug() << QString("QGroundControl supports PX4 Pro firmware Version %1.%2.%3 and above. You are using a version prior to that which will lead to unpredictable results. Please upgrade your firmware.").arg(supportedMajorVersion).arg(supportedMinorVersion).arg(supportedPatchVersion);
        }
    }
}

void UAS::writeBytes(const uint8_t* data, qint64 size)
{
    if (!data) return;
	//qDebug() << "Sending to" << remoteHost.toString() << remotePort;
    // If socket exists and is connected, transmit the data
    if (socket && connectState)
    {
        socket->writeDatagram((char*)data, size, remoteHost, remotePort);
		//qDebug() << "Writing datagram though";
    }
	else
	{
		//qDebug() << "Not connected" << connectState;
	}
	
}

void UAS::stop()
{
	_should_exit = true;
}
