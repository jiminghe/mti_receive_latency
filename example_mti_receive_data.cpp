#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>

#include <iostream>
#include <iomanip>
#include <list>
#include <string>

#include <fstream>
#include <vector>
#include <sstream>
#include <time.h>   // For clock_gettime
#include <stdint.h> // For int64_t

Journaller* gJournal = 0;

using namespace std;




double getCurrentUTCTime()
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts); // Get the current time

    // Convert seconds and nanoseconds to a double representing seconds
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) / 1e9;
}



// Structure to hold logged data
struct LogEntry {
    double utcTimestamp;
	uint32_t sampleTimeFine;
    double accX, accY, accZ;
    double gyroX, gyroY, gyroZ;
    double q0, q1, q2, q3;
};

// Store log entries in a vector
std::vector<LogEntry> logEntries;

void writeLogToCSV(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) return;

    // Write the header
    file << "UTC_Timestamp,SampleTimeFine,Acc_X,Acc_Y,Acc_Z,Gyro_X,Gyro_Y,Gyro_Z,Q0,Q1,Q2,Q3\n";

    // Write each log entry
    for (const auto& entry : logEntries) {
        file << std::fixed << std::setprecision(6)
             << entry.utcTimestamp << ","
			 << entry.sampleTimeFine << ","
             << entry.accX << ","
             << entry.accY << ","
             << entry.accZ << ","
             << entry.gyroX << ","
             << entry.gyroY << ","
             << entry.gyroZ << ","
             << entry.q0 << ","
             << entry.q1 << ","
             << entry.q2 << ","
             << entry.q3 << "\n";
    }

    file.close();
}

class CallbackHandler : public XsCallback
{
public:
	CallbackHandler(size_t maxBufferSize = 5)
		: m_maxNumberOfPacketsInBuffer(maxBufferSize)
		, m_numberOfPacketsInBuffer(0)
	{
	}

	virtual ~CallbackHandler() throw()
	{
	}

	bool packetAvailable() const
	{
		xsens::Lock locky(&m_mutex);
		return m_numberOfPacketsInBuffer > 0;
	}

	XsDataPacket getNextPacket()
	{
		assert(packetAvailable());
		xsens::Lock locky(&m_mutex);
		XsDataPacket oldestPacket(m_packetBuffer.front());
		m_packetBuffer.pop_front();
		--m_numberOfPacketsInBuffer;
		return oldestPacket;
	}

protected:
	void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override
	{
		xsens::Lock locky(&m_mutex);
		assert(packet != 0);

		// Collect data
		if (packet->containsSampleTimeFine() && packet->containsCalibratedData() && packet->containsOrientation()) {
			LogEntry entry;
			entry.utcTimestamp = getCurrentUTCTime(); // using Ubuntu native method to get time.
			entry.sampleTimeFine = packet->sampleTimeFine();
			entry.accX = packet->calibratedAcceleration()[0];
			entry.accY = packet->calibratedAcceleration()[1];
			entry.accZ = packet->calibratedAcceleration()[2];
			entry.gyroX = packet->calibratedGyroscopeData()[0];
			entry.gyroY = packet->calibratedGyroscopeData()[1];
			entry.gyroZ = packet->calibratedGyroscopeData()[2];
			entry.q0 = packet->orientationQuaternion().w();
			entry.q1 = packet->orientationQuaternion().x();
			entry.q2 = packet->orientationQuaternion().y();
			entry.q3 = packet->orientationQuaternion().z();

			logEntries.push_back(entry);
		}

		while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
			(void)getNextPacket();

		m_packetBuffer.push_back(*packet);
		++m_numberOfPacketsInBuffer;
		assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
	}
private:
	mutable xsens::Mutex m_mutex;

	size_t m_maxNumberOfPacketsInBuffer;
	size_t m_numberOfPacketsInBuffer;
	list<XsDataPacket> m_packetBuffer;
};

//--------------------------------------------------------------------------------
int main(void)
{
	cout << "Creating XsControl object..." << endl;
	XsControl* control = XsControl::construct();
	assert(control != 0);

	// Lambda function for error handling
	auto handleError = [=](string errorString)
	{
		control->destruct();
		cout << errorString << endl;
		cout << "Press [ENTER] to continue." << endl;
		cin.get();
		return -1;
	};

	cout << "Scanning for devices..." << endl;
	XsPortInfoArray portInfoArray = XsScanner::scanPorts();

	// Find an MTi device
	XsPortInfo mtPort;
	for (auto const &portInfo : portInfoArray)
	{
		if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
		{
			mtPort = portInfo;
			break;
		}
	}

	if (mtPort.empty())
		return handleError("No MTi device found. Aborting.");

	cout << "Found a device with ID: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << endl;

	cout << "Opening port..." << endl;
	if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
		return handleError("Could not open port. Aborting.");

	// Get the device object
	XsDevice* device = control->device(mtPort.deviceId());
	assert(device != 0);

	cout << "Device: " << device->productCode().toStdString() << ", with ID: " << device->deviceId().toString() << " opened." << endl;

	// Create and attach callback handler to device
	CallbackHandler callback;
	device->addCallbackHandler(&callback);

	// Put the device into configuration mode before configuring the device
	cout << "Putting device into configuration mode..." << endl;
	if (!device->gotoConfig())
		return handleError("Could not put device into configuration mode. Aborting.");

	cout << "Configuring the device..." << endl;

	// Important for Public XDA!
	// Call this function if you want to record a mtb file:
	device->readEmtsAndDeviceConfiguration();

	XsOutputConfigurationArray configArray;
	configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
	configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));
	configArray.push_back(XsOutputConfiguration(XDI_StatusWord, 0));

	if (device->deviceId().isImu())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 100));
		configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 100));
		configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 100));
	}
	else if (device->deviceId().isVru() || device->deviceId().isAhrs())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 400));
		configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 400));
		configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 100));
		configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 400));
	}
	else if (device->deviceId().isGnss())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 100));
		configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 100));
		configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 100));
		configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 100));
		configArray.push_back(XsOutputConfiguration(XDI_LatLon, 100));
		configArray.push_back(XsOutputConfiguration(XDI_AltitudeEllipsoid, 100));
		configArray.push_back(XsOutputConfiguration(XDI_VelocityXYZ, 100));
	}
	else
	{
		return handleError("Unknown device while configuring. Aborting.");
	}

	if (!device->setOutputConfiguration(configArray))
		return handleError("Could not configure MTi device. Aborting.");

	cout << "Creating a log file..." << endl;
	string logFileName = "logfile.mtb";
	if (device->createLogFile(logFileName) != XRV_OK)
		return handleError("Failed to create a log file. Aborting.");
	else
		cout << "Created a log file: " << logFileName.c_str() << endl;

	cout << "Putting device into measurement mode..." << endl;
	if (!device->gotoMeasurement())
		return handleError("Could not put device into measurement mode. Aborting.");

	cout << "Starting recording..." << endl;
	if (!device->startRecording())
		return handleError("Failed to start recording. Aborting.");

	cout << "\nMain loop. Recording data for 10 seconds." << endl;
	cout << string(79, '-') << endl;

	int64_t startTime = XsTime::timeStampNow();
	while (XsTime::timeStampNow() - startTime <= 10000)
	{
		if (callback.packetAvailable())
		{
			cout << setw(5) << fixed << setprecision(2);

			// Retrieve a packet
			XsDataPacket packet = callback.getNextPacket();

			if(packet.containsSampleTimeFine())
			{
				uint32_t sampleTimeFine = packet.sampleTimeFine();

			}


			if (packet.containsCalibratedData())
			{
				XsVector acc = packet.calibratedAcceleration();
				cout << "\r"
					<< "Acc X:" << acc[0]
					<< ", Acc Y:" << acc[1]
					<< ", Acc Z:" << acc[2];

				XsVector gyr = packet.calibratedGyroscopeData();
				cout << " |Gyr X:" << gyr[0]
					<< ", Gyr Y:" << gyr[1]
					<< ", Gyr Z:" << gyr[2];

				XsVector mag = packet.calibratedMagneticField();
				cout << " |Mag X:" << mag[0]
					<< ", Mag Y:" << mag[1]
					<< ", Mag Z:" << mag[2];
			}

			if (packet.containsOrientation())
			{
				XsQuaternion quaternion = packet.orientationQuaternion();
				cout << "\r"
					<< "q0:" << quaternion.w()
					<< ", q1:" << quaternion.x()
					<< ", q2:" << quaternion.y()
					<< ", q3:" << quaternion.z();

				XsEuler euler = packet.orientationEuler();
				cout << " |Roll:" << euler.roll()
					<< ", Pitch:" << euler.pitch()
					<< ", Yaw:" << euler.yaw();
			}

			if (packet.containsLatitudeLongitude())
			{
				XsVector latLon = packet.latitudeLongitude();
				cout << " |Lat:" << latLon[0]
					<< ", Lon:" << latLon[1];
			}

			if (packet.containsAltitude())
				cout << " |Alt:" << packet.altitude();

			if (packet.containsVelocity())
			{
				XsVector vel = packet.velocity(XDI_CoordSysEnu);
				cout << " |E:" << vel[0]
					<< ", N:" << vel[1]
					<< ", U:" << vel[2];
			}
			
			cout << flush;
		}
		XsTime::msleep(0);
	}
	cout << "\n" << string(79, '-') << "\n";
	cout << endl;

	cout << "Stopping recording..." << endl;
	if (!device->stopRecording())
		return handleError("Failed to stop recording. Aborting.");

	cout << "Closing log file..." << endl;
	if (!device->closeLogFile())
		return handleError("Failed to close log file. Aborting.");

	writeLogToCSV("data_log.csv");

	cout << "Closing port..." << endl;
	control->closePort(mtPort.portName().toStdString());

	cout << "Freeing XsControl object..." << endl;
	control->destruct();

	cout << "Successful exit." << endl;

	cout << "Press [ENTER] to continue." << endl;
	cin.get();

	return 0;
}
