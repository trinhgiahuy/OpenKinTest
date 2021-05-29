/*	Copyright (c) 2003-2016 Xsens Technologies B.V. or subsidiaries worldwide.
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1.	Redistributions of source code must retain the above copyright notice,
		this list of conditions and the following disclaimer.

	2.	Redistributions in binary form must reproduce the above copyright notice,
		this list of conditions and the following disclaimer in the documentation
		and/or other materials provided with the distribution.

	3.	Neither the names of the copyright holders nor the names of their contributors
		may be used to endorse or promote products derived from this software without
		specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
	THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
	SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
	OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
	TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <xsensdeviceapi.h> // The Xsens device API header
//#include "conio.h"			// For non ANSI _kbhit() and _getch()

#include <string>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <set>
#include <list>
#include <utility>

#include <xsens/xsmutex.h>

#include <ros/ros.h>
//#include <sensor_msgs/Imu.h>
#include <imu_sequenced/ImuSequenced.h>
#include <ctime>

/*! \brief Stream insertion operator overload for XsPortInfo */
std::ostream& operator << (std::ostream& out, XsPortInfo const & p)
{
	out << "Port: " << std::setw(2) << std::right << p.portNumber() << " (" << p.portName().toStdString() << ") @ "
		<< std::setw(7) << p.baudrate() << " Bd"
		<< ", " << "ID: " << p.deviceId().toString().toStdString()
	;
	return out;
}

/*! \brief Stream insertion operator overload for XsDevice */
std::ostream& operator << (std::ostream& out, XsDevice const & d)
{
	out << "ID: " << d.deviceId().toString().toStdString() << " (" << d.productCode().toStdString() << ")";
	return out;
}

/*! \brief Given a list of update rates and a desired update rate, returns the closest update rate to the desired one */
int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate)
{
	if (supportedUpdateRates.empty())
	{
		return 0;
	}

	if (supportedUpdateRates.size() == 1)
	{
		return supportedUpdateRates[0];
	}

	int uRateDist = -1;
	int closestUpdateRate = -1;
	for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
	{
		const int currDist = std::abs(*itUpRate - desiredUpdateRate);

		if ((uRateDist == -1) || (currDist < uRateDist))
		{
			uRateDist = currDist;
			closestUpdateRate = *itUpRate;
		}
	}
	return closestUpdateRate;
}

//----------------------------------------------------------------------
// Callback handler for wireless master
//----------------------------------------------------------------------
class WirelessMasterCallback : public XsCallback
{
public:
	typedef std::set<XsDevice*> XsDeviceSet;

	XsDeviceSet getWirelessMTWs() const
	{
		XsMutexLocker lock(m_mutex);
		return m_connectedMTWs;
	}

protected:
	virtual void onConnectivityChanged(XsDevice* dev, XsConnectivityState newState)
	{
		XsMutexLocker lock(m_mutex);
		switch (newState)
		{
		case XCS_Disconnected:		/*!< Device has disconnected, only limited informational functionality is available. */
			std::cout << "\nEVENT: MTW Disconnected -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_Rejected:			/*!< Device has been rejected and is disconnected, only limited informational functionality is available. */
			std::cout << "\nEVENT: MTW Rejected -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_PluggedIn:			/*!< Device is connected through a cable. */
			std::cout << "\nEVENT: MTW PluggedIn -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_Wireless:			/*!< Device is connected wirelessly. */
			std::cout << "\nEVENT: MTW Connected -> " << *dev << std::endl;
			m_connectedMTWs.insert(dev);
			break;
		case XCS_File:				/*!< Device is reading from a file. */
			std::cout << "\nEVENT: MTW File -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_Unknown:			/*!< Device is in an unknown state. */
			std::cout << "\nEVENT: MTW Unknown -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		default:
			std::cout << "\nEVENT: MTW Error -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		}
	}
private:
	mutable XsMutex m_mutex;
	XsDeviceSet m_connectedMTWs;
};

//----------------------------------------------------------------------
// Callback handler for MTw
// Handles onDataAvailable callbacks for MTW devices
//----------------------------------------------------------------------
class MtwCallback : public XsCallback
{
public:
	MtwCallback(int mtwIndex, XsDevice* device)
		:m_mtwIndex(mtwIndex)
		,m_device(device)
	{}

	bool dataAvailable() const
	{
		XsMutexLocker lock(m_mutex);
		return !m_packetBuffer.empty();
	}

	XsDataPacket const * getOldestPacket() const
	{
		XsMutexLocker lock(m_mutex);
		XsDataPacket const * packet = &m_packetBuffer.front();
		return packet;
	}

	void deleteOldestPacket()
	{
		XsMutexLocker lock(m_mutex);
		m_packetBuffer.pop_front();
	}

	int getMtwIndex() const
	{
		return m_mtwIndex;
	}

	XsDevice const & device() const
	{
		assert(m_device != 0);
		return *m_device;
	}

protected:
	virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
	{
		XsMutexLocker lock(m_mutex);
		// NOTE: Processing of packets should not be done in this thread.

		m_packetBuffer.push_back(*packet);
		if (m_packetBuffer.size() > 300)
		{
			std::cout << std::endl;
			deleteOldestPacket();
		}
	}

private:
	mutable XsMutex m_mutex;
	std::list<XsDataPacket> m_packetBuffer;
	int m_mtwIndex;
	XsDevice* m_device;
};

void connected(const ros::SingleSubscriberPublisher&) {}
void disconnected(const ros::SingleSubscriberPublisher&) {}

//----------------------------------------------------------------------
// Main
//----------------------------------------------------------------------
int main(int argc, char* argv[])
{
	//(void)argc;
	//(void)argv;
	const int desiredUpdateRate = 100;	// Use 75 Hz update rate for MTWs
	const int desiredRadioChannel = 25;	// Use radio channel 19 for wireless master.

	ros::init(argc, argv, "MTw_node");
	ROS_INFO("XSens MTw node for ROS");

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_("~");

	ros::AdvertiseOptions op = ros::AdvertiseOptions::create<imu_sequenced::ImuSequenced>("/imu/data", 200, &connected, &disconnected, ros::VoidPtr(), NULL);
	//op.has_header = false;

	ros::AdvertiseOptions op2 = ros::AdvertiseOptions::create<imu_sequenced::ImuSequenced>("/imu2/data", 1, &connected, &disconnected, ros::VoidPtr(), NULL);
	//op2.has_header = false;

imu_sequenced::ImuSequenced imu_msg;

	ros::Publisher imu_pub_;
	ros::Publisher imu_pub2_;

	std::string imu_frame_id_ = "mtw_node/";

	ros::Time last_update_;

	int num;
	private_nh_.param("num", num, int(1));

	std::string device;
	private_nh_.param<std::string>("device", device, "/dev/ttyUSB0");

	WirelessMasterCallback wirelessMasterCallback; // Callback for wireless master
	std::vector<MtwCallback*> mtwCallbacks; // Callbacks for mtw devices

	std::cout << "Constructing XsControl..." << std::endl;
	XsControl* control = XsControl::construct();
	if (control == 0)
	{
		std::cout << "Failed to construct XsControl instance." << std::endl;
	}

	try
	{
		std::cout << "Scanning ports..." << std::endl;
		//XsPortInfoArray detectedDevices = XsScanner::scanPorts();

		XsString portname(device.c_str());

		//std::cout << "Finding wireless master..." << std::endl;
		//XsPortInfoArray::const_iterator wirelessMasterPort = detectedDevices.begin();
		//while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster())
		//{
		//	++wirelessMasterPort;
		//}
		//if (wirelessMasterPort == detectedDevices.end())
		//{
		//	throw std::runtime_error("No wireless masters found");
		//}

		XsPortInfo awinda(portname);
		awinda.setBaudrate(XBR_2000k);

//		if (awinda.deviceId().isWirelessMaster()) {
//			std::cout << "Wireless master found @ " << awinda << std::endl;
//		} else {
//			throw std::runtime_error("Not wireless master");
//		}

		std::cout << "Opening port... " << awinda.portName().toStdString() << ", " << awinda.baudrate() << std::endl;
		if (!control->openPort(awinda))
		{
			std::ostringstream error;
			error << "Failed to open port " << portname;
			throw std::runtime_error(error.str());
		}

		std::cout << "Getting XsDevice instance for wireless master..." << std::endl;
		XsDevicePtr wirelessMasterDevice = control->device(awinda.deviceId());
		if (wirelessMasterDevice == 0)
		{
			std::ostringstream error;
			error << "Failed to construct XsDevice instance: " << portname;
			throw std::runtime_error(error.str());
		}

		std::cout << "XsDevice instance created @ " << *wirelessMasterDevice << std::endl;

		std::cout << "Setting config mode..." << std::endl;
		if (!wirelessMasterDevice->gotoConfig())
		{
			std::ostringstream error;
			error << "Failed to goto config mode: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Attaching callback handler..." << std::endl;
		wirelessMasterDevice->addCallbackHandler(&wirelessMasterCallback);

		std::cout << "Getting the list of the supported update rates..." << std::endl;
		const XsIntArray supportedUpdateRates = wirelessMasterDevice->supportedUpdateRates();

		std::cout << "Supported update rates: ";
		for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
		{
			std::cout << *itUpRate << " ";
		}
		std::cout << std::endl;

		const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, desiredUpdateRate);

		std::cout << "Setting update rate to " << newUpdateRate << " Hz..." << std::endl;
		if (!wirelessMasterDevice->setUpdateRate(newUpdateRate))
		{
			std::ostringstream error;
			error << "Failed to set update rate: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Disabling radio channel if previously enabled..." << std::endl;
		if (wirelessMasterDevice->isRadioEnabled())
		{
			if (!wirelessMasterDevice->disableRadio())
			{
				std::ostringstream error;
				error << "Failed to disable radio channel: " << *wirelessMasterDevice;
				throw std::runtime_error(error.str());
			}
		}

		std::cout << "Setting radio channel to " << desiredRadioChannel << " and enabling radio..." << std::endl;
		if (!wirelessMasterDevice->enableRadio(desiredRadioChannel))
		{
			std::ostringstream error;
			error << "Failed to set radio channel: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Waiting for MTW to wirelessly connect...\n" << std::endl;

		bool waitForConnections = true;
		size_t connectedMTWCount = wirelessMasterCallback.getWirelessMTWs().size();
		int counter = 0;
		do
		{
			XsTime::msleep(100);

			while (true)
			{
				size_t nextCount = wirelessMasterCallback.getWirelessMTWs().size();
				if (nextCount != connectedMTWCount)
				{
					std::cout << "Number of connected MTWs: " << nextCount << std::endl;
					connectedMTWCount = nextCount;
					counter = 30;
				}
				else
				{
					break;
				}
			}
			/*if (_kbhit())
			{
				waitForConnections = (toupper((char)_getch()) != 'Y');
			}*/
			counter++;
			if (counter >= 120) {
				waitForConnections = false;
			}
		}
		while (waitForConnections);

		if (connectedMTWCount == 0) {
			throw std::runtime_error("Couldn't find any MTw's");
		}

		std::cout << "Starting measurement..." << std::endl;
		if (!wirelessMasterDevice->gotoMeasurement())
		{
			std::ostringstream error;
			error << "Failed to goto measurement mode: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Getting XsDevice instances for all MTWs..." << std::endl;
		XsDeviceIdArray allDeviceIds = control->deviceIds();
		XsDeviceIdArray mtwDeviceIds;
		for (XsDeviceIdArray::const_iterator i = allDeviceIds.begin(); i != allDeviceIds.end(); ++i)
		{
			if (i->isMtw())
			{
				mtwDeviceIds.push_back(*i);
			}
		}
		XsDevicePtrArray mtwDevices;
		for (XsDeviceIdArray::const_iterator i = mtwDeviceIds.begin(); i != mtwDeviceIds.end(); ++i)
		{
			XsDevicePtr mtwDevice = control->device(*i);
			if (mtwDevice != 0)
			{
				mtwDevices.push_back(mtwDevice);
			}
			else
			{
				throw std::runtime_error("Failed to create an MTW XsDevice instance");
			}
		}

		std::cout << "Attaching callback handlers to MTWs..." << std::endl;
		mtwCallbacks.resize(mtwDevices.size());
		for (int i = 0; i < (int)mtwDevices.size(); ++i)
		{
			mtwCallbacks[i] = new MtwCallback(i, mtwDevices[i]);
			mtwDevices[i]->addCallbackHandler(mtwCallbacks[i]);
			mtwDevices[i]->resetOrientation(XRM_Alignment);
		}

		//imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data",200,false);
		imu_pub_ = nh_.advertise(op);
		imu_pub2_ = nh_.advertise(op2);

		std::cout << "\nMain loop\n" << std::endl;
		std::cout << "Outputting data..." << std::endl;

		//std::vector<XsEuler> eulerData(mtwCallbacks.size()); // Room to store euler data for each mtw
		std::vector<XsVector> calibAcc(mtwCallbacks.size()); // Calibrated accelerations
		std::vector<XsVector> calibGyro(mtwCallbacks.size()); // Calibrated gyroscope
		std::vector<XsQuaternion> quaternions(mtwCallbacks.size()); // Orientation
		std::vector<XsTimeStamp> timesOA(mtwCallbacks.size()); // Times of arrival
		std::vector<XsUtcTime> utcTimes(mtwCallbacks.size()); // UTC times of packets
		std::vector<uint16_t> packetCounters(mtwCallbacks.size()); // Packet counters
		std::vector<XsDeviceId> deviceIDs(mtwCallbacks.size()); // device ids
		std::vector<ros::Time> times(mtwCallbacks.size());
		//unsigned int printCounter = 0;

		/*time_t zero = 24*60*60L;
		double time_diff = difftime(mktime(gmtime(&zero)), mktime(localtime(&zero)));

		std::cout << "Time diff: " << time_diff << std::endl;*/

		while (ros::ok()) {
			XsTime::msleep(0);

			bool newDataAvailable = false;
			for (size_t i = 0; i < mtwCallbacks.size(); ++i)
			{
				if (mtwCallbacks[i]->dataAvailable())
				{
					//times[i] = ros::Time::now();
					newDataAvailable = true;
					XsDataPacket const * packet = mtwCallbacks[i]->getOldestPacket();
					//eulerData[i] = packet->orientationEuler();
					calibAcc[i] = packet->calibratedAcceleration();
					calibGyro[i] = packet->calibratedGyroscopeData();
					quaternions[i] = packet->orientationQuaternion();
					timesOA[i] = packet->timeOfArrival();
					//utcTimes[i] = packet->utcTime();
					packetCounters[i] = packet->packetCounter();
					deviceIDs[i] = packet->deviceId();
					mtwCallbacks[i]->deleteOldestPacket();

				}
			}

			if (newDataAvailable)
			{
				for (size_t i = 0; i < mtwCallbacks.size(); ++i)
				{
					ros::Time toa;
					uint64_t nsecs = (uint64_t)timesOA[i].msTime();
					nsecs *= 1000000;
					toa.fromNSec(nsecs);

					/*struct tm * timeparts;
					timeparts->tm_year = utcTimes[i].m_year - 1900;
					timeparts->tm_mon = utcTimes[i].m_month - 1;
					timeparts->tm_mday = utcTimes[i].m_day;
					timeparts->tm_hour = utcTimes[i].m_hour;
					timeparts->tm_min = utcTimes[i].m_minute;
					timeparts->tm_sec = utcTimes[i].m_second;*/

					/*std::cout << utcTimes[i].m_year << ", "
					<< utcTimes[i].m_month << ", "
					<< utcTimes[i].m_day << ", "
					<< utcTimes[i].m_hour << ", "
					<< utcTimes[i].m_minute << ", "
					<< utcTimes[i].m_second << ", " << std::endl;

					uint64_t time2;
					//time2 = (uint64_t)mktime(timeparts);
					//time2 *= 1e9;
					//time2 += utcTimes[i].m_nano;

					//toa.fromNSec(time2);*/

					imu_msg.imu.header.stamp = toa;
					//imu_msg.imu.header.stamp = times[i];
					imu_msg.imu.header.frame_id = imu_frame_id_ + deviceIDs[i].toString().toStdString();
					imu_msg.seq = packetCounters[i];

					imu_msg.imu.orientation.x = quaternions[i].x();
					imu_msg.imu.orientation.y = quaternions[i].y();
					imu_msg.imu.orientation.z = quaternions[i].z();
					imu_msg.imu.orientation.w = quaternions[i].w();

					imu_msg.imu.angular_velocity.x = calibGyro[i][0];
					imu_msg.imu.angular_velocity.y = calibGyro[i][1];
					imu_msg.imu.angular_velocity.z = calibGyro[i][2];

					imu_msg.imu.linear_acceleration.x = calibAcc[i][0];
					imu_msg.imu.linear_acceleration.y = calibAcc[i][1];
					imu_msg.imu.linear_acceleration.z = calibAcc[i][2];

					imu_pub_.publish(imu_msg);
					imu_pub2_.publish(imu_msg);

					/*std::cout << "[" << i << "]: ID: " << mtwCallbacks[i]->device().deviceId().toString().toStdString()
					<< ", Roll: " << std::setw(7) << std::fixed << std::setprecision(2) << eulerData[i].roll()
					<< ", Pitch: " << std::setw(7) << std::fixed << std::setprecision(2) << eulerData[i].pitch()
					<< ", Yaw: " <<  std::setw(7) << std::fixed << std::setprecision(2) << eulerData[i].yaw()
					<< std::endl << std::setprecision(14)
					<< ", Acc: " << calibAcc[i][0] << ", " << calibAcc[i][1] << ", " << calibAcc[i][2]
					<< ", Gyro: " << calibGyro[i][0] << ", " << calibGyro[i][1] << ", " << calibGyro[i][2]
					<< ", TOA: " << timesOA[i].msTime()
					<< ", Counter: " << packetCounters[i] << std::endl
					<< ", Orientation: " << quaternions[i].x() << ", " << quaternions[i].y() << ", " << quaternions[i].z() << ", " << quaternions[i].w()
					<< "\n";*/

				}
			}

		}
		//(void)_getch();

		std::cout << "Setting config mode..." << std::endl;
		if (!wirelessMasterDevice->gotoConfig())
		{
			std::ostringstream error;
			error << "Failed to goto config mode: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Disabling radio... " << std::endl;
		if (!wirelessMasterDevice->disableRadio())
		{
			std::ostringstream error;
			error << "Failed to disable radio: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}
	}
	catch (std::exception const & ex)
	{
		std::cout << ex.what() << std::endl;
		std::cout << "****ABORT****" << std::endl;
	}
	/*catch (...)
	{
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
		std::cout << "****ABORT****" << std::endl;
	}*/

	std::cout << "Closing XsControl..." << std::endl;
	control->close();

	std::cout << "Deleting mtw callbacks..." << std::endl;
	for (std::vector<MtwCallback*>::iterator i = mtwCallbacks.begin(); i != mtwCallbacks.end(); ++i)
	{
		delete (*i);
	}

	std::cout << "Successful exit." << std::endl;
	//std::cout << "Press [ENTER] to continue." << std::endl; std::cin.get();
	return 0;
}
