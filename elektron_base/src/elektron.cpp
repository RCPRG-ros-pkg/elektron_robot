/*
 * elektronv2.cpp
 *
 *  Created on: May 22, 2012
 *      Author: mwalecki
 */

#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <cstring>
#include <iostream>

#include "elektronv2.hpp"

using namespace std;

NF_STRUCT_ComBuf	NFComBuf;
uint8_t             crcTable[256];

double ang_nor_rad(double rad) {
	static double TWO_PI = 2.0 * M_PI;
	for (;;) {
		if (rad >= M_PI)
			rad -= TWO_PI;
		else if (rad <= -M_PI)
			rad += TWO_PI;
		else
			return (rad);
	}
}

Elektron::Elektron(const std::string& port, int baud) {
	connected = false;
	
    crcInit();
    
    this->txAddr = NF_RobotAddress;
    NFComBuf.myAddress = NF_TerminalAddress;
    NFComBuf.ReadDeviceVitals.addr[0] = NF_RobotAddress;
    NFComBuf.ReadDeviceVitals.addr[1] = NF_RobotAddress;
    NFComBuf.ReadDeviceVitals.addr[2] = NF_RobotAddress;
    NFComBuf.ReadDeviceVitals.addr[3] = NF_RobotAddress;
    NFComBuf.ReadDeviceVitals.addr[4] = NF_RobotAddress;
    NFComBuf.ReadDeviceVitals.addr[5] = NF_RobotAddress;
    NFComBuf.ReadDeviceVitals.addr[6] = NF_RobotAddress;
    NFComBuf.ReadDeviceVitals.addr[7] = NF_RobotAddress;
    NFComBuf.SetDrivesMode.addr[0] = NF_RobotAddress;
    NFComBuf.SetDrivesMode.addr[1] = NF_RobotAddress;
    NFComBuf.SetDrivesSpeed.addr[0] = NF_RobotAddress;
    NFComBuf.SetDrivesSpeed.addr[1] = NF_RobotAddress;
    NFComBuf.SetDigitalOutputs.addr[0] = NF_RobotAddress;


	fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd >= 0) {
		tcgetattr(fd, &oldtio);

		// set up new settings
		struct termios newtio;
		memset(&newtio, 0, sizeof(newtio));
		newtio.c_cflag = CBAUD | CS8 | CLOCAL | CREAD | CSTOPB;
		newtio.c_iflag = INPCK; //IGNPAR;
		newtio.c_oflag = 0;
		newtio.c_lflag = 0;
		if (cfsetispeed(&newtio, baud) < 0 || cfsetospeed(&newtio, baud) < 0) {
			fprintf(stderr, "Failed to set serial baud rate: %d\n", baud);
			tcsetattr(fd, TCSANOW, &oldtio);
			close(fd);
			fd = -1;
			return;
		}
		// activate new settings
		tcflush(fd, TCIFLUSH);
		tcsetattr(fd, TCSANOW, &newtio);
		connected = true;
	}
}

Elektron::~Elektron() {
	// restore old port settings
	if (fd > 0)
		tcsetattr(fd, TCSANOW, &oldtio);
	close(fd);
}

void Elektron::update() {

	if(commandCnt == 0){
		cout << "No data to send" << endl;
		return;
	}
	
    txCnt = NF_MakeCommandFrame(txBuf, commandArray, commandCnt, txAddr);
    clearCommandArray();
    
	tcflush(fd, TCIFLUSH);
	write(fd, txBuf, txCnt);

//	while (ret < sizeof(getdata))
//		ret += read(fd, ((char*) &getdata) + ret, sizeof(getdata) - ret);

}

void Elektron::setVelocity(double lvel, double rvel) {
	lvel = (int16_t)(lvel * (1 / m_per_tick) * 0.1); // Convert SI units to internal units
	rvel = (int16_t)(rvel * (1 / m_per_tick) * 0.1);

	if (rvel > MAX_VEL)
		rvel = MAX_VEL;
	else if (rvel < -MAX_VEL)
		rvel = -MAX_VEL;

	if (lvel > MAX_VEL)
		lvel = MAX_VEL;
	else if (lvel < -MAX_VEL)
		lvel = -MAX_VEL;
		
	NFComBuf.SetDrivesSpeed.data[0] = lvel;
	NFComBuf.SetDrivesSpeed.data[1] = rvel;
	
	addToCommandArray(NF_COMMAND_SetDrivesSpeed);
	
	NFComBuf.SetDrivesMode.data[0] = NF_DrivesMode_SPEED;
	NFComBuf.SetDrivesMode.data[1] = NF_DrivesMode_SPEED;
	
	addToCommandArray(NF_COMMAND_SetDrivesMode);
	
	cout << "set Velocity" << endl;
}

void Elektron::getVelocity(double &xvel, double &thvel) {
/*	static int maxl = 0, maxr = 0;
	double lvel = (double) (getdata.lvel) * m_per_tick * 10;
	double rvel = (double) (getdata.rvel) * m_per_tick * 10;

	if (getdata.lvel > maxl) maxl = getdata.lvel;
	if (getdata.rvel > maxr) maxr = getdata.rvel;

	//std::cout << maxl << " " << maxr << "\n";
	xvel = (lvel + rvel) * 0.5;
	thvel = (lvel - rvel) / AXLE_LENGTH;*/
}

void Elektron::setScale(double ls, double rs) {
	lin_scale = ls;
	rot_scale = rs;
}

bool Elektron::isConnected() {
	return connected;
}

void Elektron::addToCommandArray(uint8_t command){
	commandArray[commandCnt++] = command;
}

void Elektron::clearCommandArray(void){
	commandCnt = 0;
}








