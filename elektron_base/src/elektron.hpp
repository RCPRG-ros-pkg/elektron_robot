/*
 * elektronv2.hpp
 *
 *  Created on: May 22, 2012
 *      Author: mwalecki
 */

#ifndef ELEKTRON_HPP_
#define ELEKTRON_HPP_

#include <stdint.h>
#include <termios.h>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <fstream>
#include <inttypes.h>

#include "nfv2.h"

// baudrate
#define BAUD B115200
// port
#define PORT = "/dev/ttyUSB0"

// wheel diameter in SI units [m]
#define WHEEL_DIAM 0.1
// axle length in SI units [m]
#define AXLE_LENGTH 0.355

// regulator rate in SI units [Hz]
#define REGULATOR_RATE 100

// maximum velocity, in internal units
#define MAX_VEL 5500
// number of encoder ticks per single wheel rotation
#define ENC_TICKS 4000




class Elektron {
public:
	Elektron(const std::string& port, int baud = BAUD);
	~Elektron();

	void update();

	void setVelocity(double lvel, double rvel);
	void getVelocity(double &lvel, double &rvel);

	void updateOdometry();
	void getRawOdometry(double &linc, double &rinc);
	void getOdometry(double &x, double &y, double &a);
	void setOdometry(double x, double y, double a);
	
	void setScale(double ls, double rs);

	bool isConnected();

	double m_per_tick;
	double robot_axle_length;
	double enc_ticks;

private:
	// #### serial port
	// ####
	int fd;
	struct termios oldtio;
	bool connected;
	
	// #### NFv2
	// ####
	void addToCommandArray(uint8_t command);
	void clearCommandArray(void);
    uint8_t txBuf[256];
    uint8_t txCnt;
    uint8_t rxBuf[256];
    uint8_t rxCnt;
    uint8_t commandArray[256];
    uint8_t commandCnt;
    uint8_t txAddr;
    
    // #### Elektron params
    // ####
	double lin_scale;
	double rot_scale;
};

#endif /* ELEKTRON_HPP_ */
