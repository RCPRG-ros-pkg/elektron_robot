/*
 * elektron.cpp
 *
 *  Created on: Sep 5, 2009
 *      Author: konradb3
 */

#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <cstring>
#include <iostream>

#include "elektron.hpp"

using namespace std;

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

Protonek::Protonek(const std::string& port, int baud) {
	connected = false;

	llpos = 0;
	lrpos = 0;

	xpos = 0;
	ypos = 0;
	apos = 0;

	setvel.start = 'x';
	setvel.cmd = 'a';
	setvel.lvel = 0;
	setvel.rvel = 0;

	odom_initialized = false;

	robot_axle_length = AXLE_LENGTH;
	m_per_tick = M_PI * WHEEL_DIAM / ENC_TICKS;
	enc_ticks = ENC_TICKS;

	lin_scale = 1.0;
	rot_scale = 1.0;

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

	_dump = false;
}

Protonek::~Protonek() {
	// restore old port settings
	if (fd > 0)
		tcsetattr(fd, TCSANOW, &oldtio);
	close(fd);
}

void Protonek::setParams(double ls, double rs) {
	lin_scale = ls;
	rot_scale = rs;
}

void Protonek::dump() {
	_dump = true;
	of.open("/tmp/odom_dump.txt");
	of << "lpos;lindex;rpos;rindex;lvel;rvel;apos;xpos;ypos\n";
}

void Protonek::update() {
	unsigned int ret = 0;
	tcflush(fd, TCIFLUSH);
	write(fd, &setvel, sizeof(setvel));

	while (ret < sizeof(getdata))
		ret += read(fd, ((char*) &getdata) + ret, sizeof(getdata) - ret);

}

void Protonek::setVelocity(double lvel, double rvel) {
	setvel.lvel = (int16_t)(lvel * (1 / m_per_tick) * 0.1); // Convert SI units to internal units
	setvel.rvel = (int16_t)(rvel * (1 / m_per_tick) * 0.1);

	if (setvel.rvel > MAX_VEL)
		setvel.rvel = MAX_VEL;
	else if (setvel.rvel < -MAX_VEL)
		setvel.rvel = -MAX_VEL;

	if (setvel.lvel > MAX_VEL)
		setvel.lvel = MAX_VEL;
	else if (setvel.lvel < -MAX_VEL)
		setvel.lvel = -MAX_VEL;
}

void Protonek::getVelocity(double &xvel, double &thvel) {
	static int maxl = 0, maxr = 0;
	double lvel = (double) (getdata.lvel) * m_per_tick * 10;
	double rvel = (double) (getdata.rvel) * m_per_tick * 10;

	if (getdata.lvel > maxl) maxl = getdata.lvel;
	if (getdata.rvel > maxr) maxr = getdata.rvel;

	//std::cout << maxl << " " << maxr << "\n";
	xvel = (lvel + rvel) * 0.5;
	thvel = (lvel - rvel) / AXLE_LENGTH;
}

void Protonek::updateOdometry() {

	//std::cout << "lpos: " << getdata.lpos << ", rpos: " << getdata.rpos << " lindex: " << getdata.lindex << " rindex: " << getdata.rindex << "\n";

	//std::cout << "vel: " << getdata.lvel << " " << getdata.rvel << "\n";

	double lpos = getdata.lpos + enc_ticks * getdata.lindex;
	double rpos = getdata.rpos + enc_ticks * getdata.rindex;



	double linc = (double) (llpos - lpos) * m_per_tick * lin_scale;
	double rinc = (double) (lrpos - rpos) * m_per_tick * lin_scale;
	//std::cout << "linc: " << (llpos - lpos) << ", rinc: " << (lrpos - rpos) << "\n";

	llpos = lpos;
	lrpos = rpos;
	if (odom_initialized == true) {
		apos -= (linc - rinc) / robot_axle_length * rot_scale;
		apos = ang_nor_rad(apos);
		double dist = (rinc + linc) / 2;

		xpos += dist * cos(apos);
		ypos += dist * sin(apos);

		if (_dump) {
			of << getdata.lpos << ";" << getdata.lindex << ";" << getdata.rpos << ";" << getdata.rindex << ";" 
				<< getdata.lvel << ";" << getdata.rvel << ";"
				<< apos << ";" << xpos << ";" << ypos << "\n";				
		}
	} else
		odom_initialized = true;

}

void Protonek::getOdometry(double &x, double &y, double &a) {
	x = xpos;
	y = ypos;
	a = apos;
}

void Protonek::setOdometry(double x, double y, double a) {
	xpos = x;
	ypos = y;
	apos = a;
}

void Protonek::getRawOdometry(double &linc, double &rinc) {
	int lpos = getdata.lpos + getdata.lindex * enc_ticks;
	int rpos = getdata.rpos + getdata.rindex * enc_ticks;

	linc = (lpos - llpos) * m_per_tick;
	rinc = (rpos - lrpos) * m_per_tick;

	llpos = lpos;
	lrpos = rpos;
}

bool Protonek::isConnected() {
	return connected;
}
