/*
 * PID.cpp
 *
 *  Created on: 13 Dec 2022
 *      Author: kkivi
 */

#include "PID.h"

PID::PID(double p, double i, double d) {
	_p = p;
	_i = i;
	_d = d;
}

PID::~PID() {
	// TODO Auto-generated destructor stub
}

double PID::calc_out(void) {
	double error = sp - pv;
	double Kp = _p * error; // Proportional gain
	double Ki = _i * error; // Integral gain
    
}

