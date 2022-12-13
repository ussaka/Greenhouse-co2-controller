/*
 * PID.h
 *
 *  Created on: 13 Dec 2022
 *      Author: kkivi
 */

#ifndef PID_H_
#define PID_H_

class PID {
public:
	PID(double p, double i, double d);
	virtual ~PID();
	double calc_out();

private:
	double pv;
	double sp;
	double _p;
	double _i;
	double _d;
};

#endif /* PID_H_ */
