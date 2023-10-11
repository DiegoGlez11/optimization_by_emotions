/*
 * NeuroControllerDriver.h
 *
 *  Created on: Apr 14, 2020
 *      Author: antonio
 */

#ifndef SIMPLEDRIVER_H_
#define SIMPLEDRIVER_H_

#include "ArloDriver.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

#define INPUT_LAYER 0

using namespace std;

class SimpleDriver : public ArloDriver {
public:
	SimpleDriver();
	virtual ~SimpleDriver();

	virtual void driveArlo(vector<double> const & inputs, vector<double>& reaction);

private:
};

#endif /* SIMPLEDRIVER_H_ */
