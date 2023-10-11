/*
 * ArloDriver.h
 *
 *  Created on: Apr 14, 2020
 *      Author: antonio
 */

#ifndef SRC_ARLODRIVER_H_
#define SRC_ARLODRIVER_H_

#include <vector>

using namespace std;


class ArloDriver {
public:
	ArloDriver();
	virtual ~ArloDriver();

	virtual void driveArlo(vector<double> const & inputs, vector<double>& reaction) = 0;
    virtual void setParameters(const char *iName);
};

#endif /* SRC_ARLODRIVER_H_ */
