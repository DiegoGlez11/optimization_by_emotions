/*
 * NeuroControllerDriver.h
 *
 *  Created on: Apr 14, 2020
 *      Author: antonio
 */

#ifndef SRC_NEUROCONTROLLERDRIVER_H_
#define SRC_NEUROCONTROLLERDRIVER_H_

#include "ros/ros.h"
#include "ArloDriver.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <sys/stat.h>
#include "neurocontroller_database/load_ind.h"

#define INPUT_LAYER 0

using namespace std;
// struct stat info;

class NeuroControllerDriver : public ArloDriver {
public:
	NeuroControllerDriver(int nIn, int nOut, vector<pair<double,double> > &oRanges);
	virtual ~NeuroControllerDriver();

	virtual void driveArlo(vector<double> const & inputs, vector<double>& reaction);
    virtual void setParameters(const char* weightsFile, int num_ind = -1);
	virtual int numWeights();

private:
    double maxInputValue;
    double minInputValue;
    
    /* Input file of NN weights */
    ifstream weightsFile;
    ofstream telemetryFile;
    string telemetryName;
    string inputName;         // Name of the configuration file.

    /***  The NN structure ***/
    int nInputs;        // Number of inputs. Taken from input file
    int nOutputs;       // Number of outputs. Taken from input file
    vector<pair<double,double> > oBounds;  // Range for each output value.

    int numLayers;               // Number of total layers including input, output and hidden layers.
    int nHiddenLayers;           // Number of hidden layers.
    int bias;
    vector<int> numNodesLayer;   // Number of nodes of each layer.
    vector<vector<double> > layerOutputs;   // Outputs of each layer (input layer values are considered the first output values).
    vector<vector<vector<double> > > weights; // Set of all the weight matrices (array of matrices).

    virtual void readWeights();
    virtual void printWeights();
    void printOutputs(vector<double > &y);
    void checkOutputs(vector<double > &y);

    /* Compute NN's outputs. */
    virtual void nnOuputs();
    void adjustOutputs(vector<double > &y);
    double sigmoid(double x, double factor=1.0);
};

#endif /* SRC_NEUROCONTROLLERDRIVER_H_ */
