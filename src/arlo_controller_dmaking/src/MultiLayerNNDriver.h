/*
 * Modified by: Antonio López Jaimes.
 * Date: 19-February-2019
 * Driver using a multi-layer neural network with weights given
 * by an optimizer.
 */

#ifndef MULTILAYERNNDRIVER_H_
#define MULTILAYERNNDRIVER_H_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include "BaseDriver.h"
#include "CarState.h"
#include "CarControl.h"
#include "SimpleParser.h"
#include "WrapperBaseDriver.h"

#define PI 3.14159265

using namespace std;

class MultiLayerNNDriver : public WrapperBaseDriver
{
public:
	
	// Constructor
	MultiLayerNNDriver(char *iName);

	// NNDriver implements a simple and heuristic controller for driving
	virtual CarControl wDrive(CarState cs);

	// Print a shutdown message 
	virtual void onShutdown();
	
	// Print a restart message 
	virtual void onRestart();

	// Initialization of the desired angles for the rangefinders
	virtual void init(float *angles);

	//TONI: Regresa el número de pesos totales.
	virtual void readWeights();
	virtual int numWeights();

protected:
    /* Input file of NN weights */
    ifstream weightsFile;
    ofstream telemetryFile;
    string telemetryName;
    string inputName;         // Name of the configuration file.

    // IMPORTANT: The client does not has access to all the objective values, because
    // it only sends its reaction to the sensors values. It has access to damage or fuel
    // because they are sensor values, but it does not has access to the total time, or
    // total distance.
    // THEREFORE, the server is the one who must write the objective values. It has access
    // to every value at the end of the race.
    
    /***  The NN structure ***/
    int nInputs;        // Number of inputs. Taken from input file
    int nOutputs;       // Number of outputs. Taken from input file
    //vector<float > *x;  // Input vector. Values from the sensors.
    //vector<float > *y;  // Output vector. Values for the actuators.

    int numLayers;               // Number of total layers including input, output and hidden layers.
    int nHiddenLayers;           // Number of hidden layers.
    vector<int> numNodesLayer;   // Number of nodes of each layer.
    vector<vector<float> > layerOutputs;   // Outputs of each layer (input layer values are considered the first output values).

    vector<vector<vector<float> > > weights; // Set of all the weight matrices (array of matrices).
    //int numWeights;      // Number of total weights.
	
 //   virtual void readWeights();
    virtual void printWeights();
    void printOutputs(vector<float > &y);
    void checkOutputs(vector<float > &y);
    void recordTelemetry(CarControl &c);
    
    /* Compute NN's outputs. */
    virtual void nnOuputs();
    void normalizeOutputs(vector<float > &y);
    void adjustOutputs(vector<float > &y);
    double sigmoid(float x, float factor=1.0);
};

#endif /*MULTILAYERNNDRIVER_H_*/
