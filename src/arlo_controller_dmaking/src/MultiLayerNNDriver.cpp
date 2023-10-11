/*
 * Modified by: Antonio López Jaimes.
 * Date: 3-June-2018
 * Driver using a neural network with weights given
 * by an optimizer.
 */

#include <cmath>
#include "MultiLayerNNDriver.h"

/*
 * TONI: Constructor of the Driver module that setup the NN and load its weights
 * from the file with name iName.
 */
MultiLayerNNDriver::MultiLayerNNDriver(char *iName)
{
    inputName.assign(iName);
    telemetryName.assign("telemetry.csv");
    telemetryFile.open(telemetryName, std::ofstream::out);

    // IMPORTANT: The bot client is not connected in anyway to the module that
    // controls the evaluation of the population (e.g.,class TorcsSim). Therefore,
    // if the number of inputs is changed here. Then, the corresponding value
    // in TorcsSim MUST be changed.

    // Total of inputs: 5 + 19 + 36 = 60.
    //nInputs  = 5 + TRACK_SENSORS_NUM + OPPONENTS_SENSORS_NUM;

    // Total of inputs: 5 + 19 = 24.
    nInputs  = 5 + TRACK_SENSORS_NUM;  // For races with a single car (no opponents).
    nOutputs = 5; // accel, brake, clutch, steer, gear

    //numNodesLayer = {nInputs, 10, nOutputs};
    numNodesLayer = {nInputs, nOutputs}; // No hidden layers.
    numLayers = numNodesLayer.size();
    nHiddenLayers = numLayers - 2; // (Excluding input and output layers)

    // Set correct size of each vector of layer outputs (input is considered here to simplify computations later).
    for (auto nNodes : numNodesLayer)
    	layerOutputs.push_back( vector<float>(nNodes) );

    // Create the weight matrices for each pair layer i, layer i+1, i=1,2,...,numLayers-1
    for (int i = 0; i < numLayers-1; ++i)  // or nHiddenLayers+1
		weights.push_back( vector<vector<float> >(numNodesLayer[i],
				                                  vector<float>(numNodesLayer[i+1], 0.0) ) );
    readWeights(); //???
};


/*
 * TONI: Method wDrive using entirely the Neural Network outputs.
 * Challenge: Human-programmed Bots divide their behavior depending
 * on whether they are stuck or not. Could the NN learn how to do in
 * this situation?
 * Description of the input values:
 * - Angle (-pi,pi): angle between car direction and the tangent of current segment of the track.
 * - TrackPos (-inf, inf): Position between the car and the track axis. 0 means that the car is in the
 *   center, -1 means the car is the right edge, +1 means the car is in the left edge, and values
 *   beyond -1 and 1 means the car is outside the track.
 * - Track[0,...,19-1] (0,200): Values of 19 sensors which measure the distance to the
 *   edge track and the car in a range of 200 meters. In total, all the sensors cover a range
 *   of -90 to 90 degrees.
 * - RPM (0, inf): current revolutions per minute of the engine.
 * - Gear: current gear of the car.
 * - Z: elevation of the car with respect to the surface of the track.
 * - WheelSpinVel[0,...,4-1] (0, inf): 4 sensors measuring the rotation speed of each wheel in radians/seconds.
 * - SpeedX (-inf, inf): current speed along this axis.
 * - SpeedY (-inf, inf): current speed along this axis.
 * - SpeedZ (-inf, inf): current speed along this axis.
 * - DistRaced (0,inf): total distance covered by the car from the beginning.
 * - Damage (0,inf): current damage (INTEGER) during the race.
 * - Fuel (0, inf): current level of fuel (INTEGER) during the race.
 * - Opponents[0,...,36-1] (0,200): Values of 36 sensors which measure the distance to the
 *   closest opponent in a range of 200 meters. In total, all the sensors cover a range
 *   of -180 to 180 degrees.
 */
CarControl
MultiLayerNNDriver::wDrive(CarState cs)
{
   //auto input = layerOutputs.front(); // Set the input values of the NN.

   /*** 0. Get the NN's input values from the current car's state. Input layer is number 0. ***/
   /*******************************************************************************************/
   // Notation: layerOutputs[num. layer][i-th value]
   //TODO: Normalizar la entrada.
   layerOutputs[0][0] = cs.getAngle();
   layerOutputs[0][1] = cs.getTrackPos();
   layerOutputs[0][2] = cs.getGear(); // Integer [-1,6] -1: reversa, 0: neutral.
   layerOutputs[0][3] = cs.getSpeedX();
   layerOutputs[0][4] = cs.getSpeedY();
   //layerOutputs[0][5] = cs.getSpeedY();

/*
   input[0] = cs.getAngle();
   input[1] = cs.getTrackPos();
   input[2] = cs.getGear(); // Integer [-1,6] -1: reversa, 0: neutral.
   input[3] = cs.getSpeedX();
   input[4] = cs.getSpeedY();
*/
   //input[5] = cs.getSpeedZ();

   // Get each track sensor value, i, and add it to the input vector.
   int j = 5; // Contador que comienza en este ciclo y continua para el siguiente.
   for (int i = 0; i < TRACK_SENSORS_NUM; i+=1, j+=1) {
	   //input[j] = cs.getTrack(i);
       layerOutputs[0][j] = cs.getTrack(i);
   }

   //NOTA: Comment out this loop to EXCLUDE opponents sensors from input.
   //for (int i = 0; i < OPPONENTS_SENSORS_NUM; i+=1, j++)
	//   layerOutputs[0][j] = cs.getOpponents(i);

   /*** 1. Compute NN outputs ***/
   /*****************************/
   nnOuputs();

   float accel, brake, steer,clutch;
   int gear;
   auto output = layerOutputs.back(); // Get the computed output values vector.
   accel  = output[0]; // [0,1]
   brake  = output[1]; // [0,1]
   clutch = output[2]; // [0,1]
   steer  = output[3]; // [-1,1]
   gear   = output[4]; // [-1,6] -1: reversa, 0: neutral.

   //printOutputs(layerOutputs[nHiddenLayers+1]);
   checkOutputs(output);

   /*** 2. Use those outputs to send the reaction to the server ***/
   /***************************************************************/
//   clutch=0;
   CarControl cc(accel, brake, gear, steer, clutch);
   //CarControl cc(1, 1, 1, -1, 1);

   recordTelemetry(cc);

   return cc;
}

int MultiLayerNNDriver::numWeights() {

	int counter = 0;
	for (int i = 0; i < numLayers-1; ++i)
		counter += numNodesLayer[i]*numNodesLayer[i+1];

	return counter;
}

void MultiLayerNNDriver::checkOutputs(vector<float > &y) {
   if ( y[0] < 0 ||  y[0] > 1) {
      cerr << "ERROR: ACELARCIÓN FUERA DE INTERVALO: " << y[0] << endl;
      cerr << "Archivo entrada: " << inputName << endl;
      exit(1);
   }

   if ( y[1] < 0 ||  y[1] > 1) {
      cerr << "ERROR: FRENO FUERA DE INTERVALO " << y[1] << endl;
      cerr << "Archivo entrada: " << inputName << endl;
      exit(1);
   }

   if ( y[2] < 0 ||  y[2] > 1) {
      cerr << "ERROR: CLUTCH FUERA DE INTERVALO " << y[2] << endl;
      cerr << "Archivo entrada: " << inputName << endl;
      exit(1);
   }

   if ( y[3] < -1 ||  y[3] > 1) {
      cerr << "ERROR: VOLANTE FUERA DE INTERVALO " << y[3] << endl;
      cerr << "Archivo entrada: " << inputName << endl;
      exit(1);
   }

   if ( y[4] < 1 ||  y[4] > 6) {
      cerr << "ERROR: VELOCIDAD FUERA DE INTERVALO " << y[4] << endl;
      cerr << "Archivo entrada: " << inputName << endl;
      exit(1);
   }
}

void MultiLayerNNDriver::printOutputs(vector<float > &y) {
   cerr << "\n accel = " << y[0] << "\n";
   cerr << "brake = " << y[1] << "\n";
   cerr << "clutch= " << y[2] << "\n";
   cerr << "steer = " << y[3] << "\n";
   cerr << "gear  = " << y[4] << "\n";
}

void MultiLayerNNDriver::readWeights() {
    cout << "NNDriver: reading weight values for the Neural Network." << endl;
    cout << "NNDriver: the input file is " << inputName << endl;
    cout << "NNDriver: the format is #inputs #outputs #hidLayers #size_h1 ... #size_hN" << endl;

    weightsFile.open(inputName, std::ifstream::in);

    // Read number of inputs, outputs and number of hidden layers from the weight file
    int nInputNodes;
    int nOutputNodes;
    int nHidLayers;

    weightsFile >> nInputNodes;
    weightsFile >> nOutputNodes;
    weightsFile >> nHidLayers;

    cerr << "NNDriver: inputs=" << nInputs
         << ", hid layers=" << nHiddenLayers
         << ", outputs=" << nOutputs << endl;

    // Check whether the structure of the read NN is the same of the one expected.
    if (nInputNodes != nInputs || nOutputNodes != nOutputs || nHidLayers != nHiddenLayers) {
    	cerr << "\n\n NNDriver: The structure of the NN is not the one expected.\n\n" << endl;
    	cerr << "NNDriver: inputs (read)=" << nInputNodes
    	         << ", hid layers (read)=" << nHidLayers
    	         << ", outputs (read)=" << nOutputNodes << endl;
    	exit(0);
    }

    // NOTE: We assume that the number of nodes in hidden layers is correct.

	// Read the weights for every layer in the NN.
    for (auto& weightMatrix : weights)
    	for (auto& row : weightMatrix)
    		for (auto& element : row)
    			weightsFile >> element;


//	for (int k = 0; k < numLayers-1; ++k) { // For each pair of layers.
//		for (int i = 0; i < numNodesLayer[k]; ++i) {
//			for (int j = 0; j < numNodesLayer[k+1]; ++j) {
//				weightsFile >> currentWeight;
//				weights[k][i][j] = currentWeight;
//			}
//		}
//	}

    weightsFile.close();

    //printWeights();
}


void MultiLayerNNDriver::printWeights(){
    cout << "NNDriver: the weight are:" << endl;

    for (auto weightMatrix : weights) {
    	for (auto row : weightMatrix ) {
    		for (auto element : row)
    			cout << element << "\t";
    		cout << endl; // New line after each row
    	}
    	cout << endl; // Additional new line after each entire weight matrix.
    }
}

void MultiLayerNNDriver::recordTelemetry(CarControl &c) {
   telemetryFile << c.getAccel()  << "\t"
                 << c.getBrake()  << "\t"
                 << c.getClutch() << "\t"
                 << c.getSteer()  << "\t"
                 << c.getGear()   << endl;

//   cout << c.getAccel()  << "\t"
//                 << c.getBrake()  << "\t"
//                 << c.getClutch() << "\t"
//                 << c.getSteer()  << "\t"
//                 << c.getGear()   << endl;
}

/* Network with n hidden layers + output layer. */
//void MultiLayerNNDriver::nnOuputs(vector<float > &x, vector<float > &y)
//{
//    //cerr << "\n --------> Calling nnOutputs with ONE HIDDEN LAYER <---------" << endl;
//
//    // Compute output for the first hidden layer
//    // For each node j of the hidden layer_1: Weights output X layer_1
//    for (int j=0; j < numNodesLayer[1]; j++) { // Index 0 is for the input layer (not used here).
//    	layerOutputs[1][j] = 0.0;
//
//        // For each input i of X
//        for (int i=0; i < nInputs; i++)
//        	layerOutputs[1][j] += x[i]*weights[0][i][j];
//
//        // Normalize each output of the hidden layer.
//        sigmoid(layerOutputs[1][j], 0.00001);
//    }
//
//    // Compute output for each hidden layer 2 <= k <= nHiddenLayer
//    // Weights layer_k-1 X layer_k
//    for (int k = 2; k < nHiddenLayers+1; ++k) {
//
//		// For each node j of hidden layer k
//    	for (int j=0; j < numNodesLayer[k]; j++) {
//    		layerOutputs[k][j] = 0.0;
//
//    		// For each input i of layer k-1
//    		for (int i=0; i < numNodesLayer[k-1]; i++)
//    			layerOutputs[k][j] += layerOutputs[k-1][i]*weights[k-1][i][j];
//
//    		// Normalize each output of the hidden layer.
//    		sigmoid(layerOutputs[k][j], 0.00001);
//    	}
//    }
//
//    // Compute output layer
//    // For each node j of the output layer: Weights layer_k X output
//    for (int j=0; j < nOutputs; j++) {
//    	y[j] = 0.0;
//
//    	// For each input i from hidden layer k (last one)
//        for (int i=0; i < numNodesLayer[nHiddenLayers]; i++)
//        	y[j] += layerOutputs[nHiddenLayers][i]*weights[nHiddenLayers][i][j];
//
//
//        // Normalize each output of the hidden layer.
//        sigmoid(y[j], 0.00001);
//    }
//
//    normalizeOutputs(y);
//}

/* Network with n hidden layers + output layer. */
void MultiLayerNNDriver::nnOuputs()
{
    //cerr << "\n --------> Calling nnOutputs with ONE HIDDEN LAYER <---------" << endl;


    // Here the input layer is layerOutput[0] (given). Others have to be computed.
	 // Layers are k=0 (input), k=1,...,k=numLayers-1 (output)
	 // Compute outputs for each layer 1 <= k <= numLayers-1 (counting input & output layer)
    // Weights layer_k-1 X layer_k
	 // k-1 = 0 is the input layer.
    for (int k = 1; k < numLayers; ++k) {

		// For each node j of hidden layer k, compute its output.
    	for (int j=0; j < numNodesLayer[k]; j++) {
    		layerOutputs[k][j] = 0.0;

    		// For each input i of layer k-1
    		for (int i=0; i < numNodesLayer[k-1]; i++)
    			layerOutputs[k][j] += layerOutputs[k-1][i]*weights[k-1][i][j];

    		// Normalize each output of the hidden layer.
    		//layerOutputs[k][j] = sigmoid(layerOutputs[k][j], 0.01);
    	}
    }

    //For the output values of the NN
    adjustOutputs(layerOutputs[numLayers-1]);
}

void MultiLayerNNDriver::adjustOutputs(vector<float > &y) {
   y[0] = sigmoid(y[0], 0.00001);
   y[1] = sigmoid(y[1], 0.0001);
   y[2] = sigmoid(y[2], 0.001);
   y[3] = sigmoid(y[3], 0.00001);
   y[4] = sigmoid(y[4], 0.0001);

   // y[0],..y[2] keep their original value.
   y[3] =   2*y[3] - 1; // [-1,1] Steer.
   //y[4] = round(4*y[4]) + 1; // [1,5] Gear,  -1: reversa, 0: neutral.
   y[4] = round(3*y[4]) + 1; // [1,4] Gear,  -1: reversa, 0: neutral.
}


/**
 * The normalization greatly depends on the specific actuators chosen.
 */
void MultiLayerNNDriver::normalizeOutputs(vector<float > &y) {
   y[0] = sigmoid(y[0], 0.0000001); // [0,1] Accel
   y[1] = sigmoid(y[1], 0.00001); // [0,1] Brake
   y[2] = sigmoid(y[2], 0.0001); // [0,1] Clutch
   y[3] = 2*sigmoid(y[3], 0.000001) - 1; // [-1,1] Steer
   //y[4] = round(7*sigmoid(y[4]) - 1); // [-1,6] Gear,  -1: reversa, 0: neutral.
   y[4] = round(2*sigmoid(y[4], 0.00001) + 1); // [-1,6] Gear,  -1: reversa, 0: neutral.

   if (y[0] < 0.01)
      y[0] = 0.0;
   if (y[0] > 0.99)
      y[0] = 1.0;

   if (y[1] < 0.01)
      y[1] = 0.0;
   if (y[1] > 0.99)
      y[1] = 1.0;

   if (y[2] < 0.01)
      y[2] = 0.0;
   if (y[2] > 0.99)
      y[2] = 1.0;

   if (y[3] < -0.99)
      y[3] = -1.0;
   if (y[3] > 0.99)
      y[3] = 1.0;

   if (y[4] < -1)
      y[4] = -1;
   if (y[4] > 6)
      y[4] = 6;
}

double MultiLayerNNDriver::sigmoid(float x, float factor) {
   return (1.0 / (1.0 + exp(-factor*x)));
}

void MultiLayerNNDriver::onShutdown()
{
   cout << "The server should write the output values" << endl;
   telemetryFile.close();
}

void MultiLayerNNDriver::onRestart()
{
    cout << "Restarting the race!!." << endl;
}

void MultiLayerNNDriver::init(float *angles)
{
	// set angles as {-90,-75,-60,-45,-30,20,15,10,5,0,5,10,15,20,30,45,60,75,90}

	for (int i=0; i<5; i++)
	{
		angles[i]=-90+i*15;
		angles[18-i]=90-i*15;
	}

	for (int i=5; i<9; i++)
	{
			angles[i]=-20+(i-5)*5;
			angles[18-i]=20-(i-5)*5;
	}
	angles[9]=0;
}


