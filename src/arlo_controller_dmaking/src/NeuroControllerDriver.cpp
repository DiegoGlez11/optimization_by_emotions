/*
 * NeuroControllerDriver.cpp
 *
 *  Created on: Apr 14, 2020
 *      Author: antonio
 */

#include "NeuroControllerDriver.h"
#include <boost/filesystem.hpp>

inline bool is_file(const std::string &name)
{
   struct stat buffer;
   return (stat(name.c_str(), &buffer) == 0);
}

NeuroControllerDriver::NeuroControllerDriver(int nIn, int nOut, vector<pair<double, double>> &oRanges)
{
   nInputs = nIn;   //
   nOutputs = nOut; //
   oBounds = oRanges;
   bias = 0;

   maxInputValue = -10000;
   minInputValue = 10000;

   // numNodesLayer = {nInputs, 10, nOutputs};
   numNodesLayer = {nInputs + bias, nOutputs}; // No hidden layers. Adding pseudo input
   numLayers = numNodesLayer.size();
   nHiddenLayers = numLayers - 2; // (Excluding input and output layers)

   // Set correct size of each vector of layer outputs (input is considered here to simplify computations later).
   for (auto nNodes : numNodesLayer)
      layerOutputs.push_back(vector<double>(nNodes));

   // Create the weight matrices for each pair layer i, layer i+1, i=1,2,...,numLayers-1
   for (int i = 0; i < numLayers - 1; ++i) // or nHiddenLayers+1
      weights.push_back(vector<vector<double>>(numNodesLayer[i],
                                               vector<double>(numNodesLayer[i + 1], 0.0)));
}

NeuroControllerDriver::~NeuroControllerDriver()
{
}

void NeuroControllerDriver::setParameters(const char *weightsFile, int num_ind)
{
   inputName.assign(weightsFile);
   if (is_file(weightsFile))
   {
      readWeights();
      // printWeights();
      return;
   }

   // si se desea cargar un individuo optimizado con anterioridad
   size_t pos = inputName.find("optimized");
   if (pos != std::string::npos)
   {
      ros::NodeHandle nh;
      ros::ServiceClient srv = nh.serviceClient<neurocontroller_database::load_ind>("load_ind_var_space");

      // param del servicio
      neurocontroller_database::load_ind msg;
      msg.request.id_pareto_front = inputName;
      msg.request.num_ind = num_ind;
      cout << "-" << msg.request.id_pareto_front << "-" << endl;
      // se llama al servicio ros
      if (srv.call(msg))
      {
         int i = 0;

         // vector de pesos de la red, inicialización
         weights.clear();
         int numLayers = 2;
         for (int h = 0; h < numLayers - 1; ++h)
         {
            weights.push_back(vector<vector<double>>(50,               // input neurons
                                                     vector<double>(2, // outputneurons
                                                                    0.0)));
         }

         // se carga el neurocontrolador (individuo)
         std::vector<double> w = msg.response.var_space;
         for (auto &weightMatrix : weights)
            for (auto &row : weightMatrix)
               for (auto &matrixEntry : row)
               {
                  matrixEntry = w[i];
                  i++;
                  // cout << std::setprecision(15) <<matrixEntry << endl;
               }
      }
      else
      {
         cerr << "Error al cargar el neurocontrolador. ROS ERROR\n";
      }
   }
}

// void NeuroControllerDriver::get_weights()
// {

// }

void NeuroControllerDriver::driveArlo(const vector<double> &inputs, vector<double> &reaction)
{
   // cout << "Entrando a driveArlo\n";
   /*** 0. Get the NN's input values from the current robot's state. ***/
   layerOutputs[INPUT_LAYER] = inputs;
   // cout << " drive ------"<< endl;
   for (int i = 0; i < layerOutputs[INPUT_LAYER].size(); ++i)
   {
      // cout << layerOutputs[INPUT_LAYER][i] << endl;

      if (isinf(layerOutputs[INPUT_LAYER][i]))
         layerOutputs[INPUT_LAYER][i] = 4;
      // cout << layerOutputs[INPUT_LAYER][i] << ", ";
   }
   // cout << "\n";

   /*** 1. Compute NN outputs ***/
   /*****************************/
   nnOuputs();
   // cout << "Ya se calculo la salida de la red.\n";

   reaction = layerOutputs.back(); // Get the computed output values vector.

   // printOutputs(layerOutputs[nHiddenLayers+1]);
   checkOutputs(reaction);

   // cout << "En NN: reaction[0]= " << reaction[0] << "\n";
   // cout << "En NN: reaction[1]= " << reaction[1] << "\n";
}

/* Network with n hidden layers + output layer. */
void NeuroControllerDriver::nnOuputs()
{
   // cerr << "\n --------> Calling nnOutputs with ONE HIDDEN LAYER <---------" << endl;

   // Here the input layer is layerOutput[0] (given). Others have to be computed.
   // Layers are k=0 (input), k=1,..., k=numLayers-1 (output)
   // Compute outputs for each layer 1 <= k <= numLayers-1 (counting input & output layer)
   // Weights layer_k-1 X layer_k
   // k-1 = 0 is the input layer.
   for (int k = 1; k < numLayers; ++k)
   {

      // For each node j of hidden layer k, compute its output.
      for (int j = 0; j < numNodesLayer[k]; j++)
      {
         layerOutputs[k][j] = 0.0;

         // For each input i of layer k-1
         for (int i = 0; i < numNodesLayer[k - 1]; i++)
            layerOutputs[k][j] += layerOutputs[k - 1][i] * weights[k - 1][i][j];

         // Normalize each output of the hidden layer.
         // layerOutputs[k][j] = sigmoid(layerOutputs[k][j], 0.01);
      }
   }

   // For the output values of the NN
   adjustOutputs(layerOutputs[numLayers - 1]);
}

int NeuroControllerDriver::numWeights()
{

   int counter = 0;
   for (int i = 0; i < numLayers - 1; ++i)
      counter += numNodesLayer[i] * numNodesLayer[i + 1];

   return counter;
}

void NeuroControllerDriver::checkOutputs(vector<double> &y)
{
   for (int i = 0; i < y.size(); ++i)
   {
      if (y[i] < oBounds[i].first || y[i] > oBounds[i].second)
      {
         cerr << "ERROR en el valor de la salida " << i << ": " << y[i] << endl;
         cerr << "Archivo entrada: " << inputName << endl;
         exit(1);
      }
   }
}

void NeuroControllerDriver::printOutputs(vector<double> &y)
{
   cerr << "\n primer salida = " << y[0] << "\n";
   cerr << "segunda salida = " << y[1] << "\n";
}

void NeuroControllerDriver::readWeights()
{
   //  cout << "NNDriver: reading weight values for the Neural Network." << endl;
   //  cout << "NNDriver: the input file is " << inputName << endl;
   //  cout << "NNDriver: the format is #inputs #outputs #hidLayers #size_h1 ... #size_hN" << endl;

   weightsFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);

   try
   {
      weightsFile.open(inputName, std::ifstream::in);

      // Read number of inputs, outputs and number of hidden layers from the weight file
      int nInputNodes;
      int nOutputNodes;
      int nHidLayers;

      weightsFile >> nInputNodes;
      weightsFile >> nOutputNodes;
      weightsFile >> nHidLayers;

      //  cerr << "NNDriver: inputs=" << nInputs
      //       << ", hid layers=" << nHiddenLayers
      //       << ", outputs=" << nOutputs << endl;

      // Check whether the structure of the read NN is the same of the one expected.
      if (nInputNodes != nInputs || nOutputNodes != nOutputs || nHidLayers != nHiddenLayers)
      {
         cerr << "\n\n NNDriver: The structure of the NN is not the one expected.\n\n"
              << endl;
         cerr << "NNDriver: inputs (read)=" << nInputNodes
              << ", hid layers (read)=" << nHidLayers
              << ", outputs (read)=" << nOutputNodes << endl;
         exit(0);
      }

      // NOTE: We assume that the number of nodes in hidden layers is correct.

      // Read the weights for every layer in the NN.
      for (auto &weightMatrix : weights)
         for (auto &row : weightMatrix)
            for (auto &matrixEntry : row)
            {
               weightsFile >> matrixEntry; // writes into weights matrix.
               // cout << matrixEntry << endl;
            }

      weightsFile.close();
      // printWeights();
   }
   catch (ifstream::failure e)
   {
      cerr << "No se pudo abrir el achivo: " << inputName << "\n";
      exit(0);
   }
}

void NeuroControllerDriver::printWeights()
{
   cout << "NNDriver: the weight are:" << endl;

   for (auto weightMatrix : weights)
   {
      cout << "Rows: " << weightMatrix.size() << "Cols: " << weightMatrix[0].size() << endl;
      for (auto row : weightMatrix)
      {
         for (auto matrixEntry : row)
            cout << matrixEntry << "\t";
         cout << endl; // New line after each row
      }
      cout << endl; // Additional new line after each entire weight matrix.
   }
}

void NeuroControllerDriver::adjustOutputs(vector<double> &y)
{
   if (maxInputValue < y[0])
   {
      maxInputValue = y[0];
      cerr << "maxVal = " << maxInputValue << endl;
   }

   if (minInputValue > y[0])
   {
      minInputValue = y[0];
      cerr << "minVal = " << minInputValue << endl;
   }

   // cerr << "input 0: " << y[0] << endl;

   vector<double> factor = {0.0095, 0.1};
   for (int i = 0; i < y.size(); ++i)
   {
      y[i] = sigmoid(y[i], factor[i]);
      y[i] = oBounds[i].first + (oBounds[i].second - oBounds[i].first) * y[i];
   }
}

double NeuroControllerDriver::sigmoid(double x, double factor)
{
   return (1.0 / (1.0 + exp(-factor * x)));
}
