#ifndef SIMULATIONSTATE_H
# define SIMULATIONSTATE_H

class SimulationState {

public:
   SimulationState(double energy = 1000);

   double distanceToGo;
   double distanceTravelled;
   double finishTime;
   double currentTime;
   double currentPosition;
   double position[2];
   double robotEnergy;
   double initialEnergy;
   double robotDamage;
   bool stuck;
   bool hasTimeRunOut;
	bool finishLineCrossed;

   void resetState();
};

#endif
