#include "SimulationState.h"

SimulationState::SimulationState(double energy)
{
   initialEnergy = energy;
   resetState();
}

void SimulationState::resetState()
{
   distanceToGo = 1000.0;
   distanceTravelled = 0.0;
   finishTime = 1000.0;
   currentTime = 0.0;
   // currentPosition = 0.0;
   position[0] = 0;
   position[1] = 0;
   robotEnergy = initialEnergy;
   robotDamage = 0.0;
   stuck = false;
   hasTimeRunOut = false;
   finishLineCrossed = false;
   targetReached = false;
   crashingRisk = 0.0;
}
