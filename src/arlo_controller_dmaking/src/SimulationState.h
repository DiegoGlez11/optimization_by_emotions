#ifndef SIMULATIONSTATE_H
# define SIMULATIONSTATE_H

class SimulationState {

public:
   SimulationState(double energy = 1000);

   /* Distancia para llegar al punto meta. */
   double distanceToGo;

   /* Distancia que ha recorrido el robot. */
   double distanceTravelled;

   /* Tiempo en que llegó a la meta el robot. */
   double finishTime;

   /* Tiempo actual del recorrido. */
   double currentTime;

   /* Posición actual en el eje x. */
   //double currentPosition;

   /* Posición x,y del robot. */
   double position[2];

   /* Energía actual del robot. */
   double robotEnergy;

   /* Energía actual del robot. */
   double initialEnergy;

   /* Daño al robot. */
   double robotDamage;
   bool stuck;
   bool hasTimeRunOut;

   /* Indica si el robot ha cruzada la línea de meta (algunos escenarios)*/
	bool finishLineCrossed;

   /* Indica si el robot ha llegado al punto meta. */
   bool targetReached;

   /* distancia más pequeña a la que ha pasado de un obstáculo o pared. */
   double crashingRisk;

   double finalSpeed;

   void resetState();
};

#endif
