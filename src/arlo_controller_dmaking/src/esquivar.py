#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def esquivar():
   # Acá usarás la función matemática como ya lo estás haciendo para
   # calcular las velocidades.
   vLineal = 1
   vAngular = 1

   return vLineal, vAngular


def controlarRobot():
   # Crear publicador para el tópico de las llantas del robot.
   pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

   # Cuántas iteraciones por segundo hará el siguiente ciclo.
   # Es decir, qué tan frecuente se mandarán los comandos (velocidades) al robot.
   rate = rospy.Rate(10)  # 10hz: 10 veces por segundo

   avanzar = False
   esquivar = True

   while not rospy.is_shutdown():
      # 0. Cuando quieras que automáticamente comience a hacer la maniobra
      #    Deberías tener una variable que cambie con la función de callback
      vel_msg = Twist()

      if esquivar:
         # 1. Hacer cálculos de velocidad lineal y angular para esquivar
         vel_msg.linear.x, vel_msg.angular.z = esquivar()

      elif avanzar:
         # Para probar la maniobra de esquivar, 'avanzar' será falso desde el inicio.
         # 1. Hacer cálculos para avanzar
         # Poner aquí asignación de valores a vel_msg
         pass

      # Sea cual sea la acción (avanzar o esquivar) publicar el comando.
      pubVel.publish(vel_msg)


      # Dormir lo que se necesite para que cumplir con la frecuencia del ciclo.
      rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('prueba_robot', anonymous=True)

        # Para cuando ya necesites tener información de Gazebo hacia tu nodo (p.ej. sensores),
        # en algún punto de acá te debes suscribir al tópico y poner qué función se 
        # llamará cada que llegue un mensaje.

        controlarRobot()
    except rospy.ROSInterruptException:
        pass

