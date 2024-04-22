#!/usr/bin/env python3


import rospy
import numpy as np
from arlo_controller_dmaking.srv import EvaluateDriver, EvaluateDriverResponse
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock as Cl
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState
from arlo_controller_dmaking.srv import play_song as PlaySong
from arlo_controller_dmaking.srv import control_robot as ControlRobot
from arlo_controller_dmaking.srv import control_robotResponse
from user_interaction.srv import change_color
import tf
import math
from tensorflow import convert_to_tensor

def resetState():
    return {"timeRunOut": False, 
            "stuck": False,
            "finishLineCrossed": False, 
            "finishTime": 0,
            "targetReached": False, 
            "crashingRisk":0,
            "robotEnergy":100,
            "robotDamage":0,
            "position":[0,0,0],
            "distanceToGo":1000,
            "distanceTravelled":0,
            "currentTime":0}
    
simulationState = resetState()


class simulationController:
    NUM_RAYS = 16
    n_aux  = 0
    is_driving = False
    stop_driving = False

    maxSimTime = 300
    touchThreshold = 0
    stuckCounter = 0
    closestHitDistance = 100
    newTau = 0
    tauInicial = 0.5
    alphaValue = 0.4
    prev_x = 0
    prev_y = 0
    deltaAngle = .1459

    linear_ = 0
    angular_ = 0
    l_scale_ = 1
    a_scale_ = 1
    sensorValues = np.zeros(NUM_RAYS * 3 + 2) # guarda los valores de los sensores y delta x,y
    driver = None
    vel_pub_ = None

    def __init__(self) :
        # connect to topic for control robot
        self.vel_pub_ = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        odom_sub_ = rospy.Subscriber("odom", Odometry, self.checkModelPosition)
        clock_sub_ = rospy.Subscriber("clock", Cl,self.checkSimulationTime)
        sonar_l_sub_ = rospy.Subscriber("arlo/laser/scan_left", LaserScan,self.checkSonarLeftValues)
        sonar_c_sub_ = rospy.Subscriber("arlo/laser/scan_center", LaserScan,self.checkSonarCenterValues)
        sonar_r_sub_ = rospy.Subscriber("arlo/laser/scan_right", LaserScan,self.checkSonarRightValues)
        service = rospy.Service("evaluate_driver", EvaluateDriver,self.evaluateDriver)
        srvControl_robot = rospy.Service("control_robot", ControlRobot,self.control_robot)
        
    def setDriver(self, driver):
        self.driver = driver
        self.sensorValues = np.zeros(driver.getNumInputs())


    def simulation_color(self, ctl_color):
        try:
            srv = rospy.ServiceProxy("change_color", change_color)
            srv(ctl_color)
        except Exception as e:
            print(f"Error al cambiar el color de la ventana de estimulacion:\n{e}")
           
            
    def startSimulation(self, driver, maxtime, t_threshold):
        global simulationState
        print("\nStarting the simulation of a new driver...")
        
        # reset simulation
        srv_r = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)
        srv_r()
        # play sound (engine sound included)
        self.control_sound("unpause_world")
        # stop stimulation of simulation result
        self.simulation_color("hide")
        # sleep before to start simulation
        t = rospy.Duration(0.1)
        rospy.sleep(t)
        # maximum simulation time allowed
        self.maxSimTime = maxtime
        self.touchThreshold = t_threshold

        # Frecuencia Hz con la que le robot debe tomar una decisión
        rate = rospy.Rate(40)
        self.stop_driving = False
        self.linear_ = self.angular_ = 0

        srv = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)
        srv()

        # reset states of robot
        simulationState = resetState()

        self.stuckCounter = 0
        self.closestHitDistance = 100
        self.newTau = self.tauInicial

        iter = 0
        actuatorValues = [0,0]
        history_sensor_values = []
        
        while not rospy.is_shutdown() and self.showMustGoOn() and not self.stop_driving:
        
            self.is_driving = True
            
            # calculate the output to control the neurocontroller
            actuatorValues = driver.driveArlo([self.sensorValues])
            history_sensor_values.append(self.sensorValues) 
      
            if iter % 10 == 0:
                self.control_sound("motor_play")

            self.linear_ = actuatorValues[0]
            self.angular_ = actuatorValues[1]
    
            self.values_to_neurocontroller(self.a_scale_ * self.angular_, self.l_scale_ * self.linear_)

            rate.sleep()
            # print(self.a_scale_ * self.angular_, self.l_scale_ * self.linear_)
        
        self.is_driving = False
        self.stop_driving = False

        # dir of values
        
        # save sensor values
        obj_dir = f"{problem_dir}/obj_space_gen_{str_num}.out"
        f = open(obj_dir, "a")

        str_obj_space = " ".join(res_obj_space.astype(str))
        f.write(f"{str_obj_space}\n")
        f.close()

        # calculate the state of robot
        self.computeSimStats()

        # low velocity to stop robot
        self.values_to_neurocontroller(0, actuatorValues[0]*0.5)
        t = rospy.Duration(1)
        rospy.sleep(t)

        # stop robot
        self.values_to_neurocontroller(0,0)

        # stop stimulations
        self.control_sound("stop_effects")
        self.simulation_color("hide")

        srv = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)
        srv()
        

    def computeSimStats(self):
        global simulationState

        simulationState["finishTime"] = simulationState["currentTime"]
        simulationState["crashingRisk"] = 1/(self.closestHitDistance + 1)

        if simulationState["targetReached"] == True:
            simulationState["robotEnergy"] = 100

            self.simulation_color("green");
            # success sound effect when completing the task
            self.control_sound("sucess")
        elif simulationState["timeRunOut"] == True:
            print((f"--- computeSimStats()  timeRunOut == True"))
            self.simulation_color("red")
            self.control_sound("loss")

        if simulationState["stuck"] == True:
            self.simulation_color("red")
            self.control_sound("loss")

    def values_to_neurocontroller(self,angular_z, linear_x):
        twist = Twist()
        twist.angular.z = angular_z
        twist.linear.x = linear_x
        self.vel_pub_.publish(twist)

    def showMustGoOn(self):
        global simulationState
        return simulationState["timeRunOut"] == False and simulationState["targetReached"] == False


    def evaluateDriver(self, req):
        global simulationState

        print("evaluateDriver+++++++++++++++++++++++")

        # load ANN
        self.driver.load_weights(req.weightsfile, req.num_ind)
        
        self.startSimulation(self.driver, req.maxtime, req.touchthreshold)

        return EvaluateDriverResponse(simulationState["distanceTravelled"],
                                      simulationState["distanceToGo"],
                                      simulationState["robotDamage"],
                                      simulationState["distanceTravelled"],
                                      simulationState["crashingRisk"],
                                      abs(simulationState["finalSpeed"]))

    def control_sound(self, name):
    
        try:
            play_song = rospy.ServiceProxy("play_song", PlaySong)
            play_song(name)
        except Exception as e:
            print(f"Error al reproducir el sonido:\n{e}")


    def control_robot(self, req):
        control = req.control

        if (control.find("stop") >= 0):
            print("Deteniendo robot")
            self.stop_driving = True


        if (control.find("pause") >= 0):
            # se detiene la simulación
            rospy.ServiceProxy("/gazebo/pause_physics", Empty)
            print("pause gazebo")
            # se pausa el sonido (incluyendo el de fondo)
            self.control_sound("pause_world")
            print("pause sound")


        if (control.find("unpause") >= 0):
            print("Conduciedo el robot")
            # se continua la simulación
            try:
                srvgz = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
                srvgz()
            except Exception as e:
                print(f"Error al detener la simulacion:\n{e}")

            # se reproduce el sonido (incluyendo el de fondo)
            if (self.is_driving):
                self.control_sound("unpause_world")
            else:
                self.control_sound("unpause_world_min_volume")
    

        return control_robotResponse(self.is_driving, self.maxSimTime)



    def get_model_position(self, model_name):
        rospy.wait_for_service("/gazebo/get_model_state")
        srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        res = srv(model_name, "")

        if res.status_message.find("not exist") >= 1:
            raise Exception(f"No se encuentra el modelo: {model_name}")
        
        return res.pose.position


    def checkModelPosition(self, msg):
        global simulationState

        if simulationState["targetReached"]:
            print(f"targetReached: {simulationState['targetReached']}")
            return
        
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

        # Calcular la velocidad promedio usando el promedio exponencial (darle más peso a valores recientes de velocidad) 
        self.newTau = self.alphaValue * msg.twist.twist.linear.x + (1 - self.alphaValue) * self.newTau
        simulationState["finalSpeed"] = self.newTau

        #  Obtener la posición actual de robot
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y

        # get position of the target
        pos = self.get_model_position("mesita_meta")

        # Compute differente between target and robot in each dimension (kinda distance)
        deltaX = pos.x - new_x
        deltaY = pos.y - new_y

        #  Angle in format (0, PI), (0,-PI)
        targetDirection = math.atan(abs(deltaY / deltaX))
        if (deltaX < 0 and deltaY > 0):
        # 2o cuadrante -,+z
            targetDirection = math.pi - targetDirection
        
        elif (deltaX < 0 and deltaY < 0):
        #  3er cuadrante -,-
            targetDirection = -math.pi + targetDirection
            
        elif (deltaX > 0 and deltaY < 0):
        #  4o cuadrante +,-
            targetDirection = -targetDirection
            
        self.deltaAngle = math.atan2(math.sin(targetDirection - yaw), math.cos(targetDirection - yaw))
        
        simulationState["position"][0] = new_x
        simulationState["position"][1] = new_y
        simulationState["distanceToGo"] = math.dist([pos.x, pos.y],[new_x, new_y])

        # # Como los mensajes de la posición actual están en una cola, es posible que al comenzar
        # # una nueva simulación, en la cola se hayan quedado valores de una posición cercana a
        # # la meta. De esta manera, es posible que al inicio de una simulación, de manera incorrecta
        # # se detecte que el robot llegó a la meta.
        # # Cuando esto pasa, el tiempo es casi cero, así que se puede usar esto para ignorar algún
        # # posible falso positivo. Si el tiempo es menor que 2 y se alcanzó el objeto, quiere decir que
        # # se está leyendo un valor de la cola y no la posición actual.
        if (simulationState["distanceToGo"] <= self.touchThreshold and simulationState["currentTime"] > 1):
            print(f"simulationState['currentTime']: {simulationState['currentTime']} > 1")
            simulationState["targetReached"] = True
            return;
        

        # /* Determinar si el robot ya se quedó atorado. */
        # /* Si la distancia que ha avanzado es menor que un umbral pequeño */
        distanceBefore = simulationState["distanceTravelled"]

        # # Compute the new distance travelled.
        simulationState["distanceTravelled"] += math.dist([self.prev_x, self.prev_y], [new_x, new_y])

        # # Check if the robot has moved or not.
        if (abs(distanceBefore - simulationState["distanceTravelled"]) < 0.001):
            self.stuckCounter+=1
            
            if (self.stuckCounter > 180):
                print((f"--- stuckCounter : {self.stuckCounter}"))
                simulationState["stuck"] = True
                simulationState["timeRunOut"] = True
            
        else:
            self.stuckCounter = 0

        # # Swap values for next iteration.
        self.prev_x = new_x
        self.prev_y = new_y

    def checkSimulationTime(self,msg):
        global simulationState

        simulationState["currentTime"] = msg.clock.to_sec();
        if (simulationState["currentTime"] >= self.maxSimTime):
                
            if(self.n_aux % 1600 == 0):
                print(f"++++ checkSimulationTime() , timeRunOut:{ simulationState['timeRunOut']} --- {simulationState['currentTime']} >= {self.maxSimTime}, self.n_aux:{self.n_aux}", )
            self.n_aux += 1
            simulationState["timeRunOut"] = True


    def checkSonarLeftValues(self,msg):
        global simulationState

        for i in range(len(msg.ranges)):
            if msg.ranges[i] <= 3.0:
                self.sensorValues[i + 0 * self.NUM_RAYS] = 3.0 - msg.ranges[i]
            else:
                self.sensorValues[i + 0 * self.NUM_RAYS] = 0.0

            if (msg.ranges[i] < self.closestHitDistance and simulationState["distanceToGo"] > 1):
                self.closestHitDistance = msg.ranges[i]

    def checkSonarCenterValues(self, msg):
        global simulationState

        for i in range(len(msg.ranges)):
            if msg.ranges[i] <= 3.0:
                self.sensorValues[i + 1 * self.NUM_RAYS] = 3.0 - msg.ranges[i]
            else:
                self.sensorValues[i + 1 * self.NUM_RAYS] = 0.0
             
            if (msg.ranges[i] < self.closestHitDistance and simulationState["distanceToGo"] > 1):
                self.closestHitDistance = msg.ranges[i]

    def checkSonarRightValues(self, msg):
        global simulationState

        for i in range(len(msg.ranges)):
            if msg.ranges[i] <= 3.0:
                self.sensorValues[i + 2 * self.NUM_RAYS] = 3.0 - msg.ranges[i]
            else:
                self.sensorValues[i + 2 * self.NUM_RAYS] = 0.0
             
            if (msg.ranges[i] < self.closestHitDistance and simulationState["distanceToGo"] > 1):
                self.closestHitDistance = msg.ranges[i]

        self.sensorValues[3 * self.NUM_RAYS + 0] = simulationState["distanceToGo"] - 1.4
        self.sensorValues[3 * self.NUM_RAYS + 1] = self.deltaAngle



