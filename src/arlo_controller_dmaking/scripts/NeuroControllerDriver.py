# import torch
# import torch.nn as nn
# import torch.functional as F

from os.path import expanduser
import sys, os
home = expanduser("~")
sys.path.insert(2, f"{home}/catkin_ws/src/neurocontroller_database/src")
import rospy
import numpy as np
import tensorflow as tf
import json
from neurocontroller_database.srv import load_ind
from ROS import id_to_filePath
from keras.utils.layer_utils import count_params

NUM_RAYS = 16
NUM_ACTUATORS = 2 # Velocity linear and angular
NUM_SONARS = 3

factor = tf.convert_to_tensor([0.0095, 0.1])


class NeuroControllerDriver():
    model = None
    oBounds = None
    ini_bound = None
    frac_bound = None
    version = ""
    count = 0
    # maxInputValue = -np.infty
    # minInputValue = np.infty

    def __init__(self, version = "v2"):
        self.oBounds = np.array([[-0.8, 1.5],[-0.35, 0.35]])
        self.ini_bound = self.oBounds[:,0]
        self.end_bound = self.oBounds[:,1]
        self.frac_bound = self.oBounds[:,1] - self.oBounds[:,0]
        self.version = version

        with tf.device('/cpu:0'):
            self.model = tf.keras.Sequential()
            input = tf.keras.Input(self.getNumInputs())

            if version == "v1":
                l1 = tf.keras.layers.Dense(NUM_ACTUATORS, use_bias=False)(input)
                af = tf.keras.layers.Activation(self.custom_activation_funtion)(l1)
                # af = tf.keras.layers.Activation("sigmoid")(l1)
                self.model = tf.keras.Model(input, af)
                
            else:
                self.model.add(input)
                self.model.add(tf.keras.layers.Dense(16, input_dim=20, activation="relu"))
                self.model.add(tf.keras.layers.Dense(12, activation="relu"))
                self.model.add(tf.keras.layers.Dense(2, activation="sigmoid"))
                self.model.add(tf.keras.layers.Activation(self.custom_activation_funtion))
                
            self.model.compile(optimizer="adam", loss="mse")
        print("Modelo creado")

    def get_num_params(self):
        return count_params(self.model.trainable_weights)
    

    def map_val(self, val, ini_origin_range, end_origin_range, ini_new_range, end_new_range):
        dif_new = end_new_range - ini_new_range

        # tamaño rango inicial
        dif_origin = end_origin_range - ini_origin_range
        # posición del valor en el rango
        pos_origin = val - ini_origin_range

        # mapeo
        # sandwich rule. dif_origin/pos_origin : dif_new/x, where x is the target
        conv = (pos_origin * dif_new) / dif_origin
        conv += ini_new_range
        return conv

    def custom_activation_funtion(self,x):
        global factor
    
        if x.shape[0] is  None:
            if self.count % 30 == 0 and self.count > 0:
                return None
            self.count += 1

            return np.zeros((1,self.getNumInputs()))
        
        # return self.ini_bound + self.frac_bound * (x * factor)
        return self.map_val(x, np.zeros(2), np.ones(2), self.ini_bound, self.end_bound)
    
    def get_path(self, id_pareto):
        id_dir = id_to_filePath(id_pareto)
        paths = id_dir.path

        if len(paths) != 1:
            raise Exception(f"ID invalido: {id_pareto}")
        
        return paths[0]
        
    def save_model(self, id_pareto, num_ind):
        
        dir_ind = self.get_path(id_pareto)
        if not os.path.isdir(dir_ind):
            os.makedirs(dir_ind)

        self.model.save_weights(f"{dir_ind}/ind_{num_ind}.h5")


    def load_weights(self, id_pareto, num_ind):
        try:
            

            if self.version == "v1":
                srv = rospy.ServiceProxy("load_var_space", load_ind)
                res = srv(id_pareto, num_ind)

                # format and structure of weight matrix
                num_input = self.getNumInputs()
                w = tf.convert_to_tensor(res.var_space)
                w = tf.reshape(w, [50,2])
                w = tf.expand_dims(w, axis=0)

                # set weigths
                self.model.set_weights(w)

            elif self.version == "v2":
                # w = json.loads(res.json_var_space)
                # self.model.set_weights(w)
                dir_ind = self.get_path(id_pareto)
                self.model.load_weights(f"{dir_ind}/ind_{num_ind}.h5")
            else:
               raise Exception(f"No se pudo cargar los pesos, version no valida={self.version}")

        except Exception as e:
            print(f"Error al cargar el neurocontrolador:\n{e}")


    def driveArlo(self, x):
        
        x = np.array(x, dtype=float)
        x = np.nan_to_num(x, nan=4,posinf=4)
        # x[np.where(x == 0)] = 0.0000001
        
        x = tf.convert_to_tensor(x)
        pred = self.model.predict_on_batch(x)

        return pred[0]
    

    def getNumInputs(self):
        return (NUM_RAYS * NUM_SONARS)+2
    


    