#!/usr/bin/env python3

import sys
import os
from os.path import expanduser
home = expanduser("~")
sys.path.insert(1, f"{home}/catkin_ws/src/emotion_classification/models/arl-eegmodels")
sys.path.insert(2, f"{home}/catkin_ws/src/emotion_classification/src")
sys.path.insert(3, f"{home}/catkin_ws/src/neurocontroller_database/src")

from non_dominated import fast_non_dominated_sort
from utilities import load_json, print_error, print_color, has_attr, root_dir, bcolors
from ROS import get_type_experiment

from brainflow.data_filter import DataFilter
from brainflow.board_shim import BoardShim, BrainFlowInputParams, LogLevels, BoardIds
import rospy
from eeg_signal_acquisition.msg import eeg_block
from neurocontroller_database.srv import load_evaluation_data, load_evaluation_dataResponse
from neurocontroller_database.srv import get_type_id, get_type_idResponse
from neurocontroller_database.srv import load_dataset, load_datasetResponse
from neurocontroller_database.srv import load_obj_space, load_obj_spaceResponse
from eeg_signal_acquisition.srv import eeg_block_srv, eeg_block_srvResponse
from emotion_classification.srv import control_emotions
import numpy as np
import json
import pickle
import pandas as pd

# direccion raiz
root_dir = home+"/catkin_ws/src/neurocontroller_database/database"
data_experiment = {}
pub_eeg = None


# servicio que carga el espacio de los objetivos
def load_object_space_(req):
    global root_dir
    
    # id_front = req.id_pareto_front

    id_pareto = req.id_pareto_front

    # frente válido
    if id_pareto.find("optimized-") < 0:
        print("Frente inválido, no se puede cargar", id_pareto)
        return False
    
    # si se normaliza el dataset
    if req.normalize:
        range_obj = load_json(f"{root_dir}/range_objectives.txt")
        if len(range_obj) == 0:
            print_error("No se ha creado el archivo de los rangos de cada objetivo")
            return False
        
        min_r = np.array(range_obj["min"][:3])
        max_r = np.array(range_obj["max"][:3])

    # se extran los número del frente
    id_pareto = id_pareto.split("-")[1]
    # si es compuesto se extraen los frentes
    split_id = id_pareto.split(",")

    all_space = np.empty((0,3))
    all_id_pareto = []
    all_num_ind = []
    
    for num in split_id:
        # generación a cargar
        num = int(num)

        # num del archivo de generación
        if num < 10:
            str_num = f"00{num}"
        else:
            str_num = f"0{num}"

        if req.load_test:
            type_inds = "optimized_individuals"
        else:
            type_inds = "test_individuals"

        # archivo a cargar
        dir_ = f"{root_dir}_populations/{type_inds}/obj_space_gen_{str_num}.out"

        # se cargan los individuos del frente
        data = np.array(pd.read_csv(dir_, delim_whitespace=True, header=None))
        data = data[:, :3]

        if req.normalize:
            data = (data - min_r) / (max_r - min_r)

        # se guardan los frentes 
        all_space = np.vstack((all_space, data))
        # id de cada individuo
        all_id_pareto = np.concatenate((all_id_pareto, [f"optimized-{num}"]*data.shape[0]))
        # num_ind de cada individuo
        all_num_ind = np.concatenate((all_num_ind, np.arange(data.shape[0])))
    
    if all_space.shape[0] == 0:
        print_error("No se encontraron soluciones")
        return False

    # poblaciones sin dominancia
    if not req.non_dominated:
        shape = all_space.shape
        all_num_ind = all_num_ind.astype(int)
        print(f"num soluciones", all_space.shape)
        return load_obj_spaceResponse(all_space.reshape(shape[0]*shape[1]), shape[1], shape[0], [], all_num_ind)


    # dominancia de pareto
    rank_numbers = fast_non_dominated_sort(all_space)

    # soluciones requeridas
    if req.solutions_number == 0:
        size_req = 1
    else:
        size_req = req.solutions_number

    # datos del frente resultante
    sel_space = np.empty((0,3))
    sel_id_pareto = []
    sel_num_ind = []

    # lista con valores unicos del rank
    rank_list = np.unique(rank_numbers).tolist()
    # num de soluciones resultantes
    size_all = 0

    while(size_req > size_all):
        # nivel de rank 
        try:
            rank = rank_list[0]
            del rank_list[0]
            if rank <= 0:
                print_error(f"Rank inválido: {rank}")
                return False
            
        except Exception as e:
            break

        # soluciones pertenecientes al nivel de rank
        sel_inds = rank_numbers == rank

        # se extraen las soluciones del nivel de rank al que pertenecen
        sel_id_pareto = np.concatenate((sel_id_pareto, all_id_pareto[sel_inds]))
        sel_num_ind = np.concatenate((sel_num_ind, all_num_ind[sel_inds]))
        sel_space = np.vstack((sel_space, all_space[sel_inds]))

        # soluciones faltantes
        size_all = sel_space.shape[0]

    print(f"num soluciones", sel_space.shape,size_req)
    
    # individuos
    size_split = len(split_id)
    if size_split > 1:
        sel_num_ind = np.arange(sel_space.shape[0])

    # espacio a cargar
    shape = sel_space.shape

    sel_num_ind = sel_num_ind.astype(int)
    return load_obj_spaceResponse(sel_space.reshape(shape[0]*shape[1]), shape[1], shape[0], [], sel_num_ind)
   


if "__main__" == __name__:
    rospy.init_node("dataset_loader")

    # load_train_srv = rospy.Service("load_evaluation_data", load_evaluation_data, train_intersubject)
    # load_srv = rospy.Service("load_dataset", load_dataset, load_ds)

    load_obj_srv = rospy.Service("load_object_space", load_obj_space, load_object_space_)
    
    # topic_prepro = rospy.Subscriber("preprocessing_eeg_signals", eeg_block, train_intersubject)
    print("Nodo dataset_loader iniciado con éxito");
    rospy.spin()





# def get_sample_rate(id_signal):
#     # taza de muestreo
#     sr = 0
#     if id_signal.find("cyton") >= 0:
#         sr = 125
#     if id_signal.find("simulation") >= 0:
#         sr = 125
#     if id_signal.find("synthetic") >= 0:
#         sr = 250

#     return sr

# # carga un archivo con las señales eeg y las envía en la red ROS
# def load_ds_srv(user_name, id_experiment, num_ind, is_start, is_stop, storage_pos=-1, type_signal="experiment_data"):
#     global pub_eeg, labels_deap

#     print("------------------")
#     print_color(f"user_name:{user_name} -- id_experiment:{id_experiment} -- num_ind:{num_ind} is_start:{is_start} -- is_stop:{is_stop} -- storage_pos:{storage_pos} -- type_signal:{type_signal}", bcolors.OKGREEN)
#     # ruta de los datos
#     type_id = get_type_experiment(id_experiment)
#     if type_id == "":
#         return False
#     # dir bases
#     dir_ = f"{root_dir}/{user_name}/{type_id}"

#     # señales del protocolo de adquisicion
#     if id_experiment.find("protocol") >= 0:
#         print("load: protocol")

#         # ruta de la info del último experimento
#         dir_last = f"{dir_}/history/last_experiment.txt"
#         # se carga el ultimo experimento
#         last_obj = load_json(dir_last)

#         # tipo de carga de historial
#         if id_experiment.find("basal") >= 0:
#             type_histo = "history_basal"
#         else:
#             if id_experiment.find("video") >= 0:
#                 type_histo = "history_emotion_evaluation"
#             else:
#                 print_error("Error con el protocolo, no se encuentra el experimento", id_experiment)
#                 return False

#         # nombre del archivo del experimento a usar
#         name_video_exp = last_obj[type_histo]

#         # ruta de los datos emocionales del protocolo
#         path_v = f"{dir_}/history/{name_video_exp}"
#         # se ca rgan las evaluaciones
#         emo_data = load_json(path_v)

#         # se carga la info del número de video
#         eval_exp = emo_data[num_ind]

#         # emo evaluations del video
#         if has_attr(eval_exp, "valence") and has_attr(eval_exp, "arousal"):
#             valence = float(eval_exp["valence"])
#             arousal = float(eval_exp["arousal"])
#             emotion_evaluation = [valence, arousal]
#         else:
#             emotion_evaluation = []

#         # número de video
#         num_video = eval_exp["num_video"]
#         # id de las señales
#         if not has_attr(eval_exp, "id_eeg_signal"):
#             print_error("El experimento no cuenta con id_eeg_signal")
#             return False

#         id_signal = eval_exp["id_eeg_signal"]
        
#         # tasa de muestreo
#         sr = get_sample_rate(id_signal)

#         # ruta del archivo
#         path_eeg = f"{dir_}/{id_experiment}/ind_{num_video}/{id_signal}.csv"
#         # se verifica que exista el archivo de las señales
#         if not os.path.isfile(path_eeg):
#             print_error(f"No existe el archivo {path_eeg}")
#             return False

#         # se leen la señales EEG
#         data_eeg = DataFilter.read_file(path_eeg)
#         data_eeg = data_eeg[1:17, :]

#         # 1d to 2d para enviar la señal por un topico
#         shape = data_eeg.shape
#         data_eeg = np.array(data_eeg).reshape(shape[0] * shape[1])

#         rospy.wait_for_service("preprocessing_signal")
#         try:
#             srv_preprocessing = rospy.ServiceProxy("preprocessing_signal", eeg_block_srv)
            
#             srv_preprocessing(emotion_evaluation, data_eeg, shape[0], shape[1], user_name,
#                               id_experiment, num_ind, id_signal, storage_pos, 0, is_start, is_stop, sr, type_signal)
#             return True
#         except rospy.ServiceException as e:
#             print_error(f"Error al procesar las señales\n\n{e}")
#             return False

#     # señales de neurocontroladoreres optimizados
#     if id_experiment.find("optimized") >= 0:
#         print("load optimized")

#         path_emo_data = f"{root_dir}/{user_name}/history/adquisition_data.out"
#         emo_data = load_json(path_emo_data)
#         # id den individuo
#         id_ind = f"ind_{num_ind}"

#         # verificación de errores en la localización del experimento
#         err = f"No hay datos del experimento"
#         if len(emo_data.keys()) == 0:
#             print_error(err)
#             return False
#         if not has_attr(emo_data, id_experiment):
#             print_error(f"{err} {id_experiment}")
#             return False
#         if not has_attr(emo_data[id_experiment], id_ind):
#             print_error(f"{err}, {id_experiment}, {id_ind}")
#             return False

#         if storage_pos >= 0:
#             if not has_attr(emo_data[id_experiment][id_ind], storage_pos):
#                 print_error(f"{err}, {id_experiment}, {id_ind}, storage_pos: {storage_pos}")
#                 return False

#             emo_eval = emo_data[id_experiment][id_ind][storage_pos]
#         else:
#             if len(emo_data[id_experiment][id_ind]) == 0:
#                 print_error(f"{err}, {id_experiment}, {id_ind}, storage_pos: {storage_pos} No hay datos")
#                 return False

#             l = len(emo_data[id_experiment][id_ind]) - 1
#             emo_eval = emo_data[id_experiment][id_ind][l]


#         # etiquetas emocionales
#         if has_attr(emo_eval, "emotion_evaluation"):
#             aux = emo_eval["emotion_evaluation"]
#             label = [float(aux["valence"]), float(aux["arousal"])]
#         else:
#             label = []

#         # id de las señales
#         id_signal = emo_eval["id_eeg_signal"]

#         # tasa de muestreo
#         sr = get_sample_rate(id_signal)

#         # dir de las señales
#         path_eeg = f"{dir_}/{id_experiment}/{id_ind}/{id_signal}.csv"
#         # se leen la señales EEG
#         data_eeg = DataFilter.read_file(path_eeg)
#         data_eeg = data_eeg[1:17, :]

#         # 1d to 2d para enviar la señal por un topico
#         shape = data_eeg.shape
#         print("Datos: ", shape)
#         data_eeg = np.array(data_eeg).reshape(shape[0] * shape[1])

#         try:
#             rospy.wait_for_service("preprocessing_signal")

#             srv_preprocessing = rospy.ServiceProxy("preprocessing_signal", eeg_block_srv)
#             srv_preprocessing(label, data_eeg, shape[0], shape[1], user_name,
#                               id_experiment, num_ind, id_signal, storage_pos,0, is_start, is_stop, sr, type_signal)

#             return True
#         except rospy.ServiceException as e:
#             print_error("Error al procesar las señales")
#             return False

#     if id_experiment.find("deap") >= 0:
#         print("load DEAP")

#         path_eeg = f"{root_dir}/deap/{user_name}.dat"
#         # existe el archivo del usuario
#         if not os.path.isfile(path_eeg):
#             print_error(f"El usuario {user_name} no cuenta con su archivo en {path_eeg}")
#             return False
        
#         # carga de los datos eeg
#         data_eeg = pickle.load(open(path_eeg, 'rb'), encoding='latin1')

#         # datos de los experimentos
#         if num_ind != None and num_ind >= 0:
#             if num_ind >= data_eeg["labels"].shape[0]:
#                 print_error(f"DEAP: Error al cargar el experimento {num_ind}")
#                 return False

#             labels = [data_eeg["labels"][num_ind]]
#             data_eeg = [data_eeg["data"][num_ind, :16, :]]
#             num_exp = 1
#         else:
#             labels = data_eeg["labels"]
#             data_eeg = data_eeg["data"][:, :16, :]
#             num_exp = labels.shape[0]
        
#         print("Número de experimentos a enviar", len(data_eeg))
#         # taza de muestreo
#         sr = 128

#         # se dividen los experimentos
#         for n_exp in range(num_exp):
#             shape = data_eeg[n_exp].shape
#             ini = False
#             end = False
#             print(f"Enviando experimento {n_exp+1} con forma", shape)

#             if n_exp == 0:
#                 ini = True

#             if n_exp == num_exp - 1:
#                 end = True

#             try:
#                 rospy.wait_for_service("run_evaluations_model")

#                 srv_model_sel = rospy.ServiceProxy(
#                     "run_evaluations_model", eeg_block_srv)
#                 srv_model_sel(labels[n_exp][:2], data_eeg[n_exp].reshape(shape[0]*shape[1]), shape[0], shape[1], user_name,
#                               "deap", n_exp+1, f"s{n_exp:02d}", -1, 0, is_start, is_stop, sr, type_signal)

#             # preprocessing_eeg_signals_train
#             except rospy.ServiceException as e:
#                 print(
#                     "Error al cargar las señales de DEAP a la selección de modelos\n", e)
#                 return False

#         print(f"Experimentos enviados del usuario {user_name}")
#         return True
    
#     print("No se encuentra el experimento user_name:", user_name, "id_experiment:",id_experiment, "num_ind:",num_ind)
#     return False


# # servicio que carga los valores de una prueba intersubject
# def train_intersubject(req):
#     user_name = req.user_name
#     type_train = req.type_train

#     if user_name == "":
#         print("Error: no hay nombre de usuario")
#         return False

#     if type_train.find("optimized") >= 0:
#         # ruta de los datos
#         path_emo_data = f"{root_dir}/{user_name}/history/adquisition_data.out"
#         # carga de los datos de los experimentos
#         emo_data = load_json(path_emo_data)
#         if len(emo_data.keys()) == 0:
#             print("Error al cargar los datos de los experimentos con el neurocontrolador")
#             return False
    
#         # SE CARGAN LOS DATOS DEL ENTRENAMIENTO, SE ENVIAN LOS EXPERIMENTOS DE MANERA INDIVIDUAL
#         # FALTA DE IMPLEMENTACION
#         print("implementar carga de datos")


#     if type_train.find("protocol") >= 0:
#         if user_name == "DABB":
#             num_videos = 15
#         else:
#             num_videos = 26

#         for n_ind in range(num_videos):
#             is_start = False
#             is_stop = False

#             if n_ind == 0:
#                 is_start = True
#             if n_ind == num_videos - 1:
#                 is_stop = True

#             res = load_ds_srv(user_name, "protocol_video", n_ind, is_start, is_stop, type_signal="train")
#             # if not res:
#             #     return False


#     if type_train.find("deap") >= 0:
#         # se envian los datos de los 32 usuarios
#         num_users = 32
#         ini_user = 14
#         for n_user in range(ini_user, num_users):
#             n_user += 1
#             is_start = False
#             is_stop = False

#             if n_user == num_users-1:
#                 is_stop = True
#             if n_user == ini_user:
#                 is_start = True

#             # nombre de usuario
#             user_name = f"deap_s{n_user:02d}"

#             # se cargan los datos
#             res = load_ds_srv(user_name, "deap", n_user, is_start, is_stop, type_signal="train")
#             if not res:
#                 return False
        

#     print("+++++ Éxito al cargar : ", type_train, "+++++++")
#     return load_evaluation_dataResponse()


# # servicio que carga un experimento de manera separada
# def load_ds(req):
#     print(req)
#     val = load_ds_srv(req.user_name, req.id_experiment, req.num_ind, req.is_start, req.is_stop, req.storage_pos, req.type_signal)

#     if not val:
#         return False
#     else:
#         return load_datasetResponse()

