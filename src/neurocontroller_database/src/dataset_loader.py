#!/usr/bin/env python3

import sys
import os
from os.path import expanduser
home = expanduser("~")
sys.path.insert(1, f"{home}/catkin_ws/src/emotion_classification/models/arl-eegmodels")
sys.path.insert(2, f"{home}/catkin_ws/src/emotion_classification/src")
sys.path.insert(3, f"{home}/catkin_ws/src/neurocontroller_database/src")

from non_dominated import fast_non_dominated_sort
from utilities import load_json, print_error, print_color, has_attr, root_dir, root_database
from ROS import get_type_experiment

from brainflow.data_filter import DataFilter
from brainflow.board_shim import BoardShim, BrainFlowInputParams, LogLevels, BoardIds
import rospy
from eeg_signal_acquisition.msg import eeg_block
from neurocontroller_database.srv import load_evaluation_data, load_evaluation_dataResponse
from neurocontroller_database.srv import get_type_id, get_type_idResponse
from neurocontroller_database.srv import load_dataset, load_datasetResponse
from neurocontroller_database.srv import load_obj_space, load_obj_spaceResponse
from neurocontroller_database.srv import load_ind, load_indResponse
from neurocontroller_database.srv import id_to_path, id_to_pathResponse
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



def num_to_fileName(num_gen, is_obj_space=True):
    
    str_num = ""

    # num del archivo de generación
    if num_gen > 0 and num_gen < 10:
        str_num = f"00{num_gen}"
    if num_gen >= 10 and num_gen < 100:
        str_num = f"0{num_gen}"
    if num_gen >= 100:
        str_num = f"{num_gen}"

    if str_num == "":
        raise Exception(f"No existe el numero de generacion {num_gen}")
    else:
        if is_obj_space:
            str_num = f"obj_space_gen_{str_num}"
        else:
            str_num = f"var_space_gen_{str_num}"
            
        return str_num
    

def srv_id_to_filePath(req):
    paths =id_to_filePath(req.id_pareto_front, req.is_object_space, req.return_id, req.version)
    if len(paths.shape) > 1:
        p = paths[:,0]
        ids = paths[:,1]
    else:
        p = paths
        ids = []

    return id_to_pathResponse(p, ids)



def id_to_filePath(id_pareto, is_obj_space=True, return_id=False, version="v2"):
    # se separan los id (id compuesto)
    ids = id_pareto.split("_")

    path_list = []
    for id in ids:
        path_ = ""

        # si el id es de un usuario
        if id.find("/") >= 0:
            # ruta de inicio de la base de datos
            path_ = f"{root_database}/database"
            # se divide el ID
            s_id = id.split("/")
            # primer posición es el nombre del usuario
            path_ += f"/{s_id[0]}"
            # segunda posición es el ID de la población
            id_aux = s_id[1]
        else:
            path_ = f"{root_database}/database_populations"
            id_aux = id
            
        # se extrae el num gen y el nombre del folder
        s_id = id_aux.split("-")
        # pimer posición es el nombre del folder
        path_ += f"/{s_id[0]}"
        # segunda posición es el número de generación
        fileName = num_to_fileName(int(s_id[1]), is_obj_space)
        # ruta completa del archivo
        path_ += f"/{fileName}"

        # si son pesos de una ANN version en c++
        if version == "v1":
            path_ = f"{path_}.out"

        # weights of model version 2
        if version == "v2":
            if is_obj_space:
                path_ = f"{path_}.out"
            else:
                path_ = f"{path_}"
        
        if return_id:
            path_list.append([id, path_])
        else:
            path_list.append(path_)

    return np.array(path_list)


def check_id(id_pareto):
    if id_pareto == "" or id_pareto.find("-") < 0:
        raise Exception("ID Pareto no valido")
    
    
# servicio que carga el espacio de los objetivos
def load_object_space_(req):
    global root_dir

    id_pareto = req.id_pareto_front

    check_id(id_pareto)
    
    # si se normaliza el dataset
    if req.normalize:
        range_obj = load_json(f"{root_dir}/range_objectives.txt")
        if len(range_obj) == 0:
            print_error("No se ha creado el archivo de los rangos de cada objetivo")
            return False
        
        min_r = np.array(range_obj["min"][:3])
        max_r = np.array(range_obj["max"][:3])

    all_space = np.empty((0,3))
    all_num_ind = []
    all_id_pareto = []
    
    # dir de los archivos
    dat_inds = id_to_filePath(id_pareto, is_obj_space=True, return_id=True)
    for id_file in dat_inds:

        # se cargan los individuos del frente (pos 1 = a dir del archivo)
        data = np.array(pd.read_csv(id_file[1], delim_whitespace=True, header=None))
        data = data[:, :3]

        if req.normalize:
            data = (data - min_r) / (max_r - min_r)

        # se guardan los frentes 
        all_space = np.vstack((all_space, data))
        # num_ind de cada individuo
        all_num_ind = np.concatenate((all_num_ind, np.arange(data.shape[0])))
        # id de cada individuo (pos 1 = a id de la ruta del archivo)
        all_id_pareto = np.concatenate((all_id_pareto, [id_file[0]]*data.shape[0]))

    # errores con la carga
    if all_space.shape[0] == 0:
        raise Exception("No se encontraron soluciones")
    if all_space.shape[0] != all_num_ind.shape[0] or all_num_ind.shape[0] != all_id_pareto.shape[0]:
        raise Exception(f"Diferencia de tamaños con los datos de los individuos: all_space({all_space.shape[0]}), all_num_ind({all_num_ind.shape[0]}), all_id_pareto({all_id_pareto.shape[0]})")
    
    # formato para la red ROS
    all_num_ind = all_num_ind.astype(int)

    # poblaciones sin dominancia
    if not req.non_dominated:
        shape = all_space.shape
       
        print(f"num soluciones", all_space.shape)
        return load_obj_spaceResponse(all_space.reshape(shape[0]*shape[1]), shape[1], shape[0], all_id_pareto, all_num_ind)


    # dominancia de pareto
    rank_numbers = fast_non_dominated_sort(all_space)

    # soluciones requeridas
    if req.solutions_number == 0:
        size_req = 1
    else:
        size_req = req.solutions_number

    # datos del frente resultante
    sel_space = np.empty((0,3))
    sel_num_ind = []
    sel_id_pareto = []

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
        sel_space = np.vstack((sel_space, all_space[sel_inds]))
        sel_num_ind = np.concatenate((sel_num_ind, all_num_ind[sel_inds]))
        sel_id_pareto = np.concatenate((sel_id_pareto, all_id_pareto[sel_inds]))
        
        # soluciones faltantes
        size_all = sel_space.shape[0]

    print(f"{id_pareto} con {sel_space.shape} soluciones >= {size_req}")

    # formato
    sel_num_ind = sel_num_ind.astype(int)
    # espacio a cargar
    shape = sel_space.shape

    return load_obj_spaceResponse(sel_space.reshape(shape[0]*shape[1]), shape[1], shape[0], sel_id_pareto, sel_num_ind)
   

def load_var_space_(req):
    
    id_pareto = req.id_pareto_front
    num_ind = req.num_ind

    check_id(id_pareto)

    ind_path = id_to_filePath(id_pareto, is_obj_space=False, return_id=False)

    # solo se puede cargar un neurocontrolador a la vez (red neuronal)
    if len(ind_path) > 1:
        print_error(f"Solo se puede cargar un neurocontrolador, el ID es compuesto: {id_pareto}")
    
    ind_path = ind_path[0]
    print(ind_path)

    if ind_path.find(".out") >= 0:
        data = np.array(pd.read_csv(ind_path, delim_whitespace=True, header=None))
        data = data[num_ind]
        return load_indResponse(data, "")

    # if os.path.isdir(ind_path):
    #     ind_path += f"ind_{num_ind}.h5" 
    #     model = tf.keras.saving.load_model(ind_path)
    #     w = model.get_weights()
    #     w = json.dumps(w)

        return load_indResponse([], w) 


if "__main__" == __name__:
    rospy.init_node("dataset_loader")

    # load_train_srv = rospy.Service("load_evaluation_data", load_evaluation_data, train_intersubject)
    # load_srv = rospy.Service("load_dataset", load_dataset, load_ds)

    load_obj_srv = rospy.Service("load_object_space", load_obj_space, load_object_space_)
    load_var_srv = rospy.Service("load_var_space", load_ind, load_var_space_)
    id_to_p = rospy.Service("id_to_path", id_to_path, srv_id_to_filePath)
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

