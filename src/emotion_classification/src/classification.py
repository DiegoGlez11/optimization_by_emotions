#!/usr/bin/env python3

import os
from os.path import expanduser
home = expanduser("~")
sep = os.sep
import sys
sys.path.insert(1, f"{home}{sep}catkin_ws{sep}src{sep}emotion_classification{sep}models{sep}arl-eegmodels")
sys.path.insert(3, f"{home}{sep}catkin_ws{sep}src{sep}emotion_classification{sep}src")
sys.path.insert(2, f"{home}{sep}catkin_ws{sep}src{sep}neurocontroller_database{sep}src")
from EEGModels import EEGNet
from EEGModels2 import DeepConvNet
from EEGModels import ShallowConvNet
from emotion_classification.msg import monitor as monitor_cnn
from emotion_classification.msg import emotional_prediction
from eeg_signal_acquisition.msg import eeg_block
# from emotion_classification.srv import get_num_segments, get_num_segmentsResponse
from emotion_classification.srv import get_emotion_data, get_emotion_dataResponse
from emotion_classification.srv import get_instancies_model_name, get_instancies_model_nameResponse
from emotion_classification.srv import load_emotion_model, load_emotion_modelResponse
from emotion_classification.srv import create_emotion_model, create_emotion_modelResponse
from emotion_classification.srv import control_emotions, control_emotionsResponse
from emotion_classification.msg import complete_predict
from experiment_control_interfaces.srv import show_predictions
import rospy
import numpy as np
from tensorflow.keras.models import load_model
from tensorflow.keras import utils as np_utils
import tensorflow as tf
import json, random
from evaluations import get_evaluations, get_emotions
from utilities import load_json, save_json, has_attr
from utilities import print_error, print_color, bcolors, get_type_experiment, create_id
from processing import processing_signal, set_metadata
from brainflow.data_filter import DataFilter
import tensorflow.keras.backend as K

root_dir = home+"/catkin_ws/src/neurocontroller_database/database"


# se desactiva la GPU
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

model_name = ""

window_size = 128
stride = window_size
# número de clases de emociones
num_classes = 4
# umbral para dividir las evaluaciones emocionales en alto y bajo
threshold = 0.5
# dirección raiz
# root_dir = home+"/catkin_ws/src/neurocontroller_database/database"
# topico ros del monitor
topic_monitor = None
topic_emo_pred = None
# identificador del modelo
id_model = ""
# Datos que no alcanzaron a pasar por el ventaneo
# se almacenan para concatenerlos con el siguiente bloque EEG
# y continuar con el desplazamiento de la ventana sobre la señal
data_rest = np.empty((0, num_classes))


models = ["EEGNet", "DeepConvNet", "ShallowConvNet", "DeepForest"]

dir_model = ""
model = None
umbral_emo = 5
count = 0

# modo de captura de las emociones
EMOTIONS_CAPTURE_MODE = ""


media = None
sigma = None

def update_params():
    global window_size, stride, model_name, EMOTIONS_CAPTURE_MODE

    params = load_json(f"{root_dir}/train_parameters.txt")
    print(params)

    stride = params["stride"]
    window_size = params["window_size"]
    model_name = params["model_name"]
    
    params = load_json(f"{root_dir}/control_params.txt")
    print(params)
    EMOTIONS_CAPTURE_MODE = params["emotions_capture_mode"]

def create_emo_model(req):
    global model_name, id_model
    global model, num_classes, threshold

    model_name = req.model_name
    user_name = req.user_name
    num_classes = req.num_classes
    dropout = req.dropout
    kernel_size = req.kernel_size
    F1 = req.F1
    D = req.D
    F2 = req.F2

    # gpu_devices = tf.config.experimental.list_physical_devices('GPU')
    # for device in gpu_devices:
    #     tf.config.experimental.set_memory_growth(device, True)

    # dirección de los modelos del usuario
    dir_models = f"{root_dir}/{req.user_name}/emotion_model"
    if not os.path.isdir(dir_models):
        os.makedirs(dir_models)
    # dirección de los modelos de cnn especificas
    dir_model_cnn = dir_models+"/"+model_name
    if not os.path.isdir(dir_model_cnn):
        os.makedirs(dir_model_cnn)

    # asignación de id
    dirs = os.listdir(dir_model_cnn)
    if len(dirs) == 0:
        id = "model_0"
    else:
        max = int(dirs[0].split("_")[1])
        for i in range(1, len(dirs)):
            num = int(dirs[i].split("_")[1])
            if (max <= num):
                max = num
        id = f"model_{max+1}"

    # dir del modelo
    dir_model = dir_model_cnn+"/"+id

    # se guardan los parámetros
    num_classes = req.num_classes
    # ID del modelo
    id_model = id

    param = {}
    param["model_name"] = model_name
    param["user_name"] = user_name
    param["dropout"] = dropout

    param["num_classes"] = req.num_classes
    param["num_channels"] = req.num_channels
    param["num_samples"] = req.num_samples

    if (model_name == "EEGNet"):
        param["kernel_size"] = req.kernel_size
        param["F1"] = req.F1
        param["D"] = req.D
        param["F2"] = req.F2

    # se guardan los parametros
    create_folder(dir_model)
    f = open(dir_model+"/model_param.txt", "w")
    f.write(json.dumps(param))
    f.close()
    
    # selección del modelo a usar
    if model_name == "EEGNet":
        model = EEGNet(nb_classes=req.num_classes, Chans=req.num_channels, Samples=req.num_samples, dropoutRate=dropout,
                       kernLength=kernel_size, F1=F1, D=D, F2=F2, dropoutType=req.type_dropout)

    if model_name == "DeepConvNet":
        model = DeepConvNet(nb_classes=req.num_classes, Chans=req.num_channels,
                            Samples=req.num_samples, dropoutRate=dropout)

    if model_name == "ShallowConvNet":
        model = ShallowConvNet(nb_classes=req.num_classes, Chans=req.num_channels,
                               Samples=req.num_samples, dropoutRate=dropout)

    # compilación del modelo con sus optimizadores
    model.compile(loss='categorical_crossentropy', optimizer='adam',
                  metrics=['accuracy', "Recall", "Precision"])

    # guardado del modelo
    model.save(f"{dir_model}/{model_name}.h5")

    # respuesta del servicio
    print(f"\n\n +++++++++++ MODELO {model_name} CREADO CON EXITO, ID: {id} +++++++++++")

    return create_emotion_modelResponse(id_model)


def load_model_emo(req):
    global model_name, id_model, model, dir_model, root_dir, media, sigma
    print(f"-------- Cargando el modelo {model_name} --------")

    # dirección de los modelos del usuario
    dir_m = f"{root_dir}/{req.user_name}/emotion_model/{req.model_name}/{req.id_emotion_model}"
    if req.repetition_k >= 0:
        dir_m = f"{dir_m}/k_{req.repetition_k}_fold_{req.num_fold}"

    # se busca si existe el directorio del modelo
    if not os.path.isdir(dir_m):
        print_error(f"++++++ No se encuentra la dirección del modelo {dir_m} ++++++")
        return False

    # se busca si existe el directorio del modelo
    dir_sel_model = f"{dir_m}/{req.model_name}.h5"
    if not os.path.isfile(dir_sel_model):
        print_error(f"++++++ No se encuentra el modelo {req.model_name}.h5 ++++++")
        return False

    print(f"Cargando modelo {dir_sel_model}")

    K.clear_session()
    # se carga el modelo
    with tf.device("/device:CPU:0"):
        model = load_model(dir_sel_model)

    # se actualizan las variables
    dir_model = dir_sel_model
    model_name = req.model_name
    id_model = req.id_emotion_model

    # se carga la desviación estandar y media
    estandar = load_json(f"{dir_m}/param_standardization.txt")
    media = np.expand_dims(estandar["mean"], axis=0).transpose(1,0) 
    sigma = np.expand_dims(estandar["std"], axis=0).transpose(1,0) 

    print("estandardization: ",media.shape, sigma.shape)
    # respuesta del servicio
    print(f"\n\n +++++++++++ MODELO {model_name} con ID {id_model} cargado con éxito +++++++++++")

    return load_emotion_modelResponse()


'''
Verifica que exista una dirección y si no existe la crea
'''
def create_folder(dir_):
    if not os.path.isdir(dir_):
        os.makedirs(dir_)



def control_emo(req):
    global window_size, stride, EMOTIONS_CAPTURE_MODE

    # respuesta
    res_srv = control_emotionsResponse()

    if req.window_size > 0:
        print("\n\n--- Set window_size:", req.window_size)
        window_size = req.window_size
        res_srv.window_size = window_size

    if req.stride > 0:
        print("\n\n--- Set stride:", req.stride)
        stride = req.stride
        res_srv.stride = stride

    if req.control == "sam" or req.control == "bci" or req.control == "traditional":
        print("\n\n--- Set mode:", req.control)
        EMOTIONS_CAPTURE_MODE = req.control

    return res_srv


def predict_from_topic(msg):
    global num_classes, topic_emo_pred

    user_name = msg.user_name
    id_pareto = msg.id_pareto_front
    num_ind = msg.num_ind
    storage_pos = msg.storage_pos
    id_eeg_signal = msg.id_eeg_signal

    # el nodo puede enviar las emociones al modulo de preferencias
    # si está activo el modo de BCI
    if EMOTIONS_CAPTURE_MODE != "bci":
        print_error(f"No se puede enviar las emociones. MODO ACTIVO:{EMOTIONS_CAPTURE_MODE}")
        return
    
    if id_pareto.find("basal") >= 0:
        return
    if model_name == "" or model is None or media is None or sigma is None:
        print_error(f"Faltan parámetros para iniciar la predicción")
        print_error(f"model_name:{model_name} media:{media} sigma:{sigma}")
        return

    print(f"\n\n---- Predicciones con {model_name} ---")

    # dir de los evaluaciones
    type_id = get_type_experiment(id_pareto)
    dir_ = f"{root_dir}/{user_name}/{type_id}/{id_pareto}/ind_{num_ind}"
    # si la dir no existe
    create_folder(dir_)

    # # convertimos el array 1D a 2D con forma (numero_canales, numero_muestras)
    # eeg_data = np.array(msg.eeg_data).reshape(msg.num_channels, msg.num_samples)

    print("id_pareto:",id_pareto,"num_ind:",num_ind,"storage_pos:",storage_pos, "id_eeg_signal:",id_eeg_signal)
    print(f"{dir_}/{id_eeg_signal}_preprocessing.csv\n")
    # se leen las señales EEG
    eeg_data = DataFilter.read_file(f"{dir_}/{id_eeg_signal}_preprocessing.csv")
    eeg_data = eeg_data[1:17]

    # se extrae el estado basal y se realiza el ventaneo
    set_metadata(user_name, id_pareto, num_ind, id_eeg_signal, msg.type_signal)
    eeg_data = processing_signal(eeg_data, window_size, stride)
    
    print("Windows:", eeg_data.shape)

    # (ventana, canal, muestra) a (canal, ventana, muestra)
    eeg_data = eeg_data.transpose(1,0,2)

    # # ESTANDARIZACIÓN DE LA SEÑAL
    for n_chan in range(eeg_data.shape[0]):
        eeg_data[n_chan] = (eeg_data[n_chan] - media[n_chan])/sigma[n_chan]

    eeg_data = eeg_data.transpose(1,0,2)

    # PREDICCIÓN
    y_predict = model.predict_on_batch(eeg_data)

    # centroide de las predicciones
    valence = np.sum(y_predict[:, 0])/y_predict.shape[0]
    arousal = np.sum(y_predict[:, 1])/y_predict.shape[0]
    y_pred = [valence, arousal]

    pred_points = {"predictions": y_predict.tolist(), "centroide": y_pred}

    # historial de predicciones
    path_pred = f"{root_dir}/{user_name}/history/prediction_data.txt"
    pred_data = load_json(path_pred)

    # si no existe el registro
    if not has_attr(pred_data, id_pareto):
        pred_data[id_pareto] = {}

    id_ind = f"ind_{num_ind}"
    if not has_attr(pred_data[id_pareto], id_ind):
        pred_data[id_pareto][id_ind] = {}

    # si existe un registro 
    if not has_attr(pred_data[id_pareto][id_ind], str(storage_pos)):
        # id del archivo
        id_preds = create_id("predictions", dir_)
    else:
        id_preds = pred_data[id_pareto][id_ind][str(storage_pos)]
    
    id_preds = id_preds.split(".txt")[0]
    print("ID Prediction:", id_preds)

    # se guardan las predicciones
    save_json(pred_points, f"{dir_}/{id_preds}.txt")

    # se actualiza el historial de predicciones
    pred_data[id_pareto][id_ind][str(storage_pos)] = f"{id_preds}"
    save_json(pred_data, path_pred)

    # predicciones en la gráfica
    try:
        shape_pred = y_predict.shape[1]
        num_preds = y_predict.shape[0]

        srv = rospy.ServiceProxy("show_predictions", show_predictions)
        srv(user_name, id_pareto, num_ind, storage_pos, f"{id_pareto}/{id_ind}", [], y_pred, [], shape_pred, num_preds)
    except rospy.ServiceException as e:
        print("Error al plotear las predicciones\n", e)


    # señal de termino del experimento
    pub_pred = rospy.Publisher('complete_predict', complete_predict, queue_size=1)
    msg_end_pred = complete_predict()
    msg_end_pred.complete = True
    pub_pred.publish(msg_end_pred)


    # se mandan las prediciciones al módulo de preferencias
    msg_pred = emotional_prediction()
    msg_pred.user_name = user_name
    msg_pred.id_experiment = id_pareto
    msg_pred.num_ind = num_ind
    msg_pred.emotion_value = y_pred
    msg_pred.storage_pos = storage_pos

    topic_emo_pred.publish(msg_pred)

    
    

def get_model_data(req):
    global model_name, id_model

    res = get_emotion_dataResponse()

    if model is None:
        res.is_loaded = False
    else:
        res.is_loaded = True

    res.model_name = model_name
    res.id_emotion_model = id_model

    return res

# devuelve los id de las instancias de un modelo en especifico


def get_model_instancies(req):
    global models
    dir_emo = f"{root_dir}/{req.user_name}/emotion_model/{req.model_name}"

    res_inst = get_instancies_model_nameResponse()
    # se verifica is existe la ruta
    if (not os.path.isdir(dir_emo)):
        return res_inst

    # obtenemos los id de las diferentes instancias del modelo
    ids = os.listdir(dir_emo)

    res_inst.ids_model_name = ids
    return res_inst


def init_server():
    global topic_monitor, topic_emo_pred, root_dir, window_size, stride
    create_folder(root_dir)

    rospy.init_node("emotion_classification")
    print("Nodo iniciado: emotion_clasification")

    srv_control = rospy.Service("control_emotions", control_emotions, control_emo)
    srv_new_model = rospy.Service("create_emotion_model", create_emotion_model, create_emo_model)
    srv_load_model = rospy.Service("load_emotion_model", load_emotion_model, load_model_emo)
    srv_emo_data = rospy.Service("get_emotion_data", get_emotion_data, get_model_data)
    srv_get_ids = rospy.Service("get_instancies_model_name", get_instancies_model_name, get_model_instancies)
    subs_predict = rospy.Subscriber("preprocessing_eeg_signals", eeg_block, predict_from_topic)

    rospy.Publisher('complete_predict', complete_predict, queue_size=2)
    topic_monitor = rospy.Publisher("/monitor", monitor_cnn, queue_size=100)
    topic_emo_pred = rospy.Publisher("/emotional_prediction", emotional_prediction, queue_size=1000)

    # carga de parámetros
    update_params()

    # se carga el modelo, solo para pruebas
    # srv_load = rospy.ServiceProxy("load_emotion_model", load_emotion_model)
    # srv_load("SRAG", "DeepConvNet", "emoModelDeepConvNet_1", 1, 1)

    rospy.spin()


if __name__ == "__main__":
    init_server()


# def train_model(req):
#     global window_size, model_name, model, umbral_emo, id_model, root_dir, dir_model

#     if(model_name == "" or model == None):
#         print("---- No se a cargado ningún modelo para entrenar----")
#         return train_onlineResponse()

#     print("----- Iniciando entrenamiento con :", model_name, "-----")

#     ids_pareto_fronts = req.ids_pareto_fronts
#     num_inds = req.num_inds
#     ids_eeg_signals = req.ids_eeg_signals
#     index_actual_ind = req.index_actual_ind

#     #se obtienen las señales EEG
    # print("load video: ", is_start, is_stop)_signals),"ID para las señales EEG")
#         print("   Hay",len(num_inds),"individuos para cargar sus señales")
#         print("Deben tener el mismo tamaño.")
#         return False

#     print("shape eeg",eeg_data.shape)
#     #etiquetas
#     #todas las ventanas tienen la misma etiqueta
#     emotion_class_ = get_emotion_class(req.valence, req.arousal, umbral_emo)
#     labels = np.ones(eeg_data.shape[0])*emotion_class_
#     labels = np_utils.to_categorical(labels, num_classes=num_classes)

#     #dirección del modelo
#     dir_model = f"{root_dir}/{req.user_name}/emotion_model/{model_name}/{id_model}"
#     create_folder(dir_model)

#     #conteo de las veces que baja seguidamente la métrica
#     count_metric_down = 0
#     #evaluación en una época anterior
#     old_eval = None
#     path_best = f"{dir_model}/best_metric.txt"
#     if os.path.isfile(path_best):
#         with open(path_best,"r") as fmr:
#             old_eval = float(fmr.read())

#     #metricas de evaluación
#     metrics = {"recall":[], "precision":[], "accuracy":[], "loss":[]}
#     #numeros de batch generados
#     num_batch = int(eeg_data.shape[0]/req.batch_size)
#     print("Se generaron",num_batch,"lotes")
#     ##su equivalente en muestras
#     num_batch_samples = num_batch * req.batch_size
#     #entrenamiento en N épocas
#     for iter in range(req.num_epoch):
#         print("-------------------\nEpoca", iter+1)
#         #se onden aleatoriamente los datos
#         np.random.shuffle(eeg_data)

#         for n in range(num_batch):
#             ini = n*req.batch_size
#             ##índice para seleccionar un subconjunto de datos eeg
#             index_batch = np.arange(req.batch_size) + ini
#             #datos del batch
#             data_batch = eeg_data[index_batch]
#             label_batch = labels[index_batch]
#             #entrenamiento
#             eval_train = model.train_on_batch(data_batch, label_batch)

#         #se prueba el modelo
#         eval_model = model.test_on_batch(data_batch, label_batch)
#         print("loss:", eval_model[0]," - accuracy", eval_model[1]," - recall", eval_model[2]," - precision", eval_model[3])
#         #se almacenan los valores de las metricas
#         # [loss, 'accuracy', "Recall", "Precision"]
#         metrics["loss"] = np.concatenate((metrics["loss"], [eval_model[0]]))
#         metrics["accuracy"] = np.concatenate((metrics["accuracy"], [eval_model[1]]))
#         metrics["recall"] = np.concatenate((metrics["recall"], [eval_model[2]]))
#         metrics["precision"] = np.concatenate((metrics["precision"], [eval_model[3]]))

#         #metrica actual, usada para salir del entrenamiento
#         val_current_metric = eval_model[0]

#         #condición de paro
#         if not old_eval is None:
#             #la metrica aumenta aumenta
#             if old_eval < val_current_metric:
#                 #si se encuentra que la metrica es mayor, se reinicia el conteo
#                 count_metric_down = 0
#             else: #la metrica baja
#                 best_metric = 0
#                 ##se carga el mejor valor
#                 if os.path.isfile(path_best):
#                     with open(path_best,"r") as fmr:
#                         best_metric = float(fmr.read())

#                 print("mejor metrica", best_metric)

#                 #se guarda el mejor modelo
#                 if best_metric > val_current_metric:
#                     #se guarda el mejor modelo
#                     model.save(dir_model+"/"+model_name+".h5")
#                     #se guarda la mejor métrica
#                     with open(path_best, "w") as fmw:
#                         fmw.write(str(val_current_metric))

#                 #la metrica baja en un número de epocas seguidas, igual a la tolerancia
#                 count_metric_down += 1
#                 if(count_metric_down >= req.tolerance):
#                     break
#         else:
#             #se guarda la mejor métrica, si no hay con anterioridad
#             #valor de la mejor métrica se almacena la actual
#             with open(path_best, "w") as fmw:
#                 fmw.write(str(val_current_metric))

#         #se almacena el valor de la metrica para la siguiente época
#         old_eval = val_current_metric


#     ##se publican las métricas
#     msg_monitor = monitor_cnn()
#     msg_monitor.loss = metrics["loss"]
#     msg_monitor.accuracy = metrics["accuracy"]
#     msg_monitor.recall = metrics["recall"]
#     msg_monitor.precision = metrics["precision"]
#     topic_monitor.publish(msg_monitor)
#     ##se guardan los resultados
#     with open(f"{dir_model}/metrics.txt", "w") as f_metric:
#         for m in metrics:
#             metrics[m] = metrics[m].tolist()
#         f_metric.write(json.dumps(metrics))

#     # se guardan las etiquetas de los experimentos
#     for k in range(len(ids_pareto_fronts)):
#         id_p = ids_pareto_fronts[k]
#         id_sig = ids_eeg_signals[k]
#         n_ind = num_inds[k]

#         experiment = {}
#         experiment["arousal"] = req.arousal[k]
#         experiment["valence"] = req.valence[k]
#         experiment["emotion_class"] = emotion_class_

#         # #se almacenan las etiquetas de entrenamiento
#         path_lab = f"{root_dir}/{req.user_name}/pareto_fronts/front{id_p}/ind_{n_ind}/{id_sig}_label.txt"
#         save_file(path_lab, json.dumps(experiment))

#     print("---- Exito al entrenar ----")
#     return train_onlineResponse()




# devuelve el número de segmentos a generar con un array
# de tamaño N dado el tamaño de la ventana y el desplazamiento
# def get_num_seg(req):
#     global window_size, stride
#     if window_size <= 0 or stride <= 0:
#         print(f"Error, window_size:{window_size}  stride:{stride}")
#         return False

#     res = get_num_segmentsResponse()

#     rest = req.size_signal - window_size
#     if rest < 0:
#         res.num_segments = 0
#         return res

#     n = rest // stride
#     # num segmentos
#     res.num_segments = n + 1

#     # sobrante
#     res.rest = req.size_signal - (n * stride + stride)

#     return res





# def get_emotion_class(valence, arousal, umbral):
    
#     if valence >= umbral:
#         if arousal >= umbral:
#             return 0
#         else:
#             return 3
#     else:
#         if arousal >= umbral:
#             return 1
#         else:
#             return 2

# def update_distribution_front(user_name, id_pareto):
#     global num_classes

#     # ruta del frente
#     dir_front = f"{root_dir}/{user_name}/{get_type_experiment(id_pareto)}/{id_pareto}/"

#     # distribución global
#     front_dist = np.zeros((num_classes))

#     # individuos del frente
#     name_ind = os.listdir(dir_front)
#     for name in name_ind:
#         # solo individuos válidos
#         if name.find("ind_") >= 0:
#             # cargamos la distribución de los individuos
#             dir_ind = f"{dir_front}/{name}/emo_dist_history.out"
#             individual_dist = load_json(dir_ind)

#             # print(name, individual_dist["distribution"])

#             if len(individual_dist.keys()) > 0:
#                 front_dist = front_dist + individual_dist["distribution"]

#     # emoción predicha
#     global_pred = get_max_emotion(front_dist)

#     # guardamos la info
#     obj_g = {"distribution": front_dist.tolist(), "emotion_predicted": global_pred}
#     save_json(obj_g, dir_front+"/front_distribution.out")
#     print("\nfront_distribution", obj_g)

#     return front_dist


# def update_distribution_global(user_name, id_pareto, distribution):
#     distribution = distribution.tolist()

#     # cargamos los datos de los frentes
#     dir_global = f"{root_dir}/{user_name}/{get_type_experiment(id_pareto)}/global_distribution.out"
#     distribution_g = load_json(dir_global)

#     if len(distribution_g.keys()) == 0 or not has_attr(distribution_g, "pos_distribution"):
#         distribution_g = {"front_distributions": [], "pos_distributions": {
#         }, "distribution": [], "emotion_predicted": -1}
#         save_json(distribution_g, dir_global)

#     # se crea un registro del frente si no existe
#     if (not has_attr(distribution_g["pos_distributions"], id_pareto)):
#         # num frentes explorados
#         distribution_g["pos_distributions"][id_pareto] = len(
#             distribution_g["front_distributions"])
#         distribution_g["front_distributions"].append(distribution)

#     # se actualiza la distribución
#     pos = distribution_g["pos_distributions"][id_pareto]
#     distribution_g["front_distributions"][pos] = distribution

#     # distribución global
#     dist_g = np.array(distribution_g["front_distributions"]).sum(axis=0)
#     # emoción predicha
#     emo_pred = get_max_emotion(dist_g)

#     distribution_g["distribution"] = dist_g.tolist()
#     distribution_g["emotion_predicted"] = emo_pred

#     # se almacena la distribución global
#     save_json(distribution_g, dir_global)

#     return dist_g


# def update_distributions(user_name, id_pareto, num_ind):
#     # ruta del individuo
#     t = get_type_experiment(id_pareto)
#     dir_experiment = f"{root_dir}/{user_name}/{t}/{id_pareto}/ind_{num_ind}"

#     # distribución local (actual experimento)
#     distribution_exp = load_json(
#         dir_experiment+"/emo_dist_actual_experiment.out")

#     # cargamos el historial de experimentos
#     histo = load_json(dir_experiment+"/emo_dist_history.out")
#     if len(histo.keys()) == 0:
#         histo = {"history_distribution": [],
#                  "history_emo_pred": [], "distribution": []}

#     print("\n\ndistribution_exp", distribution_exp)
#     # nueva distribución en el historial
#     histo["history_distribution"].append(distribution_exp["distribution"])
#     histo["history_distribution"] = np.array(histo["history_distribution"])

#     # emocion predicha con la distribución del experimento actual
#     histo["history_emo_pred"].append(distribution_exp["predict"])

#     # distribución del individuo
#     histo["distribution"] = histo["history_distribution"].sum(axis=0).tolist()

#     # predicción de la emoción del individuo
#     histo["emotion_predicted"] = get_max_emotion(histo["distribution"])

#     # se guarda el historial del individuo
#     histo["history_distribution"] = histo["history_distribution"].tolist()
#     save_json(histo, dir_experiment+"/emo_dist_history.out")

#     # se reinicia el historial del experimento
#     save_json({"distribution": [0 for i in range(
#         num_classes)], "predict": -1}, dir_experiment+"/emo_dist_actual_experiment.out")

#     dist_f = update_distribution_front(user_name, id_pareto)
#     print("global_distribution", dist_f)
#     dist_g = update_distribution_global(user_name, id_pareto, dist_f)

#     return dist_f, dist_g


# retorna la posición con el valor mas grande
# def get_max_emotion(distribution):
#     # emociones con la ocurrencia más alta
#     max_emotion = np.argwhere(distribution == np.amax(distribution))
#     size = max_emotion.shape[0]
#     # si varias emociones tienen la misma proporcion se selecciona aleatoriamente
#     if size > 0:
#         index = random.randint(0, size-1)
#     else:
#         index = 0
#     # clase predicha
#     class_predicted = max_emotion[index][0]
#     return int(class_predicted)


# def save_distribution_emotion(y_pred_, user_name, id_experiment, num_ind, is_stop):
#     global topic_pred_emo

#     # se obtienen las coordenadas de las predicciones
#     emo_class = []
#     # num de clase
#     for i in range(y_pred_.shape[0]):
#         p1 = y_pred_[i][0] >= 0.5
#         p2 = y_pred_[i][1] >= 0.5

#         if not p1 and not p2:
#             n_class = 2
#         if not p1 and p2:
#             n_class = 1
#         if p1 and not p2:
#             n_class = 3
#         if p1 and p2:
#             n_class = 0

#         emo_class.append(n_class)

#     # distribución
#     elements, count_elem = np.unique(emo_class, return_counts=True)
#     # por si no aparece una clase
#     distribution = np.zeros(num_classes)
#     for i in range(elements.shape[0]):
#         n_class = elements[i]
#         distribution[n_class] = count_elem[i]

#     # ruta de la distribución del experimento
#     t = get_type_experiment(id_experiment)
#     dir_ind = f"{root_dir}/{user_name}/{t}/{id_experiment}/ind_{num_ind}"
#     dir_dist = dir_ind+"/emo_dist_actual_experiment.out"

#     # se verifica si no existe la distribución
#     if (not os.path.isfile(dir_dist)):
#         # dirección de la nueva distribución
#         create_folder(dir_ind)
#         # se almacena una nueva distribución
#         new_dist = {"distribution": [0 for i in range(num_classes)], "predict": -1}
#         save_json(new_dist, dir_dist)

#     # se carga la distribución del individuo
#     distribution_old = load_json(dir_dist)
#     distribution_old["distribution"] = np.array(
#         distribution_old["distribution"])

#     # se actualiza la distribución
#     distribution = distribution + distribution_old["distribution"]

#     # emoción predicha
#     class_predicted = get_max_emotion(distribution)

#     # se guarda la distribución
#     distribution_old["distribution"] = distribution.tolist()
#     distribution_old["predict"] = class_predicted
#     save_json(distribution_old, dir_dist)

#     # mensaje de parada
#     dist_f = []
#     dist_g = []
#     if is_stop:
#         # se guardan las distribuciones del individuo
#         dist_f, dist_g = update_distributions(
#             user_name, id_experiment, num_ind)

#     return class_predicted, distribution, dist_f, dist_g


# def save_predictions(array_emo_pred, user_name, id_experiment, num_ind, id_eeg):
#     type_id = get_type_experiment(id_experiment)
#     # dir de los evaluaciones
#     dir_ = f"{root_dir}/{user_name}/{type_id}/{id_experiment}/ind_{num_ind}"

#     # si la dir no existe
#     if not os.path.isdir(dir_):
#         create_folder(dir_)

#     emo_dat = {"emotional_predictions": array_emo_pred.tolist(),
#                "id_eeg_signal": id_eeg}

#     dir_file = f"{dir_}/emotional_predictions_eeg.txt"
#     # si no existen las evaluciones
#     if not os.path.isfile(dir_file):
#         save_json([], dir_file)

#     # se agrega las predicciones
#     eval_aux = load_json(dir_file)
#     eval_aux.append(emo_dat)

#     # se guardan las predcciones
#     save_json(eval_aux, dir_file)
