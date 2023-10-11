#!/usr/bin/env python3

import sys
from os.path import expanduser
home = expanduser("~")
sys.path.insert(1, f"{home}/catkin_ws/src/emotion_classification/src")
sys.path.insert(2, f"{home}/catkin_ws/src/emotion_classification/models/arl-eegmodels")
sys.path.insert(3, f"{home}/catkin_ws/src/neurocontroller_database/src")
from brainflow.data_filter import DataFilter
from tensorflow.keras.losses import CosineSimilarity
# from tensorflow.keras.losses import MeanSquaredLogarithmicError
from tensorflow.keras.callbacks import EarlyStopping
from tensorflow.keras.callbacks import ModelCheckpoint
from sklearn.model_selection import KFold
from tensorflow.keras.models import load_model
from tensorflow.keras import utils as np_utils
from tensorflow.keras.metrics import RootMeanSquaredError
from keras import metrics
from eeg_signal_acquisition.srv import eeg_block_srv, eeg_block_srvResponse
from neurocontroller_database.srv import get_id
from experiment_control_interfaces.srv import plot_predictions
from emotion_classification.srv import control_emotions, control_emotionsResponse
from eeg_signal_acquisition.msg import eeg_block
from emotion_classification.msg import complete_training
from evaluations import get_emotions
from utilities import load_json, save_json, print_color, print_error, bcolors, get_type_experiment, has_attr
from processing import processing_signal, set_metadata, preprocessing_signal_ros, norm_dataset, get_segments
from EEGModels2 import ShallowConvNet
from EEGModels2 import DeepConvNet
from EEGModels2 import EEGNet
import datetime
from tensorflow.python.client import device_lib
import numpy as np
import os, gc, rospy, json, threading
import pickle
import tensorflow.keras.backend as K
from tensorflow.keras.utils import set_random_seed


#X_train = np.swapaxes(sc.fit_transform(X_train)[:, np.newaxis, :], 2, 3)
# X_valid = np.swapaxes(sc.transform(X_valid)[:, np.newaxis, :], 2, 3)
# X_test = np.swapaxes(sc.transform(X_test)[:, np.newaxis, :], 2, 3) 

# from emotion_classification.srv import train_model_emotion, train_model_emotionResponse

# os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

# dirección raiz
root_dir = home+"/catkin_ws/src/neurocontroller_database/database"
arl_models = ["EEGNet", "DeepConvNet", "ShallowConvNet"]
model_name = None
model = None
window_size = None
stride_size = None
batch_size = None
tolerance = None
k_rep = None
n_fold = None
epochs = None

# identificador del modelo
id_model = ""

x_test = None
y_test = None
label_one = np.empty((0, 2))

eeg_data_global = []
labels_global = []
train_size = 70
test_size = 20
val_size = 10

is_training = False
num_channels = 16

# import numpy as np
# import sys
# from os.path import expanduser
# home = expanduser("~")
# sys.path.insert(1, f"{home}/catkin_ws/src/emotion_classification/models/arl-eegmodels")
# from EEGModels2 import DeepConvNet
# from tensorflow.keras.callbacks import EarlyStopping
# from tensorflow.keras.callbacks import ModelCheckpoint
# root_dir = home+"/catkin_ws/src/neurocontroller_database/database"
# x_tr = np.random.random((20,16,128))
# y_tr = np.random.random((20,2))
# x_val = np.empty((0,16,128))
# y_val = np.empty((0,2))
# model = DeepConvNet(nb_classes=2, Chans=16, Samples=128, dropoutRate=0.5)
# model.compile(loss='mean_squared_error', optimizer='adam', metrics=['mse', 'mae', 'mape'])
# checkpointer = ModelCheckpoint(filepath=f"{root_dir}/test.h5", verbose=1,  monitor='val_loss', save_best_only=True)
# es = EarlyStopping(monitor='val_loss', mode='auto', verbose=1, patience=5)
# fittedModel = model.fit(x_tr, y_tr, batch_size=32, epochs=10, validation_data=(x_val, y_val),
#                                     verbose=2, callbacks=[checkpointer, es])


def get_indices_partition(train, test, val, size_dataset):
    assert train+test+val == 100

    # se seleccionan aleatoriamente los datos
    indices = np.random.permutation(size_dataset)

    size_tr = round((size_dataset*train)/100)
    if val == 0:
        size_te = size_dataset-size_tr
        size_val = 0
    else:
        size_te = round((size_dataset*test)/100)
        size_val = size_dataset-(size_tr+size_te)

    i_tr = indices[:size_tr]
    i_te = indices[size_tr: size_tr+size_te]
    i_val = indices[size_tr+size_te:]

    return i_tr, i_te, i_val

def save_metadata(num_rep, num_fold, x_tr_shape, x_val_shape, x_te_shape, time_histo, dir_):
    metadata = {}
    metadata["size_fold"] = n_fold
    metadata["size_rep"] = k_rep
    metadata["num_fold"] = num_fold #count
    metadata["num_rep"] = num_rep #k
    metadata["model_name"] = model_name
    metadata["window_size"] = window_size
    metadata["stride_size"] = stride_size
    metadata["batch_size"] = batch_size
    metadata["tolerance"] = tolerance
    metadata["epochs"] = epochs

    # tamaño del dataset
    metadata["x_tr_shape"] = x_tr_shape
    metadata["x_val_shape"] = x_val_shape
    metadata["x_te_shape"] = x_te_shape
    # tiempos de ejecucion
    metadata["time_train"] = (
        time_histo["end_train"]-time_histo["ini_train"]).total_seconds() * 1000
    metadata["time_predict"] = (
        time_histo["end_predict"]-time_histo["ini_predict"]).total_seconds() * 1000
    metadata["time_unit"] = "miliseg"

    save_json(metadata, dir_)

def get_ID(key, dir_):
    rospy.wait_for_service("get_id")
    try:
        g_id = rospy.ServiceProxy("get_id", get_id)
        res = g_id(key, dir_)
        return res.id
    except rospy.ServiceException as e:
        print("Error al generar el ID emocional", e)


def map_val(val, ini_origin_range, end_origin_range, ini_new_range, end_new_range):

    dif_new = end_new_range - ini_new_range

    # tamaño rango inicial
    dif_origin = end_origin_range - ini_origin_range
    # posición del valor en el rango
    pos_origin = val - ini_origin_range

    # mapeo
    conv = (pos_origin * dif_new) / dif_origin
    conv += ini_new_range
    return conv


def control_train(req):
    global model_name, k_rep, n_fold, batch_size, stride_size, window_size, epochs, tolerance

    print("Control emotions train")

    if req.model_name != "":
        model_name = req.model_name
        print_color(f"model_name {model_name}", bcolors.OKBLUE)

    if req.k_repetitions != 0:
        k_rep = req.k_repetitions
        print_color(f"k_rep {k_rep}", bcolors.OKBLUE)
    
    if req.size_fold != 0:
        n_fold = req.size_fold
        print_color(f"n_fold {n_fold}", bcolors.OKBLUE)

    if req.stride != 0:
        stride_size = req.stride
        print_color(f"stride_size {stride_size}", bcolors.OKBLUE)
    
    if req.window_size > 0:
        window_size = req.window_size
        print_color(f"window_size {window_size}", bcolors.OKBLUE)
    
    if req.num_epoch > 0:
        epochs = req.num_epoch
        print_color(f"epochs {epochs}", bcolors.OKBLUE)
    
    if req.batch_size > 0:
        batch_size = req.batch_size
        print_color(f"batch_size {batch_size}", bcolors.OKBLUE)
    
    if req.tolerance > 0:
        tolerance = req.tolerance
        print_color(f"tolerance {tolerance}", bcolors.OKBLUE)

    return control_emotionsResponse()


def plot_predicts(y_true, y_pred, user_name, id_experiment, num_k, num_fold):
    try:
        if y_true.shape != y_pred.shape:
            print("----- No corresponden en tamaño las predicciones con las etiquetas",
                  y_true.shape, " != ", y_pred.shape, "-----")

        # numero de predicciones
        num_preds = y_true.shape[0]

        # pred con mas de dos valores
        if len(y_true.shape) > 1:
            y_t = y_true.reshape(y_true.shape[0]*y_true.shape[1])
            y_p = y_pred.reshape(y_pred.shape[0]*y_pred.shape[1])
        else:
            y_t = y_true
            y_p = y_pred

        srv_plot = rospy.ServiceProxy("plot_predictions", plot_predictions)
        srv_plot(user_name, id_experiment, num_fold,
                 num_k, y_t, y_p, num_preds)

    except rospy.ServiceException as e:
        print_error(f"Error al imprimir en la GUI las predicicones\n\n {e} \n")




def get_data_deap(user_name):
    dir_eeg = f"{root_dir}/deap/{user_name}.dat"
    # existe el archivo del usuario
    if not os.path.isfile(dir_eeg):
        print_error(f"El usuario {user_name} no cuenta con su archivo en {dir_eeg}")
        return False

    # carga de los datos eeg
    with open(dir_eeg, 'rb') as f:
        data = pickle.load(f, encoding='latin1')
        lab = np.array(data["labels"][:, :2])
        lab = map_val(lab,1, 9, 0.01, 1)
        data = data["data"][:, :32, :]

    x_fold = None
    y_fold = None
    x_test = None
    y_test = None
    for num_exp in range(data.shape[0]):
        print(f"process experiment", num_exp)

        # # datos eeg
        # set_metadata(user_name, "deap", num_exp)
        # data_eeg = processing_signal(data[num_exp], window_size, stride_size)
        
        data_eeg = data[num_exp]

        ini_signal = 128*3
        # ventaneo de la señal base
        basal = data_eeg[:32 , :ini_signal]
        basal, res_b = get_segments(basal, window_size, window_size)
        # promediamos la señal basal
        basal = np.mean(basal, axis=0)

        #se extraen las ventanas: input shape=(n_channel,n_sample)
        data_eeg, res_s = get_segments(data_eeg, window_size, stride_size)
        data_eeg = data_eeg - basal

        # labels
        y_data = np.zeros((data_eeg.shape[0], 2))
        y_data[:, 0].fill(lab[num_exp][0])
        y_data[:, 1].fill(lab[num_exp][1])

        # tamaño dataset
        size_te = int((data_eeg.shape[0]*test_size)/100)
        if size_te == 0:
            size_te = -1*data_eeg.shape[0]

        # division de los datos 
        x_f = data_eeg[:-size_te]
        x_t =  data_eeg[-size_te:]

        y_f = y_data[:-size_te]
        y_t = y_data[-size_te:]
        
        # conjuntos de datos
        if x_fold is None:
            x_fold = x_f
            x_test = x_t
            y_fold = y_f
            y_test = y_t
        else:
            x_fold = np.vstack((x_fold, x_f))
            x_test = np.vstack((x_test, x_t))
            y_fold = np.vstack((y_fold, y_f))
            y_test = np.vstack((y_test, y_t))


    print(x_fold.shape, y_fold.shape, x_test.shape, y_test.shape)
    return x_fold, y_fold, x_test, y_test





def get_data_protocol(user_name):
    global window_size, stride_size

    dir_ = f"{root_dir}/{user_name}/protocol"

    print("Protocol: ",dir_)
    # ruta de la info del último experimento
    dir_last = f"{dir_}/history/last_experiment.txt"
    # se carga el ultimo experimento
    last_obj = load_json(dir_last)
    # nombre del archivo del experimento a usar
    name_video_exp = last_obj["history_emotion_evaluation"]

    # ruta de los datos emocionales del protocolo
    path_v = f"{dir_}/history/{name_video_exp}"
    # se cargan las evaluaciones
    emo_data = load_json(path_v)

    # CORRECCIÓN
    # if user_name == "DABB":
    #     emo_data = emo_data[:15]
    
    # emo_data = emo_data[:6]

    x_fold = None
    y_fold = None
    x_test = None
    y_test = None
    for num_exp in range(len(emo_data)):
        # CORRECCIÓN
        if user_name == "DPB" and num_exp == 5:
            continue

        # se carga la info del número de video
        eval_exp = emo_data[num_exp]

        # ETIQUETAS DEL VIDEO
        if not has_attr(eval_exp, "valence") or not has_attr(eval_exp, "arousal"):
            print_error(f"El experimento {num_exp} no tiene datos emocionales")
            return False
        valence = float(eval_exp["valence"])
        arousal = float(eval_exp["arousal"])

        # id de las señales
        if not has_attr(eval_exp, "id_eeg_signal"):
            print_error("El experimento no cuenta con id_eeg_signal")
            return False
        id_signal = eval_exp["id_eeg_signal"]

        # número de video
        num_video = eval_exp["num_video"]

        # preprocesamiento
        preprocessing_signal_ros(user_name, "protocol_video", num_exp, id_signal)

        # ruta del archivo
        path_eeg = f"{dir_}/protocol_video/ind_{num_video}/{id_signal}_preprocessing.csv"
        # se verifica que exista el archivo de las señales
        if not os.path.isfile(path_eeg):
            print_error(f"No existe el archivo {path_eeg}")
            return False

        # DATOS EEG
        data_eeg = DataFilter.read_file(path_eeg)
        data_eeg = data_eeg[1:17]

        # PROCESAMIENTO DE LA SEÑAL (extración señal base)
        set_metadata(user_name, "protocol_video", num_exp)
        data_eeg = processing_signal(data_eeg, window_size, stride_size)

        # labels
        y_data = np.zeros((data_eeg.shape[0], 2))
        y_data[:, 0].fill(valence)
        y_data[:, 1].fill(arousal)
        
        # tamaño dataset
        size_te = int((data_eeg.shape[0]*test_size)/100)
        if size_te == 0:
            size_te = -1*data_eeg.shape[0]

        # division de los datos 
        x_f = data_eeg[:-size_te]
        x_t =  data_eeg[-size_te:]

        y_f = y_data[:-size_te]
        y_t = y_data[-size_te:]

        print_color(f"--- Agregando experimento user:{user_name} id_experiment:protocol_video num_ind:{num_exp} ---", bcolors.OKGREEN)
        
        # conjuntos de datos
        if x_fold is None:
            x_fold = x_f
            x_test = x_t
            y_fold = y_f
            y_test = y_t
        else:
            x_fold = np.vstack((x_fold, x_f))
            x_test = np.vstack((x_test, x_t))
            y_fold = np.vstack((y_fold, y_f))
            y_test = np.vstack((y_test, y_t))


    print(x_fold.shape, y_fold.shape, x_test.shape, y_test.shape)
    return x_fold, y_fold, x_test, y_test


def load_data_train_srv(req):
    res = capture_data(req)

    if res:
        return eeg_block_srvResponse()
    else:
        return False


def load_data_train_topic(msg):
    capture_data(msg)


def capture_data(params):
    global x_test, y_test, eeg_data_global, labels_global
    global label_one, test_size
    global is_training, num_channels

    if model_name == None:
        print_error("++++ No hay modelo seleccionado ++++")
        return False
    if is_training:
        print_error(f"++++ El modelo {model_name} se encuentra en entrenamiento +++++")
        return False

    user_name = params.user_name
    id_experiment = params.id_pareto_front
    # num_ind = params.num_ind
    # id_eeg_signal = params.id_eeg_signal
    # is_stop = params.is_stop
    # is_start = params.is_start


    # datos 1D to 2D
    # eeg_data = np.array(params.eeg_data).reshape(
    #     params.num_channels, params.num_samples)
    # eeg_data = eeg_data[:16, :]


    # # PROCESAMIENTO DE LA SEÑAL
    # set_metadata(user_name, id_experiment, num_ind)
    # eeg_data = processing_signal(eeg_data, window_size, stride_size)

    # print("Experimento con forma = EEG:", eeg_data.shape, " labels:",len(labels))

    # # se almacenan los experimentos
    # size_te = int((eeg_data.shape[0]*test_size)/100)
    # if size_te == 0:
    #     size_te = -1*eeg_data.shape[0]

    # # map labels para DEAP
    # if id_experiment.find("deap") >= 0 or id_experiment.find("DEAP") >= 0:
    #     # lab1 = map_val(params.labels[0], 1, 9, 0.01, 1)
    #     # lab2 = map_val(params.labels[1], 1, 9, 0.01, 1)
    # else:
    #     lab1 = params.labels[0]
    #     lab2 = params.labels[1]

    # y_data = np.zeros((eeg_data.shape[0], 2))
    # y_data[:, 0].fill(lab1)
    # y_data[:, 1].fill(lab2)

    # if is_start or x_test is None:
    #     # datos eeg
    #     eeg_data_global = eeg_data[:-size_te]
    #     x_test = eeg_data[-size_te:]

    #     # labels
    #     labels_global = y_data[:-size_te]
    #     y_test = y_data[-size_te:]
    # else:
    #     # datos eeg
    #     eeg_data_global = np.vstack((eeg_data_global, eeg_data[:-size_te]))
    #     x_test = np.vstack((x_test, eeg_data[-size_te:]))

    #     # labels
    #     labels_global = np.vstack((labels_global, y_data[:-size_te]))
    #     y_test = np.vstack((y_test, y_data[-size_te:]))

    # etiquetas de las ventanas
    # print("Datos para la validación cruzada:", eeg_data_global.shape,
    #       " ventanas y ", labels_global.shape, "etiquetas")
    # print("Datos para la prueba:", x_test.shape,"ventanas y ", y_test.shape,"etiquetas")

    # if not is_stop:
    #     print_color(f"Añadiendo datos {len(eeg_data)}", bcolors.OKBLUE)
    #     return True
    # eeg_data = None
    # y_data = None

    if id_experiment.find("protocol") >= 0:
        eeg_data_global, labels_global, x_test, y_test = get_data_protocol(user_name)
        num_channels = 16

    elif id_experiment.find("deap") >= 0:
        eeg_data_global, labels_global, x_test, y_test = get_data_deap(user_name)
        num_channels = 32
    else:
        print(f"Error: experimento {id_experiment} no encontrado")
        return False
    
    # se lanza el hilo del entrenamiento
    train = threading.Thread(target=train_model, args=(user_name, id_experiment))

    is_training = True
    train.start()

    return True

def train_model(user_name, id_experiment): #(eeg_data_global, labels_global, ,x_test, y_test):
    global root_dir
    global x_test, y_test, eeg_data_global, labels_global
    global label_one
    global train_size, test_size, val_size, batch_size, stride_size
    global id_model, model
    global k_rep, n_fold, window_size, model_name
    global is_training, num_channels

    print_color(f"\n\nCalculando ", bcolors.OKCYAN)
    print_color(f"\nparámetros = model_name: {model_name} stride_size: {stride_size} window_size: {window_size} k_rep: {k_rep} n_fold: {n_fold} epochs: {epochs} batch_size: {batch_size} tolerance: {tolerance}", bcolors.OKCYAN)
    

    dir_model_base = f"{root_dir}/{user_name}/emotion_model/{model_name}"
    if not os.path.isdir(dir_model_base):
        os.makedirs(dir_model_base)

    # id del modelo
    key_ = f"emoModel{model_name}_"
    id_model = get_ID(key_, dir_model_base)
    dir_model = f"{dir_model_base}/{id_model}"
    if not os.path.isdir(dir_model):
        os.makedirs(dir_model)

    # se obtienen los indices de las muestras de forma aleatoria para cada partición
    # index_fold, _, index_val = get_indices_partition(
    #     train_size + test_size, 0, val_size, size_experiments)

    index_fold = np.random.permutation(eeg_data_global.shape[0])

    # guarda las direcciones de las evaluaciones
    model_eval_path = load_json(f"{root_dir}/{user_name}/emotion_model/evaluations_path.txt")
    if not has_attr(model_eval_path, model_name):
        model_eval_path[model_name] = {}

    if not has_attr(model_eval_path[model_name], id_model):
        model_eval_path[model_name][id_model] = {}

    print("+++++++++++++ Iniciando la validación cruzada +++++++++++++")
    # REPETICIONES
    for k in range(k_rep):
        k += 1
        np.random.seed(423)
        set_random_seed(423)

        # se reordenan las muestras
        np.random.shuffle(index_fold)
        count = 0

        if n_fold == 1:
            size_index = len(index_fold)
            # num muestras para el entrenamiento
            s_val = int((size_index*val_size) / 100)
            # num muestras para el entrenamiento
            s_tr = size_index - s_val
            # indices
            i_vals = np.arange(size_index)
            exp = [(i_vals[:s_tr], i_vals[s_tr:])]
        else:
            # se mezclan y se obtienen los indices para dividir el data set
            kfold = KFold(n_splits=n_fold)  # shuffle=True)
            exp = kfold.split(np.arange(len(index_fold)))

        for index_tr_, index_val_ in exp: #kfold.split(np.arange(len(index_fold)))
            count += 1
            print(f"\n\n\n\n--- k:{k}/{k_rep} fold:{count}/{n_fold} del modelo emocional:{id_model} ---")

            # indices
            index_val = index_fold[index_val_]
            index_tr = index_fold[index_tr_]

            # datos
            if n_fold == 1:
                x_tr_aux = eeg_data_global
                y_tr = labels_global

                x_val_aux = x_test
                y_val = y_test
            else:
                x_tr_aux = eeg_data_global[index_tr]
                y_tr = labels_global[index_tr]

                x_val_aux = eeg_data_global[index_val]
                y_val = labels_global[index_val]
            
            print(x_tr_aux.dtype, x_val_aux.dtype)
            # dirección del fold
            dir_out_fold = f"{dir_model}/k_{k}_fold_{count}"
            if not os.path.isdir(dir_out_fold):
                os.makedirs(dir_out_fold)
            
            chan_dat = {}
            # prom y std de todas las ventanas en cada canal
            mean_windows = x_tr_aux.mean(axis=2)
            std_windows = x_tr_aux.std(axis=2)
            # prom y std de todas las ventanas en su respectivo canal
            mean = mean_windows.mean(axis=0)
            std = std_windows.std(axis=0)

            chan_dat = {"mean": mean.tolist(), "std":std.tolist()}
            
            # (ventana, canal, muestra) a (canal, ventana, muestra)
            x_tr_aux = x_tr_aux.transpose(1,0,2)
            x_val_aux = x_val_aux.transpose(1,0,2)

            x_tr = np.empty(x_tr_aux.shape)
            x_val = np.empty(x_val_aux.shape)
            # # ESTANDARIZACIÓN DE LA SEÑAL
            for n_chan in range(x_tr_aux.shape[0]):
                x_tr[n_chan] = (x_tr_aux[n_chan] - mean[n_chan])/std[n_chan]
                x_val[n_chan] = (x_val_aux[n_chan] - mean[n_chan]) / std[n_chan]

            x_tr = x_tr.transpose(1,0,2)
            x_val = x_val.transpose(1,0,2)
            
            print(x_tr.shape, x_val.shape)
            print(x_tr.std(axis=2).std(axis=0))
            print(x_val.std(axis=2).std(axis=0))

            # x_tr = np.empty(x_tr_aux.shape)
            # x_val = np.empty(x_val_aux.shape)
            # for n_chan in range(x_tr_aux.shape[1]):
                # # datos del canal n
                # data_chan = x_tr_aux[:,n_chan,:]
                # non_zero = data_chan.nonzero()
                # # calculo de la media y std
                # mean = data_chan[non_zero[0], non_zero[1]].mean()
                # std = data_chan[non_zero[0], non_zero[1]].std()
                # # ESTANDARIZACIÓN DE LA SEÑAL
                # x_tr[n_chan] = (data_chan - mean)/std
                # non_zero = x_val_aux[:,n_chan,:].nonzero()
                # x_val[n_chan] = (x_val_aux[non_zero[0], non_zero[1]] - mean)/std
                # se guardan los parámetros
                # chan_dat[f"chan_{n_chan}"] = {"mean": mean, "std": std}


            # x_val_aux = None
            # x_tr_aux = None

            # se guardan los parámetros de estandarización
            save_json(chan_dat, f"{dir_out_fold}/param_standardization.txt")

            print("\n\n\n validation= data:", x_val.shape, " label:",
                  y_val.shape, " test= data:", x_tr.shape, "label:", y_tr.shape)
            # # nueva sesion
            K.clear_session()
            
            # selección del modelo a usar
            if model_name == "EEGNet":
                model = EEGNet(nb_classes=2, Chans=num_channels, Samples=window_size, dropoutRate=0.5,
                               kernLength=32, F1=8, D=2, F2=16, dropoutType='Dropout')

            if model_name == "DeepConvNet":
                model = DeepConvNet(nb_classes=2, Chans=num_channels,
                                    Samples=window_size, dropoutRate=0.5)

            if model_name == "ShallowConvNet":
                model = ShallowConvNet(
                    nb_classes=2, Chans=num_channels, Samples=window_size, dropoutRate=0.5)
            
            # tiempo de las evaluaciones
            time_histo = {}
            # tiempo inicial del entrenamiento
            time_histo[f"ini_train"] = datetime.datetime.now()

            # compile the model and set the optimizers
            # 'accuracy', "Recall", "Precision",
            model.compile(loss='mean_squared_error',
                          optimizer='adam', metrics=['mse', 'mae', 'mape', "msle", RootMeanSquaredError()])# "root_mean_squared_error", "MeanSquaredLogarithmicError"])

            
            # m_aux = {"path": dir_out_fold, "num_rep": k, "num_fold": count}

            if not has_attr(model_eval_path[model_name][id_model], f"{k}"):
                model_eval_path[model_name][id_model][f"{k}"] = {}

            if not has_attr(model_eval_path[model_name][id_model][f"{k}"], f"{count}"):
                model_eval_path[model_name][id_model][f"{k}"][f"{count}"] = {}

            model_eval_path[model_name][id_model][f"{k}"][f"{count}"] = dir_out_fold


            # set a valid path for your system to record model checkpoints
            dir_b = f"{dir_out_fold}/{model_name}.h5"
            checkpointer = ModelCheckpoint(filepath=dir_b, verbose=1,  monitor='val_loss', save_best_only=True)
            es = EarlyStopping(monitor='val_loss', mode='auto', verbose=1, patience=tolerance, restore_best_weights=True)

            print(x_tr.shape, y_tr.shape, x_val.shape, y_val.shape)
            print("\n\n\n--- Entrenando el modelo ---")
            fittedModel = model.fit(x_tr, y_tr, batch_size=batch_size, epochs=epochs, validation_data=(x_val, y_val),
                                    verbose=2, callbacks=[checkpointer, es])
            
            # tiempo final del entrenamiento e inicio de la predicción
            time_histo[f"end_train"] = datetime.datetime.now()

            # load optimal weights
            model.load_weights(f'{dir_out_fold}/{model_name}.h5')

            # predicción
            time_histo[f"ini_predict"] = datetime.datetime.now()
            y_pred = model.predict(x_val)
            time_histo[f"end_predict"] = datetime.datetime.now()

            # se guardan las predicciones
            emo_pred = {"predictions": y_pred.tolist(),
                        "labels": y_val.tolist()}
            save_json(emo_pred, f"{dir_out_fold}/predictions_fold.out")

            # se muestran en la gui las predicciones
            # plot_predicts(y_val, y_pred, user_name, id_experiment, k, count)

            print("\n\n\n--- Iniciando la evaluación del modelo (CONJUNTO DE VALIDACIÓN)---")
            data_out = {}
            # evaluaciones para regresión
            histo_val = model.evaluate(x_val, y_val)
            data_evals = {}
            data_evals["loss"] = histo_val[0]
            data_evals["mse"] = histo_val[1]
            data_evals["mae"] = histo_val[2]
            data_evals["mape"] = histo_val[3]
            data_out["evaluations"] = data_evals

            data_out["history"] = fittedModel.history
            # data_out["metadata"] = metadata

            # se guardan las evaluaciones
            path_fold = f"{dir_out_fold}/eval_fold.txt"
            save_json(data_out, path_fold)

            # se guardan los metadatos
            dir_meta = f"{dir_out_fold}/metadata.txt"
            save_metadata(k, count, x_tr.shape, x_val.shape, x_test.shape, time_histo, dir_meta)
            x_tr = None
            y_tr = None
            x_val = None
            y_val = None
            gc.collect()

            print(f"+++ Fin del fold {count} ++++")
            # se termina el entrenamiento
            if n_fold == 1:
                break
            
            print("\n\n\n--- Iniciando la evaluación del modelo (CONJUNTO DE PRUEBA)---")
            # estandarización
            x_test = x_test.transpose(1,0,2)
            print(x_test.shape)
            x_test_norm = np.empty(x_test.shape)
            for n_chan in range(x_test.shape[0]):
                x_test_norm[n_chan] = (x_test[n_chan] - mean[n_chan])/std[n_chan]

            x_test = x_test.transpose(1,0,2)
            x_test_norm = x_test_norm.transpose(1,0,2)

            # evaluación del modelo
            histo_te = model.evaluate(x_test_norm, y_test)

            data_out = {}
            data_evals = {}
            data_evals["loss"] = histo_te[0]
            data_evals["mse"] = histo_te[1]
            data_evals["mae"] = histo_te[2]
            data_evals["mape"] = histo_te[3]
            data_out["evaluations"] = data_evals

            # se guardan las evaluaciones
            save_json(data_out, f"{dir_out_fold}/eval_test.txt")

            # predicciones
            y_pred_test = model.predict(x_test_norm)
            x_test_norm = None

            # se guardan las predicciones
            emo_pred_val = {"predictions": y_pred_test.tolist(),
                            "labels": y_test.tolist()}
            save_json(emo_pred_val, f"{dir_out_fold}/predictions_test.out")

            # plot de las pred en la gui
            # plot_predicts(y_test, y_pred_test, user_name,
            #               id_experiment, -1, -1)
            
            # nueva sesion
            # K.clear_session()

            print("+++++++++++ Fin de la evaluación del modelo +++++++++")
        
        # # se termina el entrenamiento
        # if n_fold == 1:
        #     break
                
    # se guardan las direcciones de las evaluaciones
    save_json(model_eval_path,f"{root_dir}/{user_name}/emotion_model/evaluations_path.txt")

    # carga de parámetros
    update_params()

    is_training = False

    x_test = y_test = None
    eeg_data_global = labels_global = None
    label_one = np.empty((0, 2))
    gc.collect()

    # termino del experimento
    pub = rospy.Publisher('complete_training', complete_training, queue_size=1)
    msg = complete_training()
    msg.complete = True
    pub.publish(msg)

    # servicio de terminación de entrenamiento
    return True

def update_params():
    global stride_size, window_size, k_rep, n_fold, epochs, batch_size, tolerance, model_name
    
    print("Cargando parámetros")
    
    params = load_json(f"{root_dir}/train_parameters.txt")
    print(params)

    stride_size = params["stride"]
    window_size = params["window_size"]
    k_rep = params["num_rep"]
    n_fold = params["num_fold"]
    epochs = params["num_epoch"]
    batch_size = params["batch_size"]
    tolerance = params["tolerance"]
    model_name = params["model_name"]
    # print(model_name, stride_size, window_size, k_rep, n_fold, epochs, batch_size, tolerance)


if __name__ == "__main__":

    rospy.init_node("model_selection")
    print("Nodo iniciado con éxito")

    print(device_lib.list_local_devices())
    update_params()

    rospy.Publisher('complete_training', complete_training, queue_size=2)

    topic_train = rospy.Subscriber("preprocessing_eeg_signals_train", eeg_block, load_data_train_topic)
    # servicios
    srv_train = rospy.Service("run_evaluations_model", eeg_block_srv, load_data_train_srv)
    srv_ctr = rospy.Service("control_emotions_train", control_emotions, control_train)
    
    # inicio del modelo
    model = ShallowConvNet(nb_classes=2, Chans=16, Samples=128, dropoutRate=0.5)

    rospy.spin()
