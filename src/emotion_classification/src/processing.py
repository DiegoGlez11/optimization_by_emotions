
import sys
from os.path import expanduser
home = expanduser("~")
sys.path.insert(2, f"{home}/catkin_ws/src/neurocontroller_database/src")
import numpy as np
from brainflow.data_filter import DataFilter
import os
import rospy
from eeg_signal_acquisition.srv import eeg_block_srv
from utilities import print_error, root_dir, load_json


user_name=None
id_experiment=None
num_ind=None
id_eeg_signal=None
type_signal=""
arl_models = ["EEGNet", "DeepConvNet", "ShallowConvNet", "DeepForest"]

def dataset_1Dto2D(dataset_1D):
    dataset_2D = np.zeros([dataset_1D.shape[0],9,9])
    for i in range(dataset_1D.shape[0]):
        dataset_2D[i] = data_1Dto2D(dataset_1D[i])
    # return shape: m*9*9
    return dataset_2D


def data_1Dto2D(data, Y=9, X=9):
    data_2D = np.zeros([Y, X])
    data_2D[0] = (0,  	   	0, 	        0,          data[0],    0,          data[16], 	0,  	    0, 	        0       )
    data_2D[1] = (0,  	   	0,          0,          data[1],    0,          data[17],   0,          0,          0       )
    data_2D[2] = (data[3],  0,          data[2],    0,          data[18],   0,          data[19],   0,          data[20])
    data_2D[3] = (0,        data[4],    0,          data[5],    0,          data[22],   0,          data[21],   0       )
    data_2D[4] = (data[7],  0,          data[6],    0,          data[23],   0,          data[24],   0,          data[25])
    data_2D[5] = (0,        data[8],    0,          data[9],    0,          data[27],   0,          data[26],   0       )
    data_2D[6] = (data[11], 0,          data[10],   0,          data[15],   0,          data[28],   0,          data[29])
    data_2D[7] = (0,        0,          0,          data[12],   0,          data[30],   0,          0,          0       )
    data_2D[8] = (0,        0,          0,          data[13],   data[14],   data[31],   0,          0,          0       )
    # return shape:9*9
    return data_2D

def norm_dataset_ACRNN(dataset_1D):
    # print("in func norm",dataset_1D.shape)
    #in func norm (7680, 32)
    norm_dataset_1D = np.zeros([dataset_1D.shape[0], 32])
    for i in range(dataset_1D.shape[0]):
        norm_dataset_1D[i] = feature_normalize(dataset_1D[i])
        
    # return shape: m*32
    return norm_dataset_1D


def norm_dataset(signals):
    mean = []
    sigma = []
    # shape(channel, samples)
    for i in range(signals.shape[0]):
        signals[i], m, s = feature_normalize(signals[i])
        mean.append(m)
        sigma.append(s)
    return signals, mean, sigma
    

def feature_normalize(data):
    nonzero = data.nonzero()
    mean = data[nonzero].mean()
    sigma = data[nonzero].std()
    data[nonzero] = (data[nonzero] - mean)/sigma
    return data, mean, sigma


#realiza un ventaneo con desplazamiento 
def get_segments(data, window_size, stride):
    b_element = data.itemsize # tamaño en bytes de un elemento en el arreglo
    stride_window =  b_element * stride #cada cuantas muestras se mueve la ventana en bytes
    size_row = b_element * data.shape[1] # tamaño en bytes de una fila
    #se calcula el número de segmentos
    r = data.shape[1] - window_size
    n = r//stride
    n_segments = n+1
    #número de datos sobrantes
    res = data.shape[1] - (n*stride+stride)
    #se extrae el ventaneo
    win = np.lib.stride_tricks.as_strided(data, strides=(stride_window, size_row, b_element), shape=(n_segments, data.shape[0], window_size)) 
    return win.copy(), data[-res:]


def set_metadata(usr_name, id_exp, n_ind, id_signal=None, type_sig =""):
    global user_name, id_experiment, num_ind, id_eeg_signal, type_signal

    user_name = usr_name
    id_experiment = id_exp
    num_ind = n_ind
    id_eeg_signal = id_signal
    type_signal = type_sig

def reset_metadata():
    global user_name, id_experiment, num_ind, id_eeg_signal, type_signal

    user_name = ""
    id_experiment = ""
    num_ind = -1
    id_eeg_signal = ""
    type_signal = ""

def get_sample_rate(id_signal):
    # taza de muestreo
    sr = 0
    if id_signal.find("cyton") >= 0:
        sr = 125
    if id_signal.find("simulation") >= 0:
        sr = 125
    if id_signal.find("synthetic") >= 0:
        sr = 250
    if id_signal.find("deap") >= 0:
        sr = 128

    return sr

def preprocessing_signal_ros(user_name, id_experiment, num_ind, id_eeg_signal, storage_pos = -1, type="guii"):
    # input signals shape(channels, samples)

    # señales EEG
    # num_channels = signals.shape[0]
    # num_samples = signals.shape[1]
    # eeg_data = signals.reshape(signals.shape[0]* signals.shape[1]);

    # sampling rate
    sampling_rate = get_sample_rate(id_eeg_signal)
    # tipo de señal
    # type_signal = "preprocessing_srv"

    rospy.wait_for_service("preprocessing_signal")
    try:
        srv_prepro = rospy.ServiceProxy("preprocessing_signal", eeg_block_srv)
        srv_prepro([], [], 0, 0, user_name, id_experiment, num_ind, id_eeg_signal, int(storage_pos), 0, False, False, sampling_rate, type)
        # res = srv_prepro([], eeg_data, num_channels, num_samples, user_name, id_pareto, num_ind, "", -1, 0, False, False, sampling_rate, type_signal)
        # data_prepro= np.array(res.eeg_data)
        # data_prepro = data_prepro.reshape(res.num_channels, res.num_samples)
        # return data_prepro
        return True
    except rospy.ServiceException as e:
        print_error(f"Error al preprocesar las señales:\n\n{e}")

    # rospy.wait_for_service("preprocessing_signal")
    # try:
        # srv_prepro = rospy.ServiceProxy("preprocessing_signal", eeg_block_srv)
        # srv_prepro([], [], 0, 0, user_name, "optimized_basal", num_ind, id_eeg_signal, 0, 0, False, False, sampling_rate, "preprocessing_srv")
    # except rospy.ServiceException as e:
    #     print_error(f"Error al preprocesar las señales:\n\n{e}")


num_global = 0
def processing_signal(data, window_size, stride):
    global user_name, id_experiment, id_eeg_signal, num_global

    if user_name==None or id_experiment==None:
        print("Error, faltan los metadatos, llama set_metadata() antes")
        return False
    
    print("BASAL STATE:\n")

    if id_experiment.find("deap") >= 0:
        ini_signal = 128*3
        # ventaneo de la señal base
        basal = data[:32 , :ini_signal]
        basal, res_b = get_segments(basal, window_size, window_size)
        # promediamos la señal basal
        basal = np.mean(basal, axis=0)

         #(n_channel, n_sample) to (n_sample, n_channel)
        # data = data.transpose(1,0)

        # num segments a generar
        num_segments = (data.shape[1]-ini_signal)//window_size
        # datos eeg
        data = data[:32 , ini_signal: num_segments*window_size+ini_signal]
        
        # extracción de la señal base
        for i in range(num_segments):
            data[: , i*window_size : (i+1)*window_size] = data[: , i*window_size : (i+1)*window_size] - basal


    # carga se señales basales
    if id_experiment.find("protocol") >= 0:
        # ruta del ultimo experimento
        dir_last = f"{root_dir}/{user_name}/protocol/history/last_experiment.txt"
        last_exp = load_json(dir_last)
        name_histo = last_exp["history_basal"]

        # datos basales
        path = f"{root_dir}/{user_name}/protocol/history/{name_histo}"
        histo_basal = load_json(path)
        eval_basal = histo_basal[num_ind]
        # id basal 
        id_eeg_signal = eval_basal["id_eeg_signal"]

        path_basal = f"{root_dir}/{user_name}/protocol/protocol_basal/ind_{num_ind}"
        id_pareto = "protocol_basal"

        # # ruta de las señales        
        # path_basal = f"{root_dir}/{user_name}/protocol/protocol_basal/ind_{num_ind}/{id_s}.csv"
        # # carga de señales basales
        # basal = DataFilter.read_file(path_basal)
        # basal = np.array(basal)
        # basal = basal[1:17,:]
        
    # carga se señales basales
    if id_experiment.find("optimized") >= 0:
        dir_histo = f"{root_dir}/{user_name}/history/histo_basal_state_dict.txt"
        histo_basal = load_json(dir_histo)

        if len(histo_basal.keys()) > 0: 
            id_eeg_signal = histo_basal[id_experiment][f"ind_{num_ind}"]
            # id_eeg_signal = histo_basal["optimized-8"]["ind_22"]
        else:
            dir_histo = f"{root_dir}/{user_name}/history/histo_basal_state.txt"
            histo_basal = load_json(dir_histo)
            if len(histo_basal) == 0:
                print_error(f"Error al cargar el estado basal {user_name} {id_experiment} {num_ind}")
                return
            
            id_eeg_signal = histo_basal.pop()

        print(f"basal exp: {user_name}-{id_experiment}-{num_ind}-{id_eeg_signal}")
        path_basal = f"{root_dir}/{user_name}/optimized_basal"
        id_pareto = "optimized_basal"

        # # último estado basal
        # id_eeg_signal = histo_basal.pop()
        # path_basal = f"{root_dir}/{user_name}/optimized_basal"
        # id_pareto = "optimized_basal"

        # # ruta del ultimo experimento
        # dir_last = f"{root_dir}/{user_name}/protocol/history/last_experiment.txt"
        # last_exp = load_json(dir_last)
        # name_histo = last_exp["history_basal"]

        # # datos basales
        # path = f"{root_dir}/{user_name}/protocol/history/{name_histo}"
        # histo_basal = load_json(path)
        # eval_basal = histo_basal[num_global]
        # # id basal 
        # id_eeg_signal = eval_basal["id_eeg_signal"]

        # path_basal = f"{root_dir}/{user_name}/protocol/protocol_basal/ind_{num_global}"
        # id_pareto = "protocol_basal"

        # num_ind = num_global

        # print(len(histo_basal), num_global)
        # num_global += 1
        # if num_global >= len(histo_basal):
        #     num_global = 0
        
        # if user_name == "DABB" and num_global >= 15:
        #     num_global = 0
        


    if id_experiment.find("optimized") >= 0 or id_experiment.find("protocol") >= 0:
        # se preprocesan las señales basales
        # basal__ = preprocessing_signal_ros([], user_name, id_experiment, id_eeg_signal )
        dir_prepro = f"{path_basal}/{id_eeg_signal}_preprocessing.csv"
        preprocessing_signal_ros(user_name, id_pareto, num_ind, id_eeg_signal)

        print(id_eeg_signal, id_pareto)
        print(dir_prepro, "\n")
        if not os.path.isfile(dir_prepro):
            print_error(f"ERROR: No se encuentran los datos preprocesados {dir_prepro}")
            return

        basal = DataFilter.read_file(dir_prepro)
        basal = np.array(basal)
        basal = basal[1:17]

        # segmentos de la señal basal
        basal, res = get_segments(basal, window_size, window_size)
        
        # promediamos la señal base
        basal = np.mean(basal, axis=0)

        # num segments a generar
        num_segments = data.shape[1]//window_size

        # datos eeg
        data = data[:, : num_segments*window_size]

        # sample to channel, segmentación, se acomoda la forma de las ventanas
        data = data.transpose(1,0).reshape(num_segments, window_size, 16).transpose(0,2,1)

        # extracción de la señal base
        data = data - basal

        # se reconstruye la señal
        data = data.transpose(1,0,2).reshape(16, window_size*num_segments)

    #se extraen las ventanas: input shape=(n_channel,n_sample)
    data, res_s = get_segments(data, window_size, stride)
    
    # reseteo de los metados
    reset_metadata()

    return data

