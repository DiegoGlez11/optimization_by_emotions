
import os, sys
from os.path import expanduser
home = expanduser("~")
sys.path.insert(2, f"{home}/catkin_ws/src/neurocontroller_database/src")
from utilities import load_json, print_error, has_attr, root_dir, bci_user_list, append_data, get_users
from PBI import get_pbi
from ROS import load_object_space_ros
import numpy as np
import pandas as pd


FILES_NAME = ["emotional_roadmap", "adquisition_data", "emotion_evaluation", "prediction_data"]


def open_file(user_name, file_name):

    dir_f = f"{root_dir}/{user_name}/history/{file_name}"

    f = load_json(f"{dir_f}.txt")

    if type(f) == list:
        return f
    if len(f.keys()) == 0:
        f = load_json(f"{dir_f}.out")

        if type(f) == list:
            return f
        if len(f.keys()) == 0:
            return

    return f

def read_value(reg, id_pareto, id_ind, store_pos):
    if not has_attr(reg, id_pareto):
        return True, f"No se encuentra el registro del FRENTE {id_pareto}"
    if not has_attr(reg[id_pareto], id_ind):
        return True, f"No se encuentra el registro del INDIVIDUO {id_ind}"
    
    positions = reg[id_pareto][id_ind] 
    if type(positions) == list:
        if not has_attr(positions, int(store_pos)):
            return True, f"No se encuentra el EXPERIMENTO, store_pos = {store_pos}"
        dat = positions[int(store_pos)]
    elif type(positions) == dict:
        if not has_attr(positions, str(store_pos)):
            return True, f"No se encuentra el EXPERIMENTO, store_pos = {store_pos}"
        dat = positions[str(store_pos)]
    else:
        return True, f"No se encuentra el EXPERIMENTO, store_pos = {store_pos}"

    return False, dat
        


def load_experiment(users):

    
    for user_name in users:
        print("--",user_name)

        # dir del usuario
        dir_user = f"{root_dir}/{user_name}"
        dir_histo = f"{dir_user}/history"

        files_data = {}
        err_file = ""
        # se carga el registro de los experimentos
        exp_record = load_json(f"{dir_histo}/experiment_record.txt")
        if len(exp_record) == 0:
            print_error("Sin registro de experimentos")
            continue
        # se buscan experimentos válidos
        exp_reg = {}
        for exp in exp_record["list"]:

            # el experimento se terminó por uno de los botones
            if has_attr(exp, "type_end"):
                # tipo de experimento
                t = exp["type"]
                
                # deben exisitir los archivos pertinentes
                if t == "sam" and not has_attr(files_data, "emotion_evaluation") :
                    emo_file = open_file(user_name, "emotion_evaluation")
                    if emo_file is None:
                        err_file += f"No existen las evaluaciones emocionales. Tipo de experimento: {type_exp}\n"
                    else:
                        files_data["emotion_evaluation"] = emo_file

                if t == "bci" and not has_attr(files_data, "adquisition_data") and not has_attr(files_data, "prediction_data"):
                    adq_file = open_file(user_name, "adquisition_data")
                    pred_file = open_file(user_name, "prediction_data")
                    if adq_file is None or pred_file is None:
                        err_file += "Datos incompletos de la BCI. Tipo de experimento: {type_exp}\n"
                    else:
                        files_data["adquisition_data"] = adq_file
                        files_data["prediction_data"] = pred_file

                # experimento correcto
                if err_file == "":
                    exp_reg[exp["storage_position"]] = exp
                else:
                    print_error(err_file)
                    break
        
        # sin experimentos
        if len(exp_reg) == 0:
            print(f"No hay experimentos para el usuario {user_name}")
            continue
        # No existen los archivos para los experimentos
        if err_file != "":
            continue
        

        # se carga el roadmap
        roadmap = open_file(user_name, "emotional_roadmap")
        if(roadmap is None):
            print_error(f"El usuario: {user_name} no tiene roadmap")
            continue
        
        # para cada experimento del roadmap
        for n_exp in range(len(roadmap)):
            ind_sel = roadmap[n_exp]
            
            # si son individuos seleccionados de experimentos con finalización sin botón de término
            if not has_attr(exp_reg, ind_sel["storage_position"]):
                continue

            # datos del experimento
            id_pareto = ind_sel["id_pareto_front"]
            num_ind = ind_sel['num_ind']
            id_ind = f"ind_{num_ind}"
            store_pos = ind_sel["storage_position"]
            # tipo de exp al que pertenece
            type_exp = exp_reg[store_pos]["type"]


            if type_exp == "bci":
                # id de la señal
                try:
                    err, value = read_value(files_data["adquisition_data"], id_pareto, id_ind, store_pos)
                    if err:
                        raise Exception(f"{value}. Datos experimento {ind_sel}")
                    
                    id_signal = value["id_eeg_signal"]

                    # no EEG simulado
                    if id_signal.find("synthetic") >= 0:
                        raise Exception(f"Archivo sintético {ind_sel}  ID signal: {id_signal}")
                    
                    # existencia del archivo EEG
                    if not os.path.isfile(f"{dir_user}/data_optimized_individuals/{id_pareto}/{id_ind}/{id_signal}.csv"):
                        raise Exception(f"No hay archivo EEG {ind_sel} ID signal: {id_signal}")
                    
                    # registro del ID
                    ind_sel["id_signal"] = id_signal

                except Exception as e:
                    print_error(f"ID_SIGNAL_EEG: {e}")
                    continue


                # id de las predicciones
                try:
                    # id_pred = files_data["prediction_data"][id_pareto][id_ind][str(store_pos)]
                    err, value = read_value(files_data["prediction_data"], id_pareto, id_ind, store_pos)
                    if err:
                        raise Exception(f"{value}. Datos experimento {ind_sel}")
                    
                    # registro del ID
                    ind_sel["id_pred"] = value.split(".txt")[0]
                except Exception as e:
                    print_error(f"ID_PREDICTION: {e}")
                    continue
            else:
                ind_sel["id_signal"] = ""
                ind_sel["id_pred"] = ""


            if type_exp == "sam" or type_exp == "bci":
                # autoevaluación del usuario con SAM
                try:
                    err, emo_eval = read_value(files_data["emotion_evaluation"], id_pareto, id_ind, store_pos)
                    if err:
                        raise Exception(f"{emo_eval}. Datos experimento {ind_sel}")
                    
                    # registro de la emoción
                    ind_sel["emo_eval"] = [emo_eval["valence"], emo_eval["arousal"]]
                except Exception as e:
                    print_error(f"EMOTION EVALUATION: {e}")
                    continue
            else:
                ind_sel["emo_eval"] = [0,0]


            # siguiente frente en el roadmap
            next_front = ""
            if n_exp + 1 < len(roadmap):
                next_front = roadmap[n_exp+1]["id_pareto_front"]
            else: 
                if n_exp + 1 >= len(roadmap) -1:
                    next_front = ""

            # bandera para indicar que se terminan los ind del experimento
            is_end_exp = False
            if id_pareto != next_front:
                is_end_exp = True

            ind_sel["user_name"] = user_name
            ind_sel["type_experiment"] = type_exp

            yield ind_sel, files_data, is_end_exp


        # indica si se genera un error en la carga de un historial
        # is_error = False

        # # carga de datos
        # size_f = len(histo_list)
        # # files_data = [0 for i in range(size_f)]
        # files_data = {}
        # for i in range(size_f):
        #     # nombre del historial
        #     name = histo_list[i]
        #     # se carga el historial
        #     d = open_file(user_name, name)
        #     if d is None:
        #         is_error = True
        #         if show_msg: print(f"{user_name}\nNo tiene el archivo {name}")
        #         break
        #     # se almacenan los datos
        #     files_data[name] = d

        # if is_error:
        #     continue

        
        # # para cada experimento del roadmap
        # for n_exp in range(len(files_data["emotional_roadmap"])):
        #     ind_sel = files_data["emotional_roadmap"][n_exp]

        #     # final del usuario
        #     is_end_user = False
        #     if n_exp == len(files_data["emotional_roadmap"]) - 1:
        #         is_end_user = True

        #     yield user_name, ind_sel, files_data, is_change_front, is_end_user




def get_data_experiments(type_data="front", do_pbi=True, users=None, type_exp="sam", is_obj_space=False):
    data_exp = {}
    count_exp = 0
    count_front = 0

    y_true_front = np.empty((0,2))
    y_pred_front = np.empty((0,2))
    num_inds = []
    users_name = []
    nums_exps = []
    ids_pareto = []
    type_experiments = []
    storage_position = []


    # usuario a utilizar
    if users is not None:
        # una lista de usuarios definida por el parámetro de la función
        users_adq = users
    else:
        users_adq = get_users()
    

    actual_user = ""
    # se carga la info del experimento
    experiments = load_experiment(users_adq)

    for (exp_reg, files, is_last_exp) in experiments:

        # usuario del exp
        user_name = exp_reg["user_name"]

        # cambio de usuario
        if actual_user != user_name:
            actual_user = user_name
            count_exp = 0
            count_front = 0

        count_exp += 1
        # id del frente visitado
        id_pareto = exp_reg["id_pareto_front"]
        # tipo de experimento
        type_exp = exp_reg["type_experiment"]

        
        # tipo de operaciones a realizar
        do_sam = False
        do_pred = False
        if type_exp == "sam" or type_exp == "bci":
            do_sam = True
        if type_exp == "bci":
            do_pred = True

        # autoevaluación del usuario
        # if do_sam:
        y_true = exp_reg["emo_eval"]
        y_true_front = np.vstack((y_true_front, y_true))

        # se cargan las predicciones
        if do_pred:
            dir_pred = f"{root_dir}/{user_name}/data_optimized_individuals/{id_pareto}/ind_{exp_reg['num_ind']}/{exp_reg['id_pred']}.txt"
            pred = load_json(dir_pred)
            predictions = np.array(pred["predictions"])
            
            # centroide
            v = np.sum(predictions[:,0]) / predictions.shape[0]
            a = np.sum(predictions[:,1]) / predictions.shape[0]
            y_pred = [v,a]
            y_pred_front = np.vstack((y_pred_front, y_pred))

        # datos del experimento
        num_inds.append(exp_reg['num_ind'])
        type_experiments.append(type_exp)
        users_name.append(user_name)
        nums_exps.append(count_exp)
        ids_pareto.append(id_pareto)
        storage_position.append(exp_reg["storage_position"])

        # el último ind del frente
        if is_last_exp:
            count_front += 1

            if (np.unique(users_name).shape[0] > 1):
                print_error(users_name, "MUCHOS USERS. ESTE MSG NO DEBE APARECER")

            if do_pbi:
                # PBI SAM POR EXPERIMENTO SAM
                # if do_sam:
                max_pbi_true, pos_true, vals_true_pbi = get_pbi(y_true_front)
                # pos_true = int(pos_true)

                # PBI pred POR EXPERIMENTO CNN
                if do_pred:
                    max_pbi_pred, pos_pred, vals_pred_pbi = get_pbi(y_pred_front)
            

            if type_data == "individuo":
                append_data(data_exp, "id pareto", ids_pareto, "concatenate")
                append_data(data_exp, "num ind", num_inds, "concatenate")
                append_data(data_exp, "num exp", nums_exps, "concatenate")
                append_data(data_exp, "usuario", users_name, "concatenate")
                append_data(data_exp, "type experiment", type_experiments, "concatenate")
                append_data(data_exp, "id experiment", storage_position, "concatenate")

                if do_pred:
                    append_data(data_exp, "valence", y_pred_front[:,0], "concatenate")
                    append_data(data_exp, "arousal", y_pred_front[:,1], "concatenate")
                if do_pbi:
                    # if do_sam:
                    append_data(data_exp, "PBI SAM", vals_true_pbi, "concatenate")
                    if do_pred:
                        append_data(data_exp, "PBI", vals_pred_pbi, "concatenate")
                # if do_sam:
                append_data(data_exp, "valence SAM", y_true_front[:,0], "concatenate")
                append_data(data_exp, "arousal SAM", y_true_front[:,1], "concatenate")
                
                # valores en el espacio de los objetivos
                if is_obj_space:
                    try:
                        obj_s = load_object_space_ros(id_pareto, num_inds)
                        append_data(data_exp, "distancia", obj_s[:,0], "concatenate")
                        append_data(data_exp, "riesgo", obj_s[:,1], "concatenate")
                        append_data(data_exp, "velocidad de llegada", obj_s[:,2], "concatenate")

                    except Exception as e:
                        print_error(f"No se pueden obtener los valores de los objetivos: {e}")


            elif type_data == "front":
                
                if do_pbi:
                    # todo el array posee la misma info, se puede seleccionar cualquiera
                    append_data(data_exp, "id pareto", ids_pareto[0])
                    append_data(data_exp, "iteración", count_front)
                    append_data(data_exp, "usuario", users_name[0])
                    append_data(data_exp, "type experiment", type_experiments[0])
                    append_data(data_exp, "id experiment", storage_position[0])

                    # if do_sam:
                    append_data(data_exp, "num ind SAM", num_inds[pos_true])
                    append_data(data_exp, "PBI SAM", max_pbi_true)
                    append_data(data_exp, "valence SAM", y_true_front[pos_true,0])
                    append_data(data_exp, "arousal SAM", y_true_front[pos_true,1])

                    if do_pred:
                        append_data(data_exp, "num ind", num_inds[pos_pred])
                        append_data(data_exp, "PBI", max_pbi_pred)
                        append_data(data_exp, "valence", y_pred_front[pos_pred,0])
                        append_data(data_exp, "arousal", y_pred_front[pos_pred,1])


                    # valores en el espacio de los objetivos
                    if is_obj_space:
                        try:
                            obj_s = load_object_space_ros(id_pareto, num_inds)
                            
                            if do_pred:
                                append_data(data_exp, "distancia", obj_s[pos_pred,0])
                                append_data(data_exp, "riesgo", obj_s[pos_pred,1])
                                append_data(data_exp, "velocidad de llegada", obj_s[pos_pred,2])
                            # if do_sam:
                            append_data(data_exp, "distancia SAM", obj_s[pos_true,0])
                            append_data(data_exp, "riesgo SAM", obj_s[pos_true,1])
                            append_data(data_exp, "velocidad de llegada SAM", obj_s[pos_true,2])
                            
                        except Exception as e:
                            print_error(f"front {id_pareto}: No se pueden obtener los valores de los objetivos {e}")

            y_true_front = np.empty((0,2))
            y_pred_front = np.empty((0,2))
            num_inds = []
            nums_exps = []
            users_name = []
            ids_pareto = []
            type_experiments = []
            storage_position = []
            
        # if is_end_user:
        #     count_exp = 0
        #     count_front = 0
            
    print("Num Columnas",len(data_exp.keys()))
    print([(f"{a}",f"SIZE: {len(data_exp[a])}") for a in data_exp])

    data_exp = pd.DataFrame(data_exp)

    return data_exp


