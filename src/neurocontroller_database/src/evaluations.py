#!/usr/bin/env python3

import os, sys
from os.path import expanduser
home = expanduser("~")
sys.path.insert(2, f"{home}/catkin_ws/src/neurocontroller_database/src")
# from utilities import load_json, print_error, has_attr, root_dir, bci_user_list, append_data, get_users
from ROS import get_type_experiment
import rospy
from neurocontroller_database.srv import get_emotion_evaluations, get_emotion_evaluationsResponse, get_emotional_distributionsResponse
from neurocontroller_database.srv import get_type_id, get_emotional_distributions
from experiment_control_interfaces.srv import show_predictions
import numpy as np
import pandas as pd
# metricas de evaluación
from sklearn.metrics import precision_score
from sklearn.metrics import f1_score
from sklearn.metrics import accuracy_score
from sklearn.metrics import recall_score
from sklearn.metrics import roc_curve, auc
from sklearn.metrics import confusion_matrix as confusion_matrix_skl
from tensorflow.keras import utils as np_utils
import pandas as pd
import json
import sys, os
from os.path import expanduser
home = expanduser("~")
# sys.path.insert(2, f"{home}/catkin_ws/src/emotion_classification/src")
# sys.path.insert(3, f"{home}/catkin_ws/src/neurocontroller_database/src")
# sys.path.insert(4, f"{home}/catkin_ws/src/neurocontroller_database/database_results")
# print(f"{home}/catkin_ws/src/neurocontroller_database/database_results")
from utilities import load_json, save_json, print_error, get_emotions, has_attr
from distributions import get_distributions



# dirección raiz
root_dir = home+"/catkin_ws/src/neurocontroller_database/database"



hl_label = np.array(["alto", "bajo"])
m_label = np.array(["feliz", "enojado", "triste", "calmado"])

def get_max_emotion(dist):
    global hl_label, m_label

    m_d = np.argwhere(dist == np.amax(dist)).transpose(1,0)[0]

    if len(dist) == 2:
        return hl_label[m_d].tolist()
    if len(dist) == 4:
        return m_label[m_d].tolist()

    return []


def confusion_matrix(l_o, l_p):
    data = {'original': l_o, 'predic': l_p}
    df = pd.DataFrame(data, columns=['original', 'predic'])
    confusion_matrix = pd.crosstab(df['original'], df['predic'], rownames=[
                                   'Clase original'], colnames=['Clase predicha'], normalize="all")
    return confusion_matrix


def roc_curve_multiclass(y_true, y_score):
    # número de clases
    # n_classes = len(y_true.shape)
    n_classes = y_true.shape[1]

    fpr = dict()
    tpr = dict()
    roc_auc = dict()
    for i in range(n_classes):
        fpr[i], tpr[i], _ = roc_curve(y_true[:, i], y_score[:, i])
        roc_auc[i] = auc(fpr[i], tpr[i])

    # Compute micro-average ROC curve and ROC area
    fpr["micro"], tpr["micro"], _ = roc_curve(y_true.ravel(), y_score.ravel())
    roc_auc["micro"] = auc(fpr["micro"], tpr["micro"])

    # First aggregate all false positive rates
    all_fpr = np.unique(np.concatenate([fpr[i] for i in range(n_classes)]))

    # Then interpolate all ROC curves at this points
    mean_tpr = np.zeros_like(all_fpr)
    for i in range(n_classes):
        mean_tpr += np.interp(all_fpr, fpr[i], tpr[i])

    # Finally average it and compute AUC
    mean_tpr /= n_classes

    fpr["macro"] = all_fpr
    tpr["macro"] = mean_tpr
    roc_auc["macro"] = auc(fpr["macro"], tpr["macro"])

    for k in fpr:
        fpr[k] = fpr[k].tolist()
    for k in tpr:
        tpr[k] = tpr[k].tolist()
    dat = {"fpr": fpr, "tpr": tpr}

    return {"auc": roc_auc, "rates": dat}



def get_evaluations(y_true_, y_preds, type_eval="high-low"):
    keys = ["macro", "micro", "weighted", None]
    metric = ["f1_score", "recall", "precision"]
    # type_eval = "high-low"

    # se pasan las etiquetas en
    # alto-valence(HV)=1,
    # bajo-valence(LV)=0,
    # alto-arousal(HA)=1,
    # bajo-arousal(LA)=0,
    y_true = (y_true_ >= 0.5) + 0
    #n*2
    # Curva ROC
    # se usa one-hot encodding para calcular la curva para cada clase
    dat_roc = roc_curve_multiclass(y_true, y_preds)

    if type_eval != "high-low":
        y_true = get_emotions(y_true_)
        y_pred = get_emotions(y_preds)
    else:
        y_pred = (y_preds >= 0.5) + 0

    # número de clases
    num_classes = len(y_true.shape)

    # print("true ",y_true.shape," pred",y_pred.shape, " numclasses", num_classes)
    # print(np.unique(y_true), np.unique(y_pred))

    if num_classes == 2:
        labels = [0, 1]
    if num_classes == 1:
        labels = [0, 1, 2, 3]

    evals = {}
    # recall
    for k in keys:
        if k is not None:
            n_metric = f"recall-{k}"
        else:
            n_metric = f"recall"

        evals[n_metric] = recall_score(
            y_true=y_true, y_pred=y_pred, average=k, zero_division=0, labels=labels).tolist()

    # accuracy
    evals["accuracy"] = accuracy_score(y_true=y_true, y_pred=y_pred).tolist()

    # f1-score
    for k in keys:
        if k is not None:
            n_metric = f"f1_score-{k}"
        else:
            n_metric = f"f1_score"

        evals[n_metric] = f1_score(
            y_true=y_true, y_pred=y_pred, average=k, zero_division=0, labels=labels).tolist()

    # precisión
    for k in keys:
        if k is not None:
            n_metric = f"precision-{k}"
        else:
            n_metric = f"precision"

        evals[n_metric] = precision_score(
            y_true=y_true, y_pred=y_pred, average=k, zero_division=0, labels=labels).tolist()

    evaluations = {}
    evaluations["roc"] = dat_roc
    evaluations["metrics"] = evals

    cm_class = {}
    for n_class in range(num_classes):
        cm_d = {}
        # cm = confusion_matrix(y_true[n_class], y_pred[n_class])
        # cm_d["table"] = cm.to_numpy().tolist()
        # cm_d["columns"] = list(cm.columns)
        # cm_d["index"] = list(cm.index)
        if type_eval != "high-low":
            cm_d["table"] = confusion_matrix_skl(
                y_true, y_pred, labels=labels, normalize="all").tolist()
        else:
            cm_d["table"] = confusion_matrix_skl(
                y_true[:, n_class], y_pred[:, n_class], labels=labels, normalize="all").tolist()

        cm_d["columns"] = labels
        cm_d["index"] = labels
        cm_class[f"{n_class}"] = cm_d

    evaluations["confusion_matrix"] = cm_class

    return evaluations

def erase_member(dict_in, member):
    try:
        del dict_in[member]
    except:
        return

def update_reg(user_name, model_name, id_model, num_rep, num_fold):
    dir_regs = f"{root_dir}/{user_name}/emotion_model/evaluations_path.txt"
    reg = load_json(dir_regs)

    if len(reg.keys()) == 0:
        return True
    
    # model name
    dir_exist = f"{root_dir}/{user_name}/emotion_model/{model_name}"
    if not os.path.isdir(dir_exist):
        erase_member(reg, model_name)
        save_json(reg, dir_regs)
        return True
    
    # id model
    dir_exist += f"/{id_model}"
    if not os.path.isdir(dir_exist):
        erase_member(reg[model_name], id_model)
        save_json(reg, dir_regs)
        return True
    
    # prueba
    dir_exist += f"/k_{num_rep}_fold_{num_fold}"
    if not os.path.isdir(dir_exist):
        erase_member(reg[model_name][id_model], str(num_rep))
        save_json(reg, dir_regs)
        return True
    
    return False

def get_evals(req):
    print("--------------------------------------")
    print(req)

    user_name = req.user_name
    model_name = req.model_name
    id_emotion = req.id_emotion_model

    num_rep = req.num_rep
    num_fold = req.num_fold
    # multiclass, high-low
    type_class = req.type_evaluation

    if type_class != "multiclass" and type_class != "high-low":
        print_error("Tipo de evaluación inválida, permitidas: [multiclass, high-low]")
        return get_emotion_evaluationsResponse("{}")


    if update_reg(user_name, model_name, id_emotion, num_rep, num_fold):
        print_error(f"No existe el registro de {user_name}/{model_name}/{id_emotion}/k_{num_rep}_fold_{num_fold}")
        return get_emotion_evaluationsResponse("{}")
        # return False

    

    dir_ = f"{root_dir}/{user_name}/emotion_model/{model_name}/{id_emotion}/k_{num_rep}_fold_{num_fold}"

    # carga de predicciones
    pred_fold = load_json(f"{dir_}/predictions_fold.out")
    if len(pred_fold.keys()) == 0:
        pred_fold = load_json(f"{dir_}/predictions_fold.txt")
    print("pred_fold", pred_fold.keys())

    # evaluaciones
    evals_fold = get_evaluations(np.array(pred_fold["labels"]), np.array(pred_fold["predictions"]), type_class)
    print("evals_fold",evals_fold.keys())
    
    # carga de predicciones
    pred_test = load_json(f"{dir_}/predictions_test.out")
    if len(pred_test.keys()) == 0:
        pred_test = load_json(f"{dir_}/predictions_test.txt")
        

    if len(pred_test.keys()) > 0:
        # evaluaciones
        evals_test = get_evaluations(np.array(pred_test["labels"]), np.array(
            pred_test["predictions"]), type_class)

        # etiquetas únicas
        uni_fold = np.unique(pred_fold["labels"], axis=0)
        uni_test = np.unique(pred_test["labels"], axis=0)
        uni_all = np.unique(np.concatenate((uni_fold, uni_test)), axis=0)

        # predicciones
        predictions_test = pred_test["predictions"]

    else:
        uni_all = np.unique(pred_fold["labels"], axis=0)
        predictions_test = {}
        evals_test = {}

    all_evals = {"eval_fold": evals_fold, "eval_test": evals_test, "labels": uni_all.tolist()
    , "fold_predictions": pred_fold["predictions"], "test_predictions": predictions_test}

    str_evals = json.dumps(all_evals)

    return get_emotion_evaluationsResponse(str_evals)


def get_preds(req):
    evals = load_predictions(req)
    return get_emotion_evaluationsResponse(json.dumps(evals))


def load_predictions(param):
    user_name = param.user_name

    model_name = param.model_name
    id_emotion = param.id_emotion_model
    num_rep = param.num_rep
    num_fold = param.num_fold

    id_pareto = param.id_pareto_front
    id_ind = f"ind_{param.num_ind}"
    storage_pos = str(param.storage_pos)

    print("----------------")
    print("load_predictions", param)
    all_evals = {}
    #PREDICCIÓN DE UN NEUROCONTROLADOR DE LA BCI
    if id_pareto != "" and param.num_ind >= 0 and int(storage_pos) >= 0:
        type_id = get_type_experiment(id_pareto)
        dir_ = f"{root_dir}/{user_name}"
        
        if type_id.find("optimized") >= 0:

            # experimento a cargar
            pred_data = load_json(f"{dir_}/history/prediction_data.txt")
            if len(pred_data) == 0:
                pred_data = load_json(f"{dir_}/history/prediction_data.out")
            
            try:
                file_pred = pred_data[id_pareto][id_ind][storage_pos]

                # predicciones
                dir_ind = f"{dir_}/{type_id}/{id_pareto}/{id_ind}"
                pred_data = load_json(f"{dir_ind}/{file_pred}.txt")
                predictions = np.array(pred_data["predictions"])
            except Exception as e:
                return {}

            # centroide de las predicciones
            valence = np.sum(predictions[:, 0])/predictions.shape[0]
            arousal = np.sum(predictions[:, 1])/predictions.shape[0]
            y_pred = [valence, arousal]


            # se carga el historial
            data = load_json(f"{dir_}/history/emotion_evaluation.txt")

            # etiqueta
            try:
                print(id_pareto, id_ind, storage_pos)
                print(data[id_pareto][id_ind])
                emodata = data[id_pareto][id_ind][storage_pos]
                labels = [[emodata["valence"],  emodata["arousal"]]]
            except Exception as e:
                labels = []

            all_evals = {"labels": labels, "fold_predictions": predictions.tolist(), "test_predictions": {}, "centroide": [y_pred]}

            
        
    # PREDICCIÓN DE UN MODELO CON ENTRENAMIENTO SENCILLO 
    elif model_name != "" and id_emotion != "" and num_rep > 0 and num_fold > 0:

        dir_ = f"{root_dir}/{user_name}/emotion_model/{model_name}/{id_emotion}/k_{num_rep}_fold_{num_fold}"

        # carga de predicciones
        # path_fold = f"{dir_}/predictions_fold.out"
        # pred_fold = load_json(path_fold)

        # carga de predicciones
        pred_fold = load_json(f"{dir_}/predictions_fold.out")
        if len(pred_fold.keys()) == 0:
            pred_fold = load_json(f"{dir_}/predictions_fold.txt")

        # carga de predicciones
        pred_test = load_json(f"{dir_}/predictions_test.out")
        if len(pred_test.keys()) == 0:
            pred_test = load_json(f"{dir_}/predictions_test.txt")

        # etiquetas únicas
        if len(pred_test.keys()) > 0:
            uni_fold = np.unique(pred_fold["labels"], axis=0)
            uni_test = np.unique(pred_test["labels"], axis=0)
            uni_all = np.unique(np.concatenate((uni_fold, uni_test)), axis=0)

            predictions_test = pred_test["predictions"]
        else:
            uni_all = np.unique(pred_fold["labels"], axis=0)
            predictions_test = {}

        all_evals = {"labels": uni_all.tolist(), "fold_predictions": pred_fold["predictions"], "test_predictions": predictions_test}
    
        
    
    return all_evals


def get_dists(req):
    str_evals = get_emo_dist(req)
    return get_emotional_distributionsResponse(str_evals)

def get_emo_dist(param):
    user_name = param.user_name

    model_name = param.model_name
    id_emotion = param.id_emotion_model
    num_rep = param.num_rep
    num_fold = param.num_fold

    id_pareto = param.id_pareto_front
    id_ind = f"ind_{param.num_ind}"
    storage_pos = param.storage_pos

    if id_pareto != "" and param.num_ind >= 0 and storage_pos >= 0:
        
        storage_pos = str(storage_pos)

        data_adq = load_json(f"{root_dir}/{user_name}/history/prediction_data.txt")
        
        # id de las predicciones a cargar
        id_pred = data_adq[id_pareto][id_ind][storage_pos]

        # archivo de las predicciones
        dir_pred = f"{root_dir}/{user_name}/data_optimized_individuals/{id_pareto}/{id_ind}/{id_pred}"
        print(dir_pred)
        pred_data = load_json(dir_pred)
        y_pred = np.array(pred_data["predictions"])
        
        # predicción
        # centroide = np.array(pred_data["centroide"])
        v = np.sum(y_pred[:,0]) / y_pred.shape[0]
        a = np.sum(y_pred[:,1]) / y_pred.shape[0]
        centroide = [v,a]

        # distribuciones de clase
        valence, arousal, emo_class = get_distributions(y_pred)
        # emoción predicha
        valence["max_emotion"] = get_max_emotion(valence["distribution"])
        arousal["max_emotion"] = get_max_emotion(arousal["distribution"])
        emo_class["max_emotion"] = get_max_emotion(emo_class["distribution"])

        
        dist_data = {"valence":valence, "arousal":arousal, "multiclass":emo_class}
        print(dist_data)
        return json.dumps(dist_data)
        



# count = 0
# def plot_evals(req):
#     global count

#     user_name = req.user_name
#     id_pareto = req.id_emotion_model
#     num_ind = req.num_rep*req.num_fold
    

#     count += 1
#     store_pos = count 

#     evals = load_predictions(req, "object")
#     labels = evals["labels"]
#     # shape = .shape
#     # se 
#     # se muestran en la gráfica
#     try:
#         srv = rospy.ServiceProxy("show_predictions", show_predictions)
        
#         srv(user_name, id_pareto, num_ind, store_pos )
#     except rospy.ServiceException as e:
#         print_error(f"ERROR: No se pudo plotear las predicciones\n{e}")

if __name__ == "__main__":

    rospy.init_node("evaluations")
    print("Nodo evaluations iniciado con éxito")

    # servicio para cargar las prediciones en la gráfica
    # srv_plot_pred = rospy.Service("plot_evals_predictions", plot_evals_predictions, plot_evals)

    # servicios
    srv_get_eval = rospy.Service("get_emotion_evaluations", get_emotion_evaluations, get_evals)
    srv_get_preds = rospy.Service("get_emotion_predictions", get_emotion_evaluations, get_preds)
    srv_get_dist = rospy.Service("get_emotional_distributions", get_emotional_distributions, get_dists)

    rospy.spin()
