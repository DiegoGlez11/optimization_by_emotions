import os
import sys
import json
import rospy
import shutil
import plotly.express as px
import plotly.graph_objects as go
import pandas as pd
from neurocontroller_database.srv import get_emotion_evaluations

from os.path import expanduser
home = expanduser("~")
sep = os.sep
sys.path.insert(2, f"{home}{sep}catkin_ws{sep}src{sep}neurocontroller_database{sep}src")
from utilities import load_json, save_json, users_list, print_error
import matplotlib.pyplot as plt
import numpy as np

root_dir = f"{home}/catkin_ws/src/neurocontroller_database/database"
models = ["DeepConvNet", "EEGNet", "ShallowConvNet"]

config = {
  'toImageButtonOptions': {
    'format': 'svg', # one of png, svg, jpeg, webp
    # 'scale': 1 # Multiply title/legend/axis/canvas sizes by this factor
  }
}

def has_attr(obj, attr):
    try:
        obj[attr]
        return True
    except:
        return False

def get_users(type_users):

    if type_users == "deap":
        user_eval = [f"s{i+1:02d}" for i in range(32)]
    else:
        # user_list = os.listdir(f"{root_dir}")
        # user_list = [user for user in user_list if user.find("deap")<0 and user.find("optimized")<0 and os.path.isdir(f"{root_dir}/{user}")]
        
        user_eval = []
        for user in users_list:
            path = f"{root_dir}/{user}/emotion_model/evaluations_path.txt"
            if os.path.isfile(path):
                user_eval.append(user)

    return user_eval

def get_evaluations(user_name, id_emotion, model_name, num_rep, num_fold, type_multiclass):
    try:
        srv = rospy.ServiceProxy("get_emotion_evaluations", get_emotion_evaluations)
        str_eval = srv(user_name, model_name, id_emotion, num_rep, num_fold, "", 0, 0, type_multiclass)

        if str_eval == "{}":
            print_error(f"No hay datos de evaluación: {user_name}/{model_name}/{id_emotion}/k_{num_rep}_fold_{num_fold}")
            return
        
        return json.loads(str_eval.evaluations)
    except rospy.ServiceException as e:
        print("Error al obtener las evaluaciones\n", e)
        return

        
def get_metrics(file, type_evals, users, type_multiclass):
    # valores a obtener
    val_metrics = {"usuario":[], "modelo":[]}

    # etiquetas de las métricas
    if type_evals == "regression":
        metrics_name = ["loss","mse","mae","mape"]  
    if type_evals == "multiclass":
        # metrics: {f1_score, recall, precision}-{micro,macro, weighted}, accuracy
        metrics_name = ["accuracy", "recall-macro", "precision-macro", "f1_score-macro"]

    # metricas según el tipo 
    for m in metrics_name:
        val_metrics[m] = []
    
    
    for user_name in users:
        dir_user = f"{root_dir}/{user_name}"

        # si no existe el user
        if not os.path.isdir(dir_user):
            continue
        
        dir_evals = f"{root_dir}/{user_name}/emotion_model/evaluations_path.txt";
        evaluations = load_json(dir_evals)
        if len(evaluations) == 0:
            print(f"No se puede cargar la ruta de los modelos: {user_name}")
            continue
        
        # nombre del modelo
        for model_name in evaluations.keys():
            dir_model = f"{dir_user}/emotion_model/{model_name}"

            # si no existe el modelo
            if not os.path.isdir(dir_model):
                print(f"Sin modelo {user_name} {model_name}")
                # del evaluations[model_name]
                # save_json(evaluations, dir_evals)
                continue

            # id de las diferentes instancias del modelo
            for id_model in evaluations[model_name].keys():
                # repetición
                for num_rep in evaluations[model_name][id_model].keys():
                    # fold
                    for num_fold in evaluations[model_name][id_model][num_rep].keys():
                        
                        if type_evals == "regression":
                            path = evaluations[model_name][id_model][num_rep][num_fold]
                            dir_pred = f"{path}/{file}.txt"
                            
                            # no existen las predicciones
                            if not os.path.isfile(dir_pred):
                                print(f"Sin predicciones: {user_name} {model_name} {id_model} {num_rep} {num_fold}")
                                # del evaluations[model_name][id_model][num_rep][num_fold]
                                # save_json(evaluations, dir_evals)
                                continue

                            eval = load_json(dir_pred)
                            if len(eval) == 0:
                                print("Error al cargar la evaluación")
                            eval = eval["evaluations"]

                        if type_evals == "multiclass":
                            # metricas multiclase
                            eval= get_evaluations(user_name, model_name, id_model, int(num_rep), int(num_fold), type_multiclass)
                            if eval is None:
                                return
                            
                            # evaluaciones: eval_fold, eval_test
                            eval = eval[file]["metrics"]
                            
                        # se guardan las metricas
                        for m in metrics_name:
                            val_metrics[m].append(eval[m])

                        val_metrics["usuario"].append(user_name)
                        val_metrics["modelo"].append(model_name)
    
    
    return pd.DataFrame(val_metrics)



def plot_graphs(evals, dir_out, type_evals, type_multiclass, y_range_bar=None):
    str_multi = ""
    if type_evals == "multiclass":
        str_multi = f"_{type_multiclass}"
        dir_out += f"/{type_multiclass}"


    if os.path.isdir(dir_out):
        shutil.rmtree(dir_out) #, ignore_errors=True)

    os.makedirs(dir_out)

    # etiquetas de las métricas
    if type_evals == "regression":
        metrics_selected3D = ["mse","mae","mape"]
        metrics_selected = ["mse", "mae"]
    if type_evals == "multiclass":
        # metrics: {f1_score, recall, precision}-{micro,macro, weighted}, accuracy
        # metrics_selected3D = ["accuracy", "recall-macro", "precision-macro", "f1_score-macro"]
        metrics_selected = ["accuracy", "recall-macro", "precision-macro"]
        metrics_selected3D = metrics_selected
    
    # BOX PLOT
    for m in metrics_selected:
        fig = px.box(evals, x="modelo", y=m)

        # # hide subplot y-axis titles and x-axis titles
        # for axis in fig.layout:
        #     if type(fig.layout[axis]) == go.layout.YAxis:
        #         fig.layout[axis].title.text = ''
        #     if type(fig.layout[axis]) == go.layout.XAxis:
        #         fig.layout[axis].title.text = ''
        # # ensure that each chart has its own y range and tick labels
        # fig.update_yaxes(matches=None, showticklabels=True, visible=True)
        # fig.update_layout(font=dict(size=5))

        fig.write_image(f"{dir_out}/box_{type_evals}_{m}{str_multi}.pdf")
        fig.show(config=config)

    # promedio de clases
    prom_data = {"modelo":[], "métrica":[], "promedio":[]} 
    for model in models:
        prom_eval = evals.loc[evals["modelo"] == model][metrics_selected].mean(axis=0)

        for m in metrics_selected:
            prom_data["promedio"].append(prom_eval[m])
            prom_data["métrica"].append(m)
            prom_data["modelo"].append(model)

    fig = px.bar(pd.DataFrame(prom_data), x="métrica", y="promedio", color="modelo", barmode="group")
    if y_range_bar is not None: fig.update_layout(yaxis_range=y_range_bar)
    # fig.update_layout(title=model, xaxis=dict(title="métricas"), yaxis=dict(title="promedio"))
    fig.write_image(f"{dir_out}/prom_{type_evals}_prom{str_multi}.pdf")
    fig.show(config=config)

    # graficas 3D

    # usuario
    fig = px.scatter_3d(evals, x=metrics_selected3D[0], y=metrics_selected3D[1], z=metrics_selected3D[2], color="usuario")
    fig.update_traces(marker_size=10, marker=dict(size=4, line=dict(width=1, color="black")))
    fig.show(config=config)

    # modelo
    fig = px.scatter_3d(evals, x=metrics_selected3D[0], y=metrics_selected3D[1], z=metrics_selected3D[2], color="modelo")
    fig.update_traces(marker_size=10, marker=dict(size=4, line=dict(width=1, color="black")))
    fig.show(config=config)

    if type_evals == "regression":
        fig = px.scatter(evals, x=metrics_selected[0], y=metrics_selected[1], color="modelo")
        fig.update_traces(marker_size=10, marker=dict(size=4, line=dict(width=1, color="black")))
        # fig.update_layout(title="Métricas de evalución")#, yaxis_zeroline=False, xaxis_zeroline=True)
        fig.update_traces(marker=dict(size=12))
        # fig.update_xaxes(zeroline=True, zerolinewidth=1, zerolinecolor='black')
        # fig.update_yaxes(zeroline=True, zerolinewidth=1, zerolinecolor='black')
        fig.write_image(f"{dir_out}/metrics_{type_evals}_2d.pdf")
        fig.show(config=config)

def make_evaluations(eval, type_eval, dataset, type_multiclass="", y_range_bar=None):
    if type_eval != "regression" and type_eval != "multiclass":
        print("Error no se puede hacer la prueba", type_eval)
        return
    if type_eval == "multiclass":
        if type_multiclass != "multiclass" and type_multiclass != "high-low":
            print_error("Tipo de evaluación inválida, permitidas: [multiclass, high-low]")
            return

    if eval == "test":
        str_eval = "eval_test"
    else:
        if eval == "fold":
            str_eval = "eval_fold"
        else:
            print("Error, no existe la prueba", eval)

    if dataset != "deap" and dataset != "protocol":
        print("No existen evaluciones en el dataset ", dataset)


    users = get_users(dataset)
    

    dir_ = f"./evaluations/{dataset}"
    if dataset == "deap":
        dir_ += f"/{eval}"    
    dir_ += f"/{type_eval}"


    evals = get_metrics(str_eval, type_eval, users, type_multiclass)
    if evals is None:
        return
    plot_graphs(evals, dir_, type_eval, type_multiclass, y_range_bar)