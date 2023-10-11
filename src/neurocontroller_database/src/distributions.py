import numpy as np
import matplotlib.pyplot as plt
import sys, os
from os.path import expanduser
home = expanduser("~")
# sys.path.insert(2, f"{home}/catkin_ws/src/neurocontroller_database/src")
from utilities import get_emotions

plt.rcParams.update({'font.size': 15})



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

def distribution(list_elem, num_classes):
    dist_data = np.unique(list_elem, return_counts=True)
    dist = dist_data[1]/np.sum(dist_data[1])

    if dist.shape[0] > num_classes:
        print("Se calcularon mas clases que las esperadas")
        return

    values = dist_data[0].tolist()
    count = dist.tolist()

    # por si no hay elementos de una clase
    dist_aux = np.zeros(num_classes, dtype=float)
    for n_class in range(len(values)):
        n_val = values[n_class]
        dist_aux[n_val] = count[n_class]

    return {"values":np.arange(num_classes).astype(str).tolist(), "distribution":dist_aux.tolist()}

def get_distributions(labels):
    labels = (labels >= 0.5) + 0

    valence = distribution(labels[:, 0], 2)
    arousal = distribution(labels[:, 1], 2)
    emo_class = distribution(get_emotions(labels), 4)

    return valence, arousal, emo_class

def get_values(classes,values, show_vals=False):
    if show_vals:
        vals = [float(f"{v:.02f}") for v in values]
    else:
        vals = ["" for i in range(len(values))]

    if len(classes) == 4:
        return [f"feliz\n{vals[0]}", f"enojado\n{vals[1]}", f"triste\n{vals[2]}", f"calmado\n{vals[3]}"]
    if len(classes) == 2:
        return [f"bajo\n{vals[0]}", f"alto\n{vals[1]}"]
    


def plot_bar(graph, dist, title, range_lim=None, show_vals=False):
    graph.bar(get_values(dist["values"], dist["distribution"], show_vals) , dist["distribution"])
    graph.set(xlabel="clases", ylabel="porcentaje")
    graph.set_title(title)

    if range_lim is not None:
        graph.set_ylim(range_lim)


def plot_distributions(user_name, dist_valence, dist_arousal, dist_class, path, range_lim=None, show_vals=False):
    print("dist_valence", dist_valence)
    print("dist_arousal", dist_arousal)
    print("dist_class", dist_class)
    
    fig, ax = plt.subplots (1,3, figsize=(17,6))
    plot_bar(ax[0],dist_valence, f"{user_name}: valence", range_lim, show_vals)
    plot_bar(ax[1],dist_arousal, f"{user_name}: arousal", range_lim, show_vals)
    plot_bar(ax[2],dist_class, f"{user_name}: multiclase", range_lim, show_vals)

    # si no existe la dir destino
    if not os.path.isdir(path):
        os.makedirs(path)

    fig.savefig(f"{path}/{user_name}_dist.pdf")
    plt.show()