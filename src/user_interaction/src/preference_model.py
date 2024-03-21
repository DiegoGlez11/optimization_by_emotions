#!/usr/bin/env python3

import sys
from os.path import expanduser
home = expanduser("~")
sys.path.insert(1, f"{home}/catkin_ws/src/neurocontroller_database/src")
import copy
from emotion_classification.msg import emotional_prediction
from user_interaction.srv import control_preference, control_preferenceResponse, experiment_control
from user_interaction.msg import next_front_ind
from user_interaction.srv import search_pareto_front, search_pareto_frontResponse
from user_interaction.srv import get_distributed_individuals, get_distributed_individualsResponse
from experiment_control_interfaces.srv import get_user
from neurocontroller_database.srv import load_obj_space
import numpy as np
import rospy
import random
from operator import itemgetter
from utilities import save_json, load_json, print_color, bcolors, has_attr, print_error
from PBI import PBI
from ROS import load_object_space_ros
from distributed_individuals import select_inds


# dirección raiz
root_dir = home+"/catkin_ws/src/neurocontroller_database/database"

# num de individuos por frente
num_individuals_by_front = None
# frentes de umbral de búsqueda
search_threshold_front = None

# datos de cada frente
DATA_FRONT = {}

# comportamiento de las preferencias
ARRAY_CONTROL = [8,8,7,7,6,4]
aux_init_control = ARRAY_CONTROL
ARRAY_CONTROL = []


# registro de frentes cargados
METADATA_REG = {}

# id de los frentes cargados
SUBTABLE = []
# rango de los frentes cargado
RANGE_LOADED = {}

# normalization
NORMALIZATION = True


# datos emocionales entrantes
emotion_values = np.empty((0, 2))
num_inds = []
ID_PARETO_INDS = []
# datos de los individuos de todos los frentes
all_fronts = None



# cuantos frentes se cargan para realizar la búsqueda
# si se supera el número de frentes, se usa el valor para cargar los siguientes
size_loaded_fronts = None

# tipo de modelo de preferencias: euclidean | pbi
type_model_pref = "pbi"

# datos del frente actual
FRONT_DATA = {"id_pareto":"", "storage_pos":"", "is_capturing":False}

def save_param(param):
    path_ = f"{root_dir}/control_params.txt"
    # load params
    params = load_json(path_)

    for k in param:
        val = param[k]

        if k == "num_individuals_by_front" or k == "search_threshold_front":
            t = type(val)
            if not t == int:
                raise f"Tipo inválido para el parámetro[{k}=={t}]"

        params[k] = val

    # save param
    save_json(params,path_)

# control preference
def control_pref(req):
    global emotion_values, num_inds, ARRAY_CONTROL
    global num_individuals_by_front, search_threshold_front, size_loaded_fronts

    if req.num_individuals_by_front > 0:
        num_individuals_by_front = req.num_individuals_by_front
        save_param({"num_individuals_by_front": num_individuals_by_front})
        print_color(f"num_individuals_by_front {num_individuals_by_front}", bcolors.OKGREEN)

    if req.search_threshold_front > 0:
        search_threshold_front = req.search_threshold_front
        save_param({"search_threshold_front":search_threshold_front})
        print_color(f"search_threshold_front {search_threshold_front}", bcolors.OKGREEN)

    if req.reset_table and req.user_name != "":
        print_color(f"reset table", bcolors.OKGREEN)
        load_subtable(req.user_name, start=True)
        emotion_values = np.empty((0, 2))
        num_inds = []

    
    # cuando se carga un frente en la GUI se debe cargar su subtabla de frentes
    if req.id_pareto_front != "" and req.user_name != "":
        print_color("load subtable", bcolors.OKGREEN)
        load_subtable(req.user_name, id_pareto=req.id_pareto_front)

    if req.size_loaded_fronts > 0:
        size_loaded_fronts = req.size_loaded_fronts
        print_color(f"size_loaded_fronts: {size_loaded_fronts}", bcolors.OKGREEN)

    if req.control == "extend_search":
        ARRAY_CONTROL = aux_init_control.copy()
        print_color(f"ARRAY_CONTROL: {ARRAY_CONTROL}", bcolors.OKGREEN)
    
    if req.control == "non_extend_search":
        ARRAY_CONTROL = []
        print_color(f"ARRAY_CONTROL: {ARRAY_CONTROL}", bcolors.OKGREEN)

    if req.control == "non_extend_search":
        ARRAY_CONTROL = []
        print_color(f"ARRAY_CONTROL: {ARRAY_CONTROL}", bcolors.OKGREEN)

    if req.control == "do_norm":
        NORMALIZATION = True
        print_color(f"normalization: {NORMALIZATION}", bcolors.OKGREEN)
    
    if req.control == "deactiv_norm":
        NORMALIZATION = False
        print_color(f"deactive_normalization: {NORMALIZATION}", bcolors.OKGREEN)

    return control_preferenceResponse()



def load_subtable(user_name, ini=None, start=False, id_pareto=None):
    global size_loaded_fronts, RANGE_LOADED
    global SUBTABLE, num_individuals_by_front

    data_table = load_json(f"{root_dir}/fronts_table.txt")
    table = data_table["fronts_table"]

    dir_pos = f"{root_dir}/{user_name}/pos_fronts_table.txt"
    pos_table = load_json(dir_pos)

    # ++++++++++++++++++++
    # rango de la subtabla
    pos_update = None
    # creación de los rangos por si no existen
    if not has_attr(pos_table, "ini") or not has_attr(pos_table, "end") :
        pos_table = {"ini": 0, "end": size_loaded_fronts}
        pos_update = 0

    # se reinicia la tabla
    if start:
        pos_table["ini"] = 0
        pos_table["end"] = size_loaded_fronts
        pos_update = 0


    # rango de los frentes a cargar
    if ini is not None:
        pos_table["ini"] = ini
        pos_table["end"] = ini + size_loaded_fronts
        pos_update = ini

    # se carga por id del frente
    if id_pareto is not None:
        try:
            pos_update = data_table["id_position"][id_pareto]
            pos_table["ini"] = pos_update
            pos_table["end"] = pos_update + size_loaded_fronts
        except Exception as e:
            print_error("No se encuentra el frente de pareto para cargar la subtabla")
            return
    
    # si el nuevo rango de frentes supera el tamaño de la tabla
    if pos_table["end"] > len(table):
        end_aux = len(table)
        ini_aux = len(table) - size_loaded_fronts
        if ini_aux < 0:
            ini_aux = 0

        pos_table["ini"] = ini_aux
        pos_table["end"] = end_aux
    
    # si la posición actual es mayor al de la tabla
    if pos_update != None:
        if pos_update >= len(table):
            pos_update = len(table) - 1

    # el num de frentes adelante de pos_update es diferente a size_loaded_fronts
    if pos_update != pos_table["ini"]:
        pos_table["ini"] = pos_update


    # ++++++++++++++++++++++++++
    # +++++ carga de la subtabla
    # ++++++++++++++++++++++++++

    # se actualiza la posición actual
    update_pos_table(user_name, pos_update, ini=pos_table["ini"], end=pos_table["end"])

    # subtabla con los ID de los frentes
    SUBTABLE = table[pos_table["ini"] : pos_table["end"]]
    print("frente inicial: ", SUBTABLE[0], "frente final: ", SUBTABLE[-1],f"rango: {pos_table}\n\n")
    # rango de los frentes en la tabla
    RANGE_LOADED = pos_table

    # if not start:
    #     load_fronts(SUBTABLE, pos_table["ini"])

    # ids en la subtabla y los cargados
    ids_subtable = set(SUBTABLE)
    ids_loaded = set(DATA_FRONT.keys())

    # se mantienen los frentes que no estan en la subtabla
    ids_dif = ids_loaded - ids_subtable
    # se eliminan los frentes 
    for id in ids_dif:
        del DATA_FRONT[id]

    ids_loaded = set(DATA_FRONT.keys())
    # se cargan los frentes que no tengan registro
    ids_dif = ids_subtable - ids_loaded
    for id in ids_dif:
        # se carga el frente
        res_obj = load_object_space_ros(id, min_sol=num_individuals_by_front, return_object=True, normalize=NORMALIZATION, non_dominated=True)
        DATA_FRONT[id] = {"id_pareto_front":id,"obj_space":res_obj["obj_space"], "num_inds":np.array(res_obj["num_inds"]), "id_pareto_inds": res_obj["id_pareto_inds"]}

    return table[pos_update]



# actualiza el número de generación en la cual se encuentra
def update_pos_table(user_name, pos=None, ini=None, end=None):
    dir_actual = f"{root_dir}/{user_name}/pos_fronts_table.txt"
    
    pos_tab = load_json(dir_actual)

    if pos != None :
        pos_tab["pos_table"] = pos
        # print("save actual pos ", pos)

    if(ini != None and end != None):
        pos_tab["range_table"] = {"ini":ini, "end":end}
        # print("save range ", pos_tab["range_table"])

    save_json(pos_tab, dir_actual)



def get_pos_table(user_name):
    dir_actual = f"{root_dir}/{user_name}/pos_fronts_table.txt"
    actual_f = load_json(dir_actual)

    return actual_f["pos_table"]


def predictions(msg):
    global emotion_values, num_inds, ID_PARETO_INDS
    global num_individuals_by_front

    user_name = msg.user_name
    #FALTA IMPLEMENTAR EL CONTROL DE USUARIO

    id_pareto = msg.id_pareto
    id_pareto_ind = msg.id_pareto_ind
    num_ind = msg.num_ind
    storage_pos = str(msg.storage_pos)

    print_color(f"\n\nadd to preference model\nid_pareto:{id_pareto} id_pareto_ind:{id_pareto_ind} ind_{num_ind} storage_pos:{storage_pos}", bcolors.OKBLUE)
    # print(f"add: {id_pareto}/ind_{storage_pos}    old:{FRONT_DATA['id_pareto']}/ind_{FRONT_DATA['storage_pos']}")
    
    # proceso de capturar las emociones de un frente
    is_err = False
    if FRONT_DATA["is_capturing"]:

        # # si se cambia de frente
        # if FRONT_DATA["id_pareto"] != id_pareto:
        #     print_error(f"Emoción con id diferente= new:{id_pareto} != old:{FRONT_DATA['id_pareto']}")
        #     is_err = True

        # si se cambia de experimento
        if FRONT_DATA["storage_pos"] != storage_pos:
            print_error(f"Cambio de ID del experimento y aún no se cargan todas las emociones= new:{storage_pos} != old:{FRONT_DATA['storage_pos']}")
            is_err = True        
            
    # proceso de inicio de la captura emocional
    if not FRONT_DATA["is_capturing"] or is_err:
        #  se incia el proceso de capturar las emociones de un frente
        FRONT_DATA["is_capturing"] = True
        FRONT_DATA["id_pareto"] = id_pareto
        FRONT_DATA["id_pareto_ind"] = id_pareto_ind
        FRONT_DATA["storage_pos"] = storage_pos

        emotion_values = np.empty((0, 2))
        num_inds = []
        ID_PARETO_INDS = []
        
        if is_err:
            print_color("Se reinicia la captura emocional", bcolors.OKGREEN)
        else:
            print_color("Inicio de la captura de emociones", bcolors.OKGREEN)


    # emociones
    emotion_values = np.vstack((emotion_values, msg.emotion_value))
    # num inds
    num_inds.append(num_ind)
    # id inds
    ID_PARETO_INDS.append(id_pareto_ind)

    print(f"Se han capturado {emotion_values.shape[0]} emociones")

    # se finaliza la captura
    if emotion_values.shape[0] >= num_individuals_by_front:
        FRONT_DATA["is_capturing"] = False

        print_color("Fin de la captura de emociones", bcolors.OKGREEN)
        print("\n\n")
        # copias de los valores
        emo_v, num_i, id_p = emotion_values.copy(), num_inds.copy(), ID_PARETO_INDS.copy()
        # se reinician las variables de almacenamiento
        emotion_values, num_inds, ID_PARETO_INDS = np.empty((0, 2)), [], []

        # se calcula el punto ref en base a las evaluaciones emocionales
        point_pref, num_ind_pref, id_pareto_ind_ref = preference_emo_model(user_name, id_pareto, emo_v, num_i, id_p)

        # siguiente frente
        next_front(user_name, point_pref, id_pareto, num_ind_pref, storage_pos)
    else:
        print("\n\n")



def preference_emo_model(user_name, id_pareto, emotion_values, num_inds, id_pareto_inds):

    print("--- preference_model ---")

    if type_model_pref == "euclidean":
        # norma de las evaluaciones
        dist = np.linalg.norm(emotion_values, ord=2, axis=1)
        
    if type_model_pref == "pbi":
        dist = PBI( emotion_values)
    
    # se obtiene el mayor 
    pos_max = np.argpartition(dist, -1)
    pos_max = pos_max[-1]

    # si no existe el id
    if not has_attr(DATA_FRONT, id_pareto):
        print_color(f"No existe el id_pareto {id_pareto}", bcolors.OKCYAN)
        load_subtable(user_name, id_pareto=id_pareto)

    # num del ind con mayor preferencia
    num_ind = num_inds[pos_max]
    # id del ind
    id_pareto_ind = id_pareto_inds[pos_max]
    
    pos_ind = DATA_FRONT[id_pareto]["num_inds"] == num_ind
    # mas de un ind o nada
    if np.sum(pos_ind) != 1:
        raise Exception("ERROR CON EL MANEJO DE INDIVIDUOS EN EL MODELO DE PREFERENCIAS ...")
    
    print(id_pareto, id_pareto_ind, num_ind, DATA_FRONT[id_pareto]["num_inds"])
    point_ref = DATA_FRONT[id_pareto]["obj_space"][pos_ind][0]
    print("point_ref", point_ref)

    # # posición relativa del ind
    # pos_ind = np.argwhere(DATA_FRONT[id_pareto]["num_inds"] == num_ind)
    # print(id_pareto,num_ind, pos_ind, DATA_FRONT[id_pareto]["num_inds"])
    # # si hay mas de uno existen inds repetidos o si no existe hay error en el manejo de inds
    # # s = np.sum(pos_ind)
    # # if s > 1 or s < 1:
    # if pos_ind.shape[0] != 1:
    #     # print_error()
    #     raise Exception("ERROR CON EL MANEJO DE INDIVIDUOS EN EL MODELO DE PREFERENCIAS ...")
    # else:
    #     pos_ind = pos_ind[0][0]

    # print("PREFERENCE MODEL: id_pareto", id_pareto ,"num inds ", num_inds, "sel_num_ind", num_ind, "sel_pos_ind", pos_ind )

    # point_ref = DATA_FRONT[id_pareto]["obj_space"][pos_ind]
    # print(point_ref)

    print("-----------------------")

    return point_ref, num_ind, id_pareto_ind

# servicio ROS para buscar un nuevo frente 
def search_new_front(req):
    print_color(req, bcolors.WARNING)
    next_front(req.user_name, np.array(req.reference_point), req.id_pareto_front, req.num_ind, req.storage_position)

    return search_pareto_frontResponse()

def print_arr(req):
    global METADATA_REG

    id_pareto = req.id_pareto_front

    if has_attr(METADATA_REG, id_pareto):
        meta = METADATA_REG[id_pareto]

        print(f"++++ {len(meta['num_inds'])}")
        print("meta", meta)
    

        if req.num_ind > 0:
            print("values", all_fronts[meta["range"]["ini"]: meta["range"]["end"]])
    else:
        print_error(f"No existe el registro del ID: {id_pareto}")

    return search_pareto_frontResponse()



def load_fronts_table():
    table = load_json(f"{root_dir}/fronts_table.txt")
    
    if len(table) == 0:
        raise Exception("No existe la tabla de frentes")
    else:
        return table

# busqueda de un nuevo frente 
def next_front(user_name, point_reference, id_pareto, num_ind_ref, storage_pos):
    global num_individuals_by_front
    global all_fronts
    global search_threshold_front
    global RANGE_LOADED, SUBTABLE

    # se carga la tabla de frentes
    fronts_table = load_fronts_table()

    # posición del frente actual
    pos_origin = fronts_table["id_position"][id_pareto]

    # nueva posición
    pos_ini_search = pos_origin + search_threshold_front
    print("pos_origin", pos_origin, "pos_ini_search", pos_ini_search)
    id_front_search = load_subtable(user_name, ini=pos_ini_search)

    min_dist = np.infty
    close_ind = {}

    # num_ind_close = ""
    # num_ind_close_relative = ""
    # id_pareto_close = ""
    
    # para cada frente
    for id_front in SUBTABLE:
        print("buscando en", id_front)
        # se extraen los individuos
        individuals = DATA_FRONT[id_front]["obj_space"]

        # chebyshev
        chebyshev_dist = np.abs(individuals - point_reference)
        chebyshev_dist = np.max(chebyshev_dist, axis=1)

        # ind con menor distancia
        num_ind = np.argmin(chebyshev_dist)
        dist_ = chebyshev_dist[num_ind]
        # print(dist_, min_dist)
        if dist_ < min_dist:
            min_dist = dist_
            close_ind["num_ind_relative"] = num_ind
            close_ind["num_ind_real"] = DATA_FRONT[id_front]["num_inds"][num_ind]
            close_ind["id_pareto"] = id_front

            # num_ind_close_relative = num_ind
            # num_ind_close = DATA_FRONT[id_front]["num_inds"][num_ind]
            # id_pareto_close = id_front
 
    # se actualiza la posición actual de la tabla
    update_pos_table(user_name, pos=fronts_table["id_position"][close_ind["id_pareto"]])


    print(f"from:{id_front_search} to new_front:{close_ind['id_pareto']}")

    # individuos de la población actual
    actual_inds = DATA_FRONT[close_ind["id_pareto"]]

    # valor del individuo mas cercano
    point_close = actual_inds["obj_space"][close_ind["num_ind_relative"]]

    # vector director de point_close al resto de individuos del frente
    search_vector = actual_inds["obj_space"] - point_close 

    # distancias euclideanas de point_close con el resto de su población
    search_vector = np.linalg.norm(search_vector, ord=2, axis=1)
    print("num_individuals_by_front", num_individuals_by_front)
    # si hay cambios en el tamaño de la extensión de la búsqueda
    size_search = num_individuals_by_front
    if len(ARRAY_CONTROL) > 0:
        size_search = ARRAY_CONTROL[0]
        del ARRAY_CONTROL[0]

    # se obtienen los de menor distancia
    if len(search_vector) == size_search: size_search = len(search_vector) - 1
    search_vector = np.argpartition(search_vector, size_search)
    search_vector = search_vector[:size_search]

    # num de los individuos seleccionados
    individuals_selected = actual_inds["num_inds"][search_vector]
    # id al que pertenecen
    id_pareto_inds = actual_inds["id_pareto_inds"][search_vector]
    
    if len(ARRAY_CONTROL) > 0:
        # valores de los ind en el espacio de los objetivos
        individuals_obj = actual_inds["obj_space"][search_vector]
        # individuos seleccionados
        inds_selected = select_inds(individuals_obj)

        # num de los individuos seleccionados
        inds_selected = individuals_selected[inds_selected]
        
        # guarda los ind para la búsqueda y los seleccionados
        dir_s = f"{root_dir}/{user_name}/history/sel_pref.txt"
        d = load_json(dir_s)
        if not isinstance(d, list):
            d = []
        d.append({"inds":individuals_selected.tolist(), "sel": inds_selected.tolist()})
        save_json(d, dir_s)

        # num de los individuos seleccionados
        individuals_selected = inds_selected

    print("individuals_selected", individuals_selected, "search_vector", search_vector)

    save_data(user_name, storage_pos, id_pareto, id_pareto_inds, point_reference, num_ind_ref, close_ind["id_pareto"], point_close, close_ind["num_ind_real"], individuals_selected)

    print_color(f"New front:{close_ind['id_pareto']}  old:{id_pareto}  indsel:{individuals_selected}",bcolors.OKCYAN)
    print("-----------------------------------------------------")


def save_data(user_name, storage_pos, id_pareto, id_pareto_inds, point_reference, num_ind_ref, id_pareto_close, point_close, num_ind_close, individuals_selected):
    
    # se carga el registro de individuos seleccionados
    all_inds_sel = load_json(f"{root_dir}/{user_name}/history/selected_individuals.txt")
    if not has_attr(all_inds_sel, storage_pos):
        all_inds_sel[storage_pos] = {}
    if not has_attr(all_inds_sel[storage_pos], id_pareto_close):
        all_inds_sel[storage_pos][id_pareto_close] = {}

    # formato 
    point_reference = point_reference.tolist()
    num_ind_ref = int(num_ind_ref)
    num_ind_close = int(num_ind_close)
    point_close = point_close.tolist()
    individuals_selected = individuals_selected.tolist()
    id_pareto_inds = list(id_pareto_inds)
    

    # se almacenan los individuos
    mref = { "id_pareto_front": id_pareto, "num_ind": num_ind_ref, "point": point_reference }
    mclose = { "id_pareto_front": id_pareto_close, "num_ind": num_ind_close, "point": point_close}
    obj_next = {"id_pareto_front": id_pareto_close, "num_individuals": individuals_selected, "id_pareto_inds": id_pareto_inds}

    all_inds_sel[storage_pos][id_pareto_close]["reference_point"] = mref
    all_inds_sel[storage_pos][id_pareto_close]["close_individual"] = mclose
    all_inds_sel[storage_pos][id_pareto_close]["selected_individuals"] = obj_next
    # se guarda el registro de inds sel
    save_json(all_inds_sel, f"{root_dir}/{user_name}/history/selected_individuals.txt")
    print_color(f"Resultado de la búsqueda\n{all_inds_sel[storage_pos][id_pareto_close]}", bcolors.OKGREEN)
    # último frente de la tabla
    last_front = load_json(f"{root_dir}/fronts_table.txt")
    last_front = last_front["fronts_table"][-1]

    if last_front == id_pareto_close and last_front == id_pareto:
        try:
            srv = rospy.ServiceProxy("/experiment_control", experiment_control)
            srv("last_front", "",[],False,False,False)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
    
    # se actualizan
    pub_next = rospy.Publisher("next_front_ind", next_front_ind, queue_size=10)
    msg_next = next_front_ind()
    print("..................")
    msg_next.id_pareto_front = id_pareto_close
    msg_next.id_pareto_inds = id_pareto_inds
    msg_next.num_individuals = individuals_selected
    msg_next.storage_position = storage_pos

    # msg_next.reference_point = point_reference
    # msg_next.num_ind_ref_point = num_ind_ref
    # msg_next.id_pareto_front_ref = id_pareto 

    # msg_next.close_individual = point_close
    # msg_next.num_ind_close = num_ind_close
    # msg_next.id_pareto_front_close = id_pareto_close

    pub_next.publish(msg_next)


# def get_last_front():
#     global root_dir

#     data_table = load_json(f"{root_dir}/fronts_table.txt")

#     return data_table["fronts_table"][-1]


def get_dist_inds(req):
    id_pareto = req.id_pareto_front

    # soluciones no dominadas
    res_obj = load_object_space_ros(id_pareto, min_sol=num_individuals_by_front, return_object=True, normalize=NORMALIZATION, non_dominated=True)
  
    points = res_obj["obj_space"]
    relative_num_ind = np.array(res_obj["num_inds"])
    relative_id_pareto = np.array(res_obj["id_pareto_inds"])
    
    # indiviudos seleccionados
    sel_inds = select_inds(points)
    relative_num_ind = relative_num_ind[sel_inds]
    relative_id_pareto = list(relative_id_pareto[sel_inds])
  
    return get_distributed_individualsResponse(relative_id_pareto, relative_num_ind)



def load_params():
    global num_individuals_by_front, search_threshold_front, size_loaded_fronts, ARRAY_CONTROL, NORMALIZATION

    path = f"{root_dir}/control_params.txt"
    params = load_json(path)
    num_individuals_by_front = params["num_individuals_by_front"]
    search_threshold_front = params["search_threshold_front"]
    size_loaded_fronts = params["size_loaded_fronts"]

    if params["extend_search"] == "extend_search":
        ARRAY_CONTROL = aux_init_control.copy()
        
    if params["extend_search"] == "non_extend_search":
        ARRAY_CONTROL = []
    
    NORMALIZATION = params["normalization"] == "do_norm"

    
    print("parameters: ", params, "NORMALIZATION", NORMALIZATION)


if __name__ == "__main__":

    rospy.init_node("preference_model")
    print("Nodo iniciado: preference_model")

    load_params()

    # load_range_fronts("AMC", 1, 60)

    # se inicia el servico para extraer las preferencias
    srv_f = rospy.Subscriber("emotional_prediction", emotional_prediction, predictions)

    # servicio de control
    srv_ctl = rospy.Service("control_preference", control_preference, control_pref)
    
    # servicio para encontrar un nuevo frente
    srv_chg = rospy.Service("search_pareto_front", search_pareto_front, search_new_front)

    # servicio para encontrar los inds distribuidos sobre el frente
    srv_dist_inds = rospy.Service("get_distributed_individuals", get_distributed_individuals, get_dist_inds)

    rospy.Service("print_arr", search_pareto_front, print_arr)

    
    # topicos
    # siguiente frente con sus individuos
    rospy.Publisher("next_front_ind", next_front_ind, queue_size=10)
    rospy.spin()

