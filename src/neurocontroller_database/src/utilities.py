from os.path import expanduser
import json, os
import numpy as np
from pdf2image import convert_from_bytes
from IPython.display import display


home = expanduser("~")
#dirección raiz
root_dir = home+"/catkin_ws/src/neurocontroller_database/database"
POPULATION_DATABASE_DIR = f"{home}/catkin_ws/src/neurocontroller_database/database_populations"
root_database = f"{home}/catkin_ws/src/neurocontroller_database"

def get_users(type_user="sam"):
    dirs = os.listdir(root_dir)
    list_dir = []
    for d in dirs:
        dir_user = f"{root_dir}/{d}"
        if d != "optimized_individuals" and not os.path.isfile(dir_user):
            list_dir.append(d)
            # if type_user == "sam" and os.path.isfile(f"{dir_user}/history/emotion_evaluation.txt"):
            #     list_dir.append(d)
            # if type_user == "bci" and os.path.isfile(f"{dir_user}/history/adquisition_data.txt"):
            #     list_dir.append(d)
            # if type_user == "protocol_video" and os.path.isdir(f"{dir_user}/protocol/history"):
            #     list_dir.append(d)

    return list_dir


# users_list= ["AleTC", "AMC", "BAMG", "DABB", "DFH", "DPB", "HRLV", "OGRA", "RGN", "SRAG",  "TaniaTC"]#, "diego"]
users_list = get_users("protocol_video")
users_list_deap = [f"s{i+1:02d}" for i in range(32)]


users_adquisition = [ {"name": "AleTC", "file": "history_emotion_evaluation_1.txt"}
        , {"name": "AMC", "file": "history_emotion_evaluation_1.txt"}
        , {"name": "BAMG", "file": "history_emotion_evaluation_1.txt"}
        , { "name": "DABB", "file": "history_emotion_evaluation_1.txt", } #history_emotion_evaluation_3
        , {"name": "DFH", "file": "history_emotion_evaluation_1.txt"}
        , { "name": "DPB", "file": "history_emotion_evaluation_1.txt" }
        , { "name": "HRLV", "file": "history_emotion_evaluation_1.txt" }
        , { "name": "OGRA", "file": "history_emotion_evaluation_1.txt", }
        , { "name": "RGN", "file": "history_emotion_evaluation_1.txt" }
        , { "name": "SRAG", "file": "history_emotion_evaluation_1.txt", }
        , {"name": "TaniaTC", "file": "history_emotion_evaluation_1.txt"}
        # , {"name": "diego", "file": "history_emotion_evaluation_1.txt"}
    ]

# user_protocol_aqd = [user["name"] for user in users_adquisition]
user_protocol_aqd = get_users("protocol_video")

# usuarios de la bci
# user_err_basal = ["SRAG", "DPB", "DFH", "DABB"]
# user_basal = ["AleTC", "AMC", "TaniaTC"]
# bci_user_list = user_basal + user_err_basal
bci_user_list = get_users("bci")
# sam_user_list = get_users()

models_list = ["DeepConvNet", "EEGNet", "ShallowConvNet"]
color_axes = "rgba(50, 59, 66, 0.534)"

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def print_error(txt):
    print(f"{bcolors.FAIL}ERROR: {txt} {bcolors.ENDC}")


def print_color(txt, color):
    print(f"{color}{txt} {bcolors.ENDC}")


def load_json( path):
    try:
        file_m = open(path, "r")
        data = file_m.read()
        data = json.loads(data)
        file_m.close()
        return data
    except OSError as e:
        # print("error",e)
        return {}


def save_json(object, path):
    file_emo = open(path, "w")
    file_emo.write(json.dumps(object))
    file_emo.close()


def has_attr(obj, attr):
    try:
        obj[attr]
        return True
    except:
        return False


def get_emotion_class(valence, arousal, umbral=0.5):
    
    if valence >= umbral:
        if arousal >= umbral:
            return 0
        else:
            return 3
    else:
        if arousal >= umbral:
            return 1
        else:
            return 2


def get_emotions(evals):
    num_class = []

    for v in evals:
        n_c = get_emotion_class(v[0], v[1])
        num_class.append(n_c)

    return np.array(num_class)


def show_pdf(path_):
    images = convert_from_bytes(open(path_, 'rb').read(), size=800)
    display(images[0])


# concatena los datos, listas o apila arrays 
def append_data(dict_values, key, value, type_append="append"):
    
    if type_append == "append":
        if not has_attr(dict_values, key):
            dict_values[key] = []

        dict_values[key].append(value)
    elif type_append == "vstack":
        if not has_attr(dict_values, key):
            dict_values[key] = np.empty((0,2))

        dict_values[key] = np.vstack((dict_values[key], value))
        
    elif type_append == "concatenate":
        if not has_attr(dict_values, key):
            dict_values[key] = []
        
        dict_values[key] = np.concatenate((dict_values[key], value))
    else:
        print_error(f"No se pueden agregar los datos de la forma: {type_append}")







# def get_max(num_gen_fronts):
#     # distribución de los números de generación
#     elements, distribution = np.unique(num_gen_fronts, return_counts=True)

#     # emociones con la ocurrencia más alta
#     max_pos = np.argwhere(distribution == np.amax(distribution))
#     size = max_pos.shape[0]

#     # si varias emociones tienen la misma proporcion se selecciona aleatoriamente
#     if size > 1:
#         index = random.randint(0, size-1)
#     else:
#         index = 0

#     # clase predicha
#     p = max_pos[index][0]
#     class_predicted = elements[p]

#     return class_predicted