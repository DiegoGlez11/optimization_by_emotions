import rospy
import numpy as np
from neurocontroller_database.srv import get_type_id
from neurocontroller_database.srv import get_id
from neurocontroller_database.srv import load_obj_space
from neurocontroller_database.srv import id_to_path

def id_to_filePath(id_pareto, is_object_space=False, return_id=False, version="v2"):
    try:
        srv = rospy.ServiceProxy("id_to_path", id_to_path)
        res = srv(id_pareto, is_object_space, return_id, version)
        return res
    except Exception as e:
        print(f"Error al convertir el ID:{id_pareto} a su ruta:\n{e}")

def create_id(base_name, directory):
    rospy.wait_for_service("get_id")

    try:
        srv = rospy.ServiceProxy("get_id", get_id)
        res = srv(base_name, directory)
        return res.id
    except rospy.ServiceException as e:
        print(f"Error al creare el ID {base_name}\n", e)


def get_type_experiment(id):
    rospy.wait_for_service("get_type_id")

    try:
        srv = rospy.ServiceProxy("get_type_id", get_type_id)
        res = srv(id)
        return res.type
    except rospy.ServiceException as e:
        print("Error al obtener el tipo de ID", e)


# # carga una poblacion
# def load_object_space_ros(id_pareto, num_inds=None ,return_object=False):
#     global all_fronts, metadata_reg, metadata_array

#     if type(num_inds) != list and num_inds != None:
#         raise Exception(f"Error al cargar el espacio de los objetivo: id_pareto={id_pareto}: num_inds={num_inds}")

#     if num_inds == None:
#         num_inds = 0

#     rospy.wait_for_service("load_object_space")

#     try:
#         load_objetives = rospy.ServiceProxy("load_obj_space", load_obj_space)
#         res = load_objetives(id_pareto, num_inds, NORMALIZATION, )
       
#         # se pasa a numpy
#         data = np.array(res.obj_space)
#         # 1D to 2D
#         data = data.reshape((res.pop_size, res.num_obj))
#         # solo los objetivos
#         data = data[:,:3]

#         if return_object:
#             if type(num_inds) == list:
#                 return {"obj_space":data[num_inds], "num_inds":num_inds}
#             else:
#                 return {"obj_space":data, "num_inds":np.arange(data.shape[0])}


#         if type(num_inds) == list:
#             return data[num_inds]
#         else:
#             return data

#     except rospy.ServiceException as e:
#         print("Error al cargar la poblacion", e)




# load_object_space_ros_non_dom
# carga una poblacion
def load_object_space_ros(id_pareto, min_sol=0, return_object=False, normalize=True, non_dominated=True):
    global all_fronts, metadata_reg, metadata_array

    rospy.wait_for_service("load_object_space")

    try:
        load_objetives = rospy.ServiceProxy("load_object_space", load_obj_space)
        res = load_objetives(id_pareto, min_sol, normalize, non_dominated)

        # se pasa a numpy
        data = np.array(res.obj_space)
        # 1D to 2D
        data = data.reshape((res.pop_size, res.num_obj))
        # solo los objetivos
        data = data[:,:3]

        if return_object:
            return {"obj_space":data, "num_inds":res.relative_num_ind, "id_pareto_inds": np.array(res.relative_id_pareto)}
        else:
            return data

    except rospy.ServiceException as e:
        print("Error al cargar la poblacion no dominada", e)