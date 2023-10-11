import rospy
import numpy as np
from neurocontroller_database.srv import get_type_id
from neurocontroller_database.srv import get_id
from neurocontroller_database.srv import load_obj_space


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


# carga una poblacion
def load_object_space_ros(id_pareto, num_inds=None, return_object=False):
    global all_fronts, metadata_reg, metadata_array

    rospy.wait_for_service("load_obj_space")

    try:
        load_objetives = rospy.ServiceProxy("load_obj_space", load_obj_space)
        res = load_objetives(id_pareto, 0)
       
        # se pasa a numpy
        data = np.array(res.obj_space)
        # 1D to 2D
        data = data.reshape((res.pop_size, res.num_obj))
        # solo los objetivos
        data = data[:,:3]

        if return_object:
            if type(num_inds) == list:
                return {"obj_space":data[num_inds], "num_inds":num_inds}
            else:
                return {"obj_space":data, "num_inds":np.arange(data.shape[0])}


        if type(num_inds) == list:
            return data[num_inds]
        else:
            return data

    except rospy.ServiceException as e:
        print("Error al cargar la poblacion", e)



# def chebyshev_distance(x, y):
#     dist = np.abs(y-x)
#     dist = np.max(dist, axis=1)
#     return dist



# carga una poblacion
def load_object_space_ros_non_dom(id_pareto, min_sol=0, return_object=False, normalize=True, non_dominated=True, test_data=False):
    global all_fronts, metadata_reg, metadata_array

    rospy.wait_for_service("load_object_space")

    try:
        load_objetives = rospy.ServiceProxy("load_object_space", load_obj_space)
        res = load_objetives(id_pareto, min_sol, normalize, non_dominated, test_data)

        # se pasa a numpy
        data = np.array(res.obj_space)
        # 1D to 2D
        data = data.reshape((res.pop_size, res.num_obj))
        # solo los objetivos
        data = data[:,:3]

        # # soluciones equidistantes
        # if equidistant_solutions:
        #     # soluciones extremas
        #     chebyshev_distance()


        if return_object:
            return {"obj_space":data, "num_inds":res.num_inds, "id_paretos": res.id_pareto_fronts}
        else:
            return data

        # if type(num_inds) == list:
        #     return data[num_inds]
        # else:
        #     return data
    except rospy.ServiceException as e:
        print("Error al cargar la poblacion no dominada", e)