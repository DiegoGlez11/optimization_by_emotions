
import numpy as np
from pymoo.util.ref_dirs import get_reference_directions

def normalization(St):
    # intercept
    a = np.max(St, axis=0)

    # ideal point
    z = np.min(St, axis=0)

    # translate objetives
    f_ = St - z

    # normalization
    fn = f_ / (a - z)

    return fn


def vect_proj(Sp, w): 
    num = np.dot(Sp, w.T)
    dem = np.sum(w*w, axis=1)
    proj_scalar = num/dem
    # print(proj_scalar)
    # print("--------")
    res = []
    for i in range(w.shape[0]):
        proj_sca = np.repeat(proj_scalar[:,i], w.shape[1]).reshape(proj_scalar.shape[0], w.shape[1])
        proj_vect = proj_sca*w[i]
        res.append(proj_vect)

    return np.array(res)


def perpendicular_distance(St,W):
    # vector projection
    v_proj = vect_proj(St, W)

    w_dist = []
    for i in range(v_proj.shape[0]):
        # direction vector from projection to St
        vd = St - v_proj[i]
        # perpendicular distance
        dist_per = np.linalg.norm(vd, axis=1)
        w_dist.append(dist_per)
        
    w_dist = np.array(w_dist).T

    return w_dist


def unique_inds(sel_ref, w_dist):

    # orden de las distancias
    sel_inds_sort = np.argsort(w_dist, axis=0)

    # individuos seleccionados por vector de referencia
    sel_inds = sel_inds_sort[0, sel_ref]

    # individuos únicos
    sel_inds_uni = np.unique(sel_inds, return_counts=True)

    # se detecta que hay individuos repetidos
    if sel_inds.shape[0] != sel_inds_uni[0].shape[0]:
        # inds repetidos
        repeated_pos = sel_inds_uni[1] > 1
        repeated_inds_num = sel_inds_uni[0][repeated_pos]
        # repeated_inds_count = sel_inds_uni[1][repeated_pos]

        # por cada individuo distinto que pertenece a mas de un w
        for num_rep in repeated_inds_num:
            # pos de los vectores w que poseen el individuo num_rep
            pos_w = sel_inds == num_rep
            pos_w_real = sel_ref[pos_w]
            pos_w_relative = np.arange(sel_inds.shape[0])[pos_w]

            # para cada w
            for w_info in zip(pos_w_real[1:], pos_w_relative[1:]):
                # distancias ordenadas del vector w con los individuos
                dist_sort = sel_inds_sort[1:, w_info[0]]
                
                # para cada ind
                for ind_sort in dist_sort:
                    # si tiene registro
                    if not ind_sort in sel_inds:
                        # print("new_sort",ind_sort, "pos_sel",w_info[1])
                        sel_inds[w_info[1]] = ind_sort
                        break 
    
    return sel_inds


def select_inds(points, is_crowding_dist=False, get_all_data = False):
    # normalization
    St = normalization(points)

    if not is_crowding_dist:
        # puntos de referencia a utilizar para encontrar los ind con menor distancia
        sel_ref = np.array([0,3,5,9])

        # ref points 
        W = get_reference_directions("uniform", 3, n_partitions=3, seed=1)
        
        # distance
        w_dist = perpendicular_distance(St, W)

        sel_inds = unique_inds(sel_ref, w_dist)

        if not get_all_data:
                return sel_inds
        else:

            return {"num_inds":sel_inds, "ref_points": W, "points":St, "distance": w_dist}
    else:
        num_inds_sel = 4

        cd = crowding_distance(St)

        # mayor crowding distance
        cd_sort = np.argsort(cd)

        sel_inds = cd_sort[:num_inds_sel]
    
        if not get_all_data:
                return sel_inds
        else:
            return {"num_inds":sel_inds,"distance":cd ,"points":St}
    
    

    # pos min dist
    # pi_pos = np.argmin(dist, axis=1)

    # val of w assigned to St_i
    # pi = W[pi_pos]

    # distance of St_i assigned to w_j
    # d = dist[np.arange(pi_pos.shape[0]) , pi_pos]
    
    # return {"pi_pos":pi_pos, "pi":pi, "distance":d}


def crowding_distance(P):
    # rango
    p_min = np.min(P, axis=0)
    p_max = np.max(P, axis=0)
    r = p_max - p_min

    # inicio crowding distance
    CD = np.zeros(P.shape)

    # ordenamiento ascendente
    P_sort = np.argsort(P, axis=0)
    
    # indices
    i1 = np.arange(P.shape[0]) - 1
    i1[0] = 0
    index1 = P_sort[i1]
    i2 = np.arange(P.shape[0]) + 1
    # i2[-1] = 0
    i2[-1] = i2[-2]
    index2 = P_sort[i2]
    
    # Para cada dimensión
    for i in range(P.shape[1]):
        # crowding distance
        CD[:,i] = np.abs((P[index2[:,i], i] - P[index1[:,i], i]) / r[i])
        #infinito soluciones extremas
        # CD[ 0 ,i] = np.infty
        # CD[-1 ,i] = np.infty
        # CD[ 0 ,i] = np.abs((P[1,i]-P[0,i])/r[i])
        # CD[-1 ,i] = np.abs((P[-1,i]-P[-2,i])/r[i])


    # los valores de los puntos en CD, cada dimensión
    # tiene diferente orden. Se ordenan los índices de los individuos
    pos_sort = np.argsort(P_sort, axis=0)
    CD_R = np.zeros(CD.shape)

    # Para cada dimensión
    for i in range(P.shape[1]):
        # pos del índice en la dimensión i
        pos_p = pos_sort[:, i]

        # índices de los individuos
        # pos_p = P_sort[pos_p, i]
        # print("P_sort", pos_p)

        CD_R[:, i] += CD[pos_p, i]
    
    return np.sum(CD_R, axis=1)
    

def crowding_distance2(P, pos):
    # rango
    p_min = np.min(P, axis=0)
    p_max = np.max(P, axis=0)
    r = p_max - p_min

    # inicio crowding distance
    CD = np.zeros(P.shape)

    # crowding distance
    # CD = (P - P[pos]) / r
    CD = np.abs(P - P[pos]) / r

    return np.sum(CD, axis=1)



