import numpy as np


# para PBI
# p1 = np.array([0.5,0])
# p2 = np.array([1,1])
# theta = np.array([0.45, 0.55])

def projection(a,b):
    r= np.dot(a, b) / np.dot(b, b)
    r = np.vstack((r,r))
    r = r.transpose(1,0)
    return r * b

def PBI(points, p1=[0.5,0], p2=[1,1], theta = [0.45, 0.55]):
    # global p1, p2, theta
    
    p1 = np.array(p1)
    p2 = np.array(p2)
    theta = np.array(theta)

    # vector dir
    d = points-p1
    d_w = p2-p1

    # projection
    inter = projection(d, d_w)

    d1_dist = np.linalg.norm(points-inter, ord=2, axis=1)
    d2_dist = np.linalg.norm(inter-p1, ord=2, axis=1)

    dot = np.dot(theta, [d1_dist, d2_dist])

    return dot

def get_pbi(emo_values):
    
    p_r = PBI(emo_values)

    # mayor PBI del frente
    pos_m = np.argpartition(p_r, -1)
    pos_m = pos_m[-1]
    max_pbi = p_r[pos_m]

    return max_pbi, pos_m, p_r