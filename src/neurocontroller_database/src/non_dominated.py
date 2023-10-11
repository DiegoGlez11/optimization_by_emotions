import numpy as np


def dominance(a, b):
    if len(a) != len(b):
        # print()
        raise Exception(f"Error de dominancia: los puntos son de diferente dimensión {len(a)} != {len(b)}")
    
    # is_dom = 0
    # for i in range(len(a)):
    #     if a[i] < b[i]:
    #         is_dom += 1
    
    # if is_dom == len(a):
    #     return True
    # else:
    #     return False

    # ordenamiento por componente débil
    if np.sum(a <= b) == len(a):
        # ordenamiento por componente
        if np.sum(a == b) < len(a):
            return True
        else:
            return False
    else:
        return False
        
def fast_non_dominated_sort(P):
    # no_dominated = []

    # soluciones dominadas por p
    S = [[] for i in range(len(P))]
    # contador de dominancia
    # num de soluciones que son dominadas por p
    n = [0 for i in range(len(P))]
    
    rank = [0 for i in range(len(P))]
    F = []

    for p in range(len(P)):
        S[p] = []
        n[p] = 0

        for q in range(len(P)):

            if dominance(P[p], P[q]):
                S[p].append(q)
            else:
                if dominance(P[q], P[p]):
                    n[p] += 1
        
        if n[p] == 0:
            rank[p] = 1
            F.append(p)
    i = 1
    while len(F) != 0:
        Q = []
        for p_num in range(len(F)):
            p = F[p_num]

            for q_num in range(len(S[p])):
                q = S[p][q_num]

                n[q] -= 1

                if n[q] == 0:
                    rank[q] = i+1
                    Q.append(q)
        
        i+=1
        F = Q
    
    # rank = np.array(rank) == 1
    # return rank
    return np.array(rank)