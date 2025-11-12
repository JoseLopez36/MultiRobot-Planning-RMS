import numpy as np
import glob
import os

# Cargar trayectorias de los 3 UAVs
files = sorted(glob.glob('output/uav_traj_cpp_uav*.csv'))
trajs = []
times = []
for f in files:
    data = np.loadtxt(f, delimiter=',', skiprows=1)
    times.append(data[:,0])
    trajs.append(data[:,1:])




# Detectar colisiones considerando los segmentos rectos entre puntos consecutivos
def segment_min_dist(p1, p2, q1, q2):
    # Calcula la distancia mínima entre dos segmentos en 3D
    # p1,p2: extremos del segmento 1; q1,q2: extremos del segmento 2
    # Algoritmo estándar: https://stackoverflow.com/a/14772574
    u = p2 - p1
    v = q2 - q1
    w0 = p1 - q1
    a = np.dot(u, u)
    b = np.dot(u, v)
    c = np.dot(v, v)
    d = np.dot(u, w0)
    e = np.dot(v, w0)
    D = a*c - b*b
    sc, tc = 0.0, 0.0
    if D < 1e-12:
        sc = 0.0
        tc = e/c if c > 1e-12 else 0.0
    else:
        sc = (b*e - c*d) / D
        tc = (a*e - b*d) / D
    sc = np.clip(sc, 0.0, 1.0)
    tc = np.clip(tc, 0.0, 1.0)
    pt1 = p1 + sc*u
    pt2 = q1 + tc*v
    return np.linalg.norm(pt1 - pt2)

n = len(trajs)
min_sep = 2.0
min_dist = float('inf')
min_pair = None
min_time = None
collisions = []
for seg_idx in range(len(times[0])-1):
    t0 = times[0][seg_idx]
    t1 = times[0][seg_idx+1]
    # Para cada par de UAVs, calcular distancia mínima entre segmentos
    for i1 in range(n):
        p1 = trajs[i1][seg_idx]
        p2 = trajs[i1][seg_idx+1]
        for i2 in range(i1+1, n):
            q1 = trajs[i2][seg_idx]
            q2 = trajs[i2][seg_idx+1]
            d = segment_min_dist(p1, p2, q1, q2)
            if d < min_dist:
                min_dist = d
                min_pair = (i1, i2)
                min_time = (t0 + t1)/2
            if d < min_sep:
                collisions.append(((t0, t1), i1, i2, d))

print(f"Distancia mínima entre segmentos: {min_dist:.4f} entre UAV {min_pair[0]} y UAV {min_pair[1]} en t~{min_time:.2f}s")
if collisions:
    print(f"¡COLISIÓN detectada en {len(collisions)} tramos!")
    for (t0, t1), i1, i2, d in collisions:
        print(f"  tramo t=[{t0:.2f},{t1:.2f}]: UAV {i1} y UAV {i2} (dist={d:.3f})")
else:
    print("No hay colisión: todas las distancias >= min_sep en los segmentos")
