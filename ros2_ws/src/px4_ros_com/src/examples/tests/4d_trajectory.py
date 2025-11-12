"""
uav_pso_4d.py
Primera aproximación a planificación de trayectorias 4D (x,y,z,t) para varios UAVs usando PSO.

Uso:
    - Ejecuta `python uav_pso_4d.py` en un entorno con numpy y matplotlib instalados.
    - El script muestra gráficos (trayectorias 3D, distancias inter-UAV, evolución del coste)
      y opcionalmente puede guardar las trayectorias en CSV (descomenta la sección correspondiente).

Autor: José Antonio García Campanario
Fecha: 2025-10-31
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import random
import os

# --- Parámetros del escenario ---
n_uavs = 3
T = 25          # pasos de tiempo discretos
dt = 1.0        # duración entre pasos (segundos)
max_speed = 3.0 # unidades de distancia por dt
min_sep = 2.0   # separación mínima entre UAVs (penalizada)
space_bounds = np.array([[0, 40], [0, 40], [0.5, 10]])  # [x_min,x_max], [y_min,y_max], [z_min,z_max]

# Posiciones de inicio y objetivo (para cada UAV)
starts = np.array([[2, 2, 3],
                   [2, 8, 3],
                   [2, 14, 3]], dtype=float)
goals  = np.array([[35, 35, 4],
                   [35, 30, 6],
                   [35, 25, 3]], dtype=float)

# --- Funciones de ayuda ---
def clamp_pos(pos):
    #"""Limita las posiciones al dominio definido por space_bounds"""
    for dim in range(3):
        pos[..., dim] = np.clip(pos[..., dim], space_bounds[dim,0], space_bounds[dim,1])
    return pos

def path_length(trajectories):
    # trajectories: (n_uavs, T, 3)
    diffs = np.diff(trajectories, axis=1)
    dists = np.linalg.norm(diffs, axis=2)  # (n_uavs, T-1)
    return np.sum(dists)

def speed_penalty(trajectories):
    diffs = np.diff(trajectories, axis=1)
    speeds = np.linalg.norm(diffs, axis=2) / dt
    excess = np.maximum(0.0, speeds - max_speed)
    return np.sum(excess**2) * 100.0  # factor fuerte para penalizar violaciones

def collision_penalty(trajectories):
    # Penaliza cuando distancia entre dos UAVs < min_sep en cualquier paso temporal
    pen = 0.0
    for t in range(T):
        pos_t = trajectories[:, t, :]  # (n_uavs,3)
        # compute pairwise distances
        for i in range(n_uavs):
            for j in range(i+1, n_uavs):
                d = np.linalg.norm(pos_t[i] - pos_t[j])
                if d < min_sep:
                    pen += (min_sep - d)**2 * 100.0
    return pen

def boundary_penalty(trajectories):
    pen = 0.0
    # penaliza salir de límites (cuadrático)
    for dim in range(3):
        low = space_bounds[dim,0]
        high = space_bounds[dim,1]
        below = np.minimum(0.0, trajectories[...,dim] - low)
        above = np.maximum(0.0, trajectories[...,dim] - high)
        pen += np.sum(below**2 + above**2) * 100.0
    return pen

def objective(flat_vec):
    # convierte vector plano a (n_uavs, T, 3) y calcula coste
    traj = flat_vec.reshape((n_uavs, T, 3))
    # anclar inicio y fin (fuerte penalización si no coinciden)
    start_err = np.sum((traj[:,0,:] - starts)**2) * 1000.0
    goal_err  = np.sum((traj[:,-1,:] - goals)**2) * 1000.0
    L = path_length(traj)
    sp = speed_penalty(traj)
    coll = collision_penalty(traj)
    bnd = boundary_penalty(traj)
    smoothness = np.sum(np.linalg.norm(np.diff(traj, n=2, axis=1), axis=2)) * 30.0  # Penalización más fuerte
    cost = L + smoothness + sp + coll + bnd + start_err + goal_err
    return cost

# --- PSO ---
def run_pso(n_particles=500, iters=500, w=0.6, c1=1.8, c2=1.8, seed=None):  # Ajuste de parámetros
    if seed is not None:
        np.random.seed(seed)
    dim = n_uavs * T * 3

    particles = np.zeros((n_particles, dim))
    velocities = np.zeros_like(particles)
    pbest = np.zeros_like(particles)
    pbest_cost = np.full(n_particles, np.inf)
    gbest = None
    gbest_cost = np.inf

    for p in range(n_particles):
        traj = np.zeros((n_uavs, T, 3))
        for i in range(n_uavs):
            for k in range(3):
                traj[i, :, k] = np.linspace(starts[i,k], goals[i,k], T)
            # añadir ruido pequeño para diversidad
            traj[i] += np.random.normal(scale=0.6, size=traj[i].shape)
        traj = clamp_pos(traj)
        particles[p] = traj.reshape(-1)
        velocities[p] = np.random.normal(scale=0.5, size=dim)
        cost = objective(particles[p])
        pbest[p] = particles[p].copy()
        pbest_cost[p] = cost
        if cost < gbest_cost:
            gbest_cost = cost
            gbest = particles[p].copy()

    history = []
    # PSO loop
    for it in range(iters):
        r1 = np.random.rand(n_particles, dim)
        r2 = np.random.rand(n_particles, dim)
        velocities = w * velocities + c1 * r1 * (pbest - particles) + c2 * r2 * (gbest - particles)
        particles = particles + velocities
        # aplicar límites: clamp posiciones al espacio
        for p in range(n_particles):
            traj = particles[p].reshape((n_uavs, T, 3))
            traj = clamp_pos(traj)
            # forzar inicio y fin hacia valores deseados (más estable)
            traj[:,0,:] = starts.copy()
            traj[:,-1,:] = goals.copy()
            particles[p] = traj.reshape(-1)
            cost = objective(particles[p])
            # actualizar pbest
            if cost < pbest_cost[p]:
                pbest_cost[p] = cost
                pbest[p] = particles[p].copy()
            # actualizar gbest
            if cost < gbest_cost:
                gbest_cost = cost
                gbest = particles[p].copy()
        history.append(gbest_cost)
        if (it+1) % 25 == 0 or it == 0:
            print(f"Iter {it+1}/{iters}, best cost={gbest_cost:.3f}")  # Corregido con f-string

    best_traj = gbest.reshape((n_uavs, T, 3))
    return best_traj, history

def plot_results(best_traj, history):
    # 1) Trayectorias 3D
    fig = plt.figure(figsize=(10,6))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Trayectorias 3D optimizadas por PSO (x,y,z) vs tiempo")
    ax.set_xlim(space_bounds[0])
    ax.set_ylim(space_bounds[1])
    ax.set_zlim(space_bounds[2])
    for i in range(n_uavs):
        traj = best_traj[i]
        ax.plot(traj[:,0], traj[:,1], traj[:,2], label= "UAV {i}")
        ax.scatter(starts[i,0], starts[i,1], starts[i,2], marker='o', s=40)
        ax.scatter(goals[i,0], goals[i,1], goals[i,2], marker='^', s=40)
    ax.legend()
    plt.show()

    # 2) Distancias inter-UAV en el tiempo (para verificar separación)
    fig2 = plt.figure(figsize=(10,4))
    ax2 = fig2.add_subplot(111)
    ax2.set_title("Distancias inter-UAV a lo largo del tiempo (deben ser >= min_sep)")
    pairs = []
    for i in range(n_uavs):
        for j in range(i+1, n_uavs):
            pairs.append((i,j))
    for (i,j) in pairs:
        dists = np.linalg.norm(best_traj[i] - best_traj[j], axis=1)
        ax2.plot(np.arange(T)*dt, dists, label= "{i}-{j}")
    ax2.axhline(min_sep, linestyle='--', linewidth=1)
    ax2.set_xlabel("Tiempo (s)")
    ax2.set_ylabel("Distancia")
    ax2.legend()
    plt.show()

    # 3) Evolución del coste
    fig3 = plt.figure(figsize=(8,3))
    ax3 = fig3.add_subplot(111)
    ax3.set_title("Evolución del coste (historia de gbest)")
    ax3.plot(np.arange(len(history)), history)
    ax3.set_xlabel("Iteración")
    ax3.set_ylabel("Coste")
    plt.show()

def save_trajectories_csv(best_traj, filename_prefix='uav_traj', dt=1.0):
    os.makedirs('output', exist_ok=True)
    n_uavs, T, _ = best_traj.shape
    for i in range(n_uavs):
        data = np.zeros((T, 4))
        data[:,0] = np.arange(T) * dt
        data[:,1:] = best_traj[i]
        fname = os.path.join('output', f"{filename_prefix}_uav{i}.csv")
        np.savetxt(fname, data, delimiter=',', header='t,x,y,z', comments='')
        print(f"Guardado: {fname}")

def trajectories_to_4d(best_traj, dt=1.0):
    # best_traj: (n_uavs, T, 3)
    n_uavs, T, _ = best_traj.shape
    out = np.zeros((n_uavs, T, 4))
    out[:, :, 0:3] = best_traj
    out[:, :, 3] = np.arange(T) * dt
    return out

def get_uav_points_4d(best_traj, uav_index, dt=1.0):
    traj4 = trajectories_to_4d(best_traj, dt)
    return traj4[uav_index]   # array (T,4) con filas [x,y,z,t]

if __name__ == '__main__':
    best_traj, history = run_pso(n_particles=500, iters=500, seed=42)  # Más partículas e iteraciones
    plot_results(best_traj, history)
    # Descomenta la siguiente línea si quieres guardarlo en CSV automáticamente
    # save_trajectories_csv(best_traj)
