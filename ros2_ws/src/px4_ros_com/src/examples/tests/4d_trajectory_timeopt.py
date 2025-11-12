"""
uav_pso_4d_timeopt.py
Segunda versión: optimiza posiciones (x,y,z) y tiempos (delta-t) por intervalo.

Concepto:
- Cada UAV tiene T posiciones (incluye inicio y fin) y T-1 intervalos de tiempo.
- Los intervalos de tiempo son variables y positivas; los tiempos de cada paso son acumulados.
- Las velocidades se calculan como distancia / delta_t y se penalizan si exceden max_speed.
- La detección de colisiones usa interpolación lineal para evaluar la posición de otros UAVs en tiempos arbitrarios.

Uso:
    python 4d_trajectory_timeopt.py

Autor: José Antonio García Campanario
Fecha: 2025-10-31
"""

import numpy as np
import math
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cm

# ----- Escenario (copiado/adaptado) -----
n_uavs = 3
T = 25
dt_base = 1.0           # tiempo base (escala para inicialización)
max_speed = 3.0         # unidades de distancia por segundo
min_sep = 2.0
space_bounds = np.array([[0, 40], [0, 40], [0.5, 10]])

starts = np.array([[2, 2, 3],
                   [2, 8, 3],
                   [2, 14, 3]], dtype=float)
goals  = np.array([[35, 35, 4],
                   [35, 30, 6],
                   [35, 25, 3]], dtype=float)

# ----- Helpers -----

def clamp_pos(pos):
    for dim in range(3):
        pos[..., dim] = np.clip(pos[..., dim], space_bounds[dim,0], space_bounds[dim,1])
    return pos


def cumulative_times_from_deltas(deltas):
    """deltas: (n_uavs, T-1) -> times: (n_uavs, T)
    times[:,0] = 0
    times[:,i] = sum_{k< i} deltas[:,k]
    """
    n, m = deltas.shape
    times = np.zeros((n, m+1))
    times[:,1:] = np.cumsum(deltas, axis=1)
    return times


def interp_pos_at_time(traj, times, t_query):
    """Interpolación lineal en una trayectoria de un UAV.
    traj: (T,3), times: (T,), t_query: scalar
    Devuelve posición interpolada. Si t_query fuera de rango, devuelve endpoint.
    """
    if t_query <= times[0]:
        return traj[0]
    if t_query >= times[-1]:
        return traj[-1]
    # localizar segmento
    idx = np.searchsorted(times, t_query) - 1
    t0 = times[idx]
    t1 = times[idx+1]
    p0 = traj[idx]
    p1 = traj[idx+1]
    if t1 == t0:
        return p0
    alpha = (t_query - t0) / (t1 - t0)
    return (1-alpha) * p0 + alpha * p1

# ----- Cost components (accept time deltas) -----

def path_length(trajectories):
    diffs = np.diff(trajectories, axis=1)
    return np.sum(np.linalg.norm(diffs, axis=2))


def speed_penalty_timeopt(trajectories, deltas):
    """trajectories: (n_uavs,T,3), deltas: (n_uavs,T-1)
    Penaliza velocidades que exceden max_speed (vel = dist / delta)
    """
    pen = 0.0
    diffs = np.diff(trajectories, axis=1)
    dists = np.linalg.norm(diffs, axis=2)  # (n_uavs, T-1)
    speeds = dists / deltas
    excess = np.maximum(0.0, speeds - max_speed)
    pen += np.sum(excess**2) * 100.0
    # penalizar delta muy pequeños (para evitar t->0)
    pen += np.sum(np.maximum(0.0, 0.05 - deltas)**2) * 1000.0
    return pen


def collision_penalty_timeopt(trajectories, deltas):
    """Considera tiempos diferentes por UAV y usa interpolación para chequear colisiones.
    Para cada UAV u y cada timestamp t_u[i], evalúa la posición del otro UAV v interpolada en ese tiempo.
    """
    pen = 0.0
    times = cumulative_times_from_deltas(deltas)
    for u in range(n_uavs):
        for i in range(T):
            t_u = times[u, i]
            p_u = trajectories[u, i]
            for v in range(u+1, n_uavs):
                p_v = interp_pos_at_time(trajectories[v], times[v], t_u)
                d = np.linalg.norm(p_u - p_v)
                if d < min_sep:
                    pen += (min_sep - d)**2 * 200.0
    return pen


def boundary_penalty(trajectories):
    pen = 0.0
    for dim in range(3):
        low = space_bounds[dim,0]
        high = space_bounds[dim,1]
        below = np.minimum(0.0, trajectories[...,dim] - low)
        above = np.maximum(0.0, trajectories[...,dim] - high)
        pen += np.sum(below**2 + above**2) * 100.0
    return pen

# objective: takes flat vector with positions and deltas

def objective_timeopt(flat_vec):
    # flat_vec layout: positions (n_uavs*T*3) then deltas (n_uavs*(T-1))
    pos_len = n_uavs * T * 3
    del_len = n_uavs * (T-1)
    pos_flat = flat_vec[:pos_len]
    del_flat = flat_vec[pos_len: pos_len + del_len]
    traj = pos_flat.reshape((n_uavs, T, 3))
    deltas = del_flat.reshape((n_uavs, T-1))
    # anchor start and end softly via penalty
    start_err = np.sum((traj[:,0,:] - starts)**2) * 1000.0
    goal_err  = np.sum((traj[:,-1,:] - goals)**2) * 1000.0
    L = path_length(traj)
    sp = speed_penalty_timeopt(traj, deltas)
    coll = collision_penalty_timeopt(traj, deltas)
    bnd = boundary_penalty(traj)
    # smoothness on positions and time deltas
    pos_smooth = np.sum(np.linalg.norm(np.diff(traj, n=2, axis=1), axis=2)) * 10.0
    time_smooth = np.sum(np.abs(np.diff(deltas, axis=1))) * 50.0
    cost = L + pos_smooth + time_smooth + sp + coll + bnd + start_err + goal_err
    return cost

# ----- PSO for time-optimized trajectories -----

def run_pso_timeopt(n_particles=300, iters=300, w=0.6, c1=1.8, c2=1.8, seed=None):
    if seed is not None:
        np.random.seed(seed)
    pos_dim = n_uavs * T * 3
    del_dim = n_uavs * (T-1)
    dim = pos_dim + del_dim

    particles = np.zeros((n_particles, dim))
    velocities = np.zeros_like(particles)
    pbest = np.zeros_like(particles)
    pbest_cost = np.full(n_particles, np.inf)
    gbest = None
    gbest_cost = np.inf

    # Initialize particles: positions along straight line + small noise; deltas = dt_base + noise
    for p in range(n_particles):
        traj = np.zeros((n_uavs, T, 3))
        deltas = np.zeros((n_uavs, T-1))
        for i in range(n_uavs):
            for k in range(3):
                traj[i, :, k] = np.linspace(starts[i,k], goals[i,k], T)
            traj[i] += np.random.normal(scale=0.6, size=traj[i].shape)
            # deltas around dt_base
            deltas[i] = np.clip(dt_base + np.random.normal(scale=0.1, size=(T-1,)), 0.05, 10.0)
        traj = clamp_pos(traj)
        particles[p, :pos_dim] = traj.reshape(-1)
        particles[p, pos_dim:] = deltas.reshape(-1)
        velocities[p] = np.random.normal(scale=0.5, size=dim)
        cost = objective_timeopt(particles[p])
        pbest[p] = particles[p].copy()
        pbest_cost[p] = cost
        if cost < gbest_cost:
            gbest_cost = cost
            gbest = particles[p].copy()

    history = []
    for it in range(iters):
        r1 = np.random.rand(n_particles, dim)
        r2 = np.random.rand(n_particles, dim)
        velocities = w * velocities + c1 * r1 * (pbest - particles) + c2 * r2 * (gbest - particles)
        particles = particles + velocities
        # enforce bounds: clamp positions and deltas
        for p in range(n_particles):
            traj = particles[p, :pos_dim].reshape((n_uavs, T, 3))
            deltas = particles[p, pos_dim:].reshape((n_uavs, T-1))
            traj = clamp_pos(traj)
            # clamp deltas to positive reasonable range
            deltas = np.clip(deltas, 0.05, 10.0)
            # re-pack
            particles[p, :pos_dim] = traj.reshape(-1)
            particles[p, pos_dim:] = deltas.reshape(-1)
            cost = objective_timeopt(particles[p])
            if cost < pbest_cost[p]:
                pbest_cost[p] = cost
                pbest[p] = particles[p].copy()
            if cost < gbest_cost:
                gbest_cost = cost
                gbest = particles[p].copy()
        history.append(gbest_cost)
        if (it+1) % 25 == 0 or it == 0:
            print(f"Iter {it+1}/{iters}, best cost={gbest_cost:.3f}")

    best_pos = gbest[:pos_dim].reshape((n_uavs, T, 3))
    best_deltas = gbest[pos_dim:].reshape((n_uavs, T-1))
    return best_pos, best_deltas, history

# ----- Utilities: export and validation -----

def trajectories_to_4d_timeopt(best_traj, deltas):
    times = cumulative_times_from_deltas(deltas)
    n, T, _ = best_traj.shape
    out = np.zeros((n, T, 4))
    out[:, :, :3] = best_traj
    out[:, :, 3] = times
    return out


def validate_trajectory_timeopt(best_traj, deltas):
    """Devuelve métricas útiles: max_speed_obs, min_sep_obs, out_of_bounds_count"""
    metrics = {}
    # speeds
    dists = np.linalg.norm(np.diff(best_traj, axis=1), axis=2)  # (n_uavs,T-1)
    speeds = dists / deltas
    metrics['max_speed_obs'] = float(np.nanmax(speeds))
    # min separation (sample each UAV's times)
    times = cumulative_times_from_deltas(deltas)
    min_sep_obs = float(np.inf)
    for u in range(n_uavs):
        for i in range(T):
            p_u = best_traj[u, i]
            t_u = times[u, i]
            for v in range(u+1, n_uavs):
                p_v = interp_pos_at_time(best_traj[v], times[v], t_u)
                d = np.linalg.norm(p_u - p_v)
                if d < min_sep_obs:
                    min_sep_obs = d
    metrics['min_sep_obs'] = float(min_sep_obs)
    # bounds
    out_of_bounds = 0
    for dim in range(3):
        low = space_bounds[dim,0]
        high = space_bounds[dim,1]
        out_of_bounds += int(np.sum((best_traj[...,dim] < low) | (best_traj[...,dim] > high)))
    metrics['out_of_bounds_count'] = int(out_of_bounds)
    return metrics

# ----- Simple CSV save for timeopt trajectories -----

def save_trajectories_csv_timeopt(best_traj, deltas, filename_prefix='uav_traj_timeopt'):
    os.makedirs('output', exist_ok=True)
    traj4 = trajectories_to_4d_timeopt(best_traj, deltas)
    for i in range(n_uavs):
        data = traj4[i]
        fname = os.path.join('output', f"{filename_prefix}_uav{i}.csv")
        # columns: t,x,y,z
        data_out = np.zeros((T, 4))
        data_out[:,0] = data[:,3]
        data_out[:,1:] = data[:,0:3]
        np.savetxt(fname, data_out, delimiter=',', header='t,x,y,z', comments='')
        print(f"Guardado: {fname}")


### --- Visualización para time-optimized trajectories ---


def sample_trajectories_on_grid(best_traj, deltas, num_samples=None):
    """Muestra las trayectorias de todos los UAVs en una rejilla temporal común.
    Devuelve: times_grid (M,), samples (n_uavs, M, 3)
    Si num_samples es None se usa la unión de todos los tiempos discretos.
    """
    times = cumulative_times_from_deltas(deltas)  # (n_uavs, T)
    if num_samples is None:
        # unión de todos los tiempos (ordenados)
        grid = np.unique(times.flatten())
    else:
        tmax = np.max(times[:, -1])
        grid = np.linspace(0.0, tmax, num_samples)
    n = best_traj.shape[0]
    M = grid.size
    samples = np.zeros((n, M, 3))
    for u in range(n):
        for i, tg in enumerate(grid):
            samples[u, i] = interp_pos_at_time(best_traj[u], times[u], tg)
    return grid, samples


def plot_timeopt_results(best_traj, deltas, history=None, show=True, save_prefix=None, sample_num=None):
    """Dibuja:
    - 3D: trayectorias x,y,z con color por tiempo
    - distancias par-a-par en la rejilla temporal común
    - historial de coste (si se pasa)
    Si save_prefix se proporciona, guarda PNG en output/{save_prefix}_*.png
    """
    os.makedirs('output', exist_ok=True)
    # 3D plot
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')
    times = cumulative_times_from_deltas(deltas)
    cmap = cm.get_cmap('viridis')
    tmin = 0.0
    tmax = np.max(times[:, -1])
    for u in range(best_traj.shape[0]):
        t_u = times[u]
        traj = best_traj[u]
        # color by normalized time per point
        norm_t = (t_u - tmin) / (tmax - tmin + 1e-9)
        for i in range(traj.shape[0]-1):
            col = cmap(norm_t[i])
            ax.plot(traj[i:i+2,0], traj[i:i+2,1], traj[i:i+2,2], color=col)
        ax.scatter(traj[:,0], traj[:,1], traj[:,2], c=norm_t, cmap='viridis', s=20)
        ax.scatter(traj[0,0], traj[0,1], traj[0,2], marker='o', s=40)
        ax.scatter(traj[-1,0], traj[-1,1], traj[-1,2], marker='^', s=40)
    ax.set_xlim(space_bounds[0])
    ax.set_ylim(space_bounds[1])
    ax.set_zlim(space_bounds[2])
    ax.set_title('Trayectorias optimizadas (x,y,z) con color por tiempo')
    if save_prefix:
        fig.savefig(os.path.join('output', f"{save_prefix}_3d.png"))
    if show:
        plt.show()
    plt.close(fig)

    # Pairwise distances on a common grid
    grid, samples = sample_trajectories_on_grid(best_traj, deltas, num_samples=sample_num)
    pairs = []
    n = samples.shape[0]
    for i in range(n):
        for j in range(i+1, n):
            pairs.append((i, j))
    fig2, ax2 = plt.subplots(figsize=(10,4))
    for (i,j) in pairs:
        dists = np.linalg.norm(samples[i] - samples[j], axis=1)
        ax2.plot(grid, dists, label=f"{i}-{j}")
    ax2.axhline(min_sep, linestyle='--', color='k', label='min_sep')
    ax2.set_xlabel('Tiempo (s)')
    ax2.set_ylabel('Distancia inter-UAV')
    ax2.set_title('Distancias par-a-par sobre la rejilla temporal común')
    ax2.legend()
    if save_prefix:
        fig2.savefig(os.path.join('output', f"{save_prefix}_pairwise.png"))
    if show:
        plt.show()
    plt.close(fig2)

    # Cost history
    if history is not None:
        fig3, ax3 = plt.subplots(figsize=(8,3))
        ax3.plot(np.arange(len(history)), history)
        ax3.set_xlabel('Iteración')
        ax3.set_ylabel('Coste (gbest)')
        ax3.set_title('Evolución del coste')
        if save_prefix:
            fig3.savefig(os.path.join('output', f"{save_prefix}_cost.png"))
        if show:
            plt.show()
        plt.close(fig3)


# ----- If run as script, run a short example for demonstration -----
if __name__ == '__main__':
    # Ejecutar demo con parámetros reducidos para que termine rápido
    best_pos, best_deltas, history = run_pso_timeopt(n_particles=300, iters=500, seed=42)
    metrics = validate_trajectory_timeopt(best_pos, best_deltas)
    print('Metrics:', metrics)
    # guardar CSV de ejemplo
    save_trajectories_csv_timeopt(best_pos, best_deltas)

    # Mostrar y/o guardar gráficas. Guardamos siempre en output/ y solo mostramos si hay DISPLAY.
    has_display = bool(os.environ.get('DISPLAY'))
    if not has_display:
        print('No DISPLAY detected: guardando gráficas en output/ (modo headless)')
    plot_timeopt_results(best_pos, best_deltas, history=history, show=has_display, save_prefix='timeopt_demo', sample_num=200)

