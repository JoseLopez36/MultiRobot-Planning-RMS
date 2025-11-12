import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# Load CSVs saved by the C++ program: output/uav_traj_cpp_uav{i}.csv and output/history_cpp.csv

def load_trajs(prefix='output/uav_traj_cpp_uav'):
    files = sorted([f for f in os.listdir('output') if f.startswith('uav_traj_cpp_uav') and f.endswith('.csv')])
    trajs = []
    times = []
    for f in files:
        data = np.loadtxt(os.path.join('output', f), delimiter=',', skiprows=1)
        t = data[:,0]
        xyzt = data[:,1:]
        trajs.append(xyzt)
        times.append(t)
    return times, trajs

def load_history(fname='output/history_cpp.csv'):
    data = np.loadtxt(fname, delimiter=',', skiprows=1)
    return data[:,0], data[:,1]

if __name__ == '__main__':
    times, trajs = load_trajs()
    iters, costs = load_history()
    # 3D plot
    fig = plt.figure(figsize=(10,6))
    ax = fig.add_subplot(111, projection='3d')
    for i, tr in enumerate(trajs):
        ax.plot(tr[:,0], tr[:,1], tr[:,2], label=f'UAV {i}')
        ax.scatter(tr[0,0], tr[0,1], tr[0,2], marker='o')
        ax.scatter(tr[-1,0], tr[-1,1], tr[-1,2], marker='^')
    ax.legend()
    plt.show()

    # pairwise distances over common grid (use union of times)
    grid = np.unique(np.concatenate(times))
    samples = []
    for i, tr in enumerate(trajs):
        t = times[i]
        # simple linear interp
        samples_i = np.zeros((grid.size, 3))
        for j, tg in enumerate(grid):
            if tg <= t[0]: samples_i[j] = tr[0]
            elif tg >= t[-1]: samples_i[j] = tr[-1]
            else:
                idx = np.searchsorted(t, tg) - 1
                t0, t1 = t[idx], t[idx+1]
                a = (tg - t0) / (t1 - t0 + 1e-12)
                samples_i[j] = (1-a)*tr[idx] + a*tr[idx+1]
        samples.append(samples_i)
    pairs = []
    for i in range(len(samples)):
        for j in range(i+1, len(samples)):
            pairs.append((i,j))
    fig2, ax2 = plt.subplots(figsize=(10,4))
    for (i,j) in pairs:
        dists = np.linalg.norm(samples[i] - samples[j], axis=1)
        ax2.plot(grid, dists, label=f'{i}-{j}')
    ax2.axhline(2.0, linestyle='--', color='k')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Distance')
    ax2.legend()
    plt.show()

    # cost history
    fig3, ax3 = plt.subplots(figsize=(8,3))
    ax3.plot(iters, costs)
    ax3.set_xlabel('Iter')
    ax3.set_ylabel('Cost')
    plt.show()
