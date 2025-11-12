"""
coverage_lawnmower.py
Coverage Path Planning determinista (lawnmower) para 1 UAV en 2D con zonas prohibidas.
Genera y muestra la trayectoria, y puede guardar la ruta a CSV.
Uso: python coverage_lawnmower.py

Autor: José Antonio García Campanario
Fecha: 2025-10-31
"""

import numpy as np
import matplotlib.pyplot as plt
import heapq
import os
import csv

# --- Parámetros del escenario ---
GRID_W = 40  # celdas en X
GRID_H = 30  # celdas en Y
CELL_SIZE = 1.0  # metros por celda
ALTITUDE = 10.0  # altura de vuelo (no usada en la planificacion 2D)
SENSOR_RADIUS = 1.0  # radio de cobertura (en metros)

# Obstáculos: lista de rectángulos (x_min, y_min, x_max, y_max) en coordenadas de celdas
OBSTACLES = [
    (10, 5, 14, 12),
    (22, 18, 30, 22),
    (5, 20, 9, 28),
]

START = (0, 0)  # celda de inicio (x,y)
OUTPUT_DIR = 'output_lawnmower'


# --- Utilidades de grilla ---
def make_grid(w, h, obstacles):
    grid = np.zeros((h, w), dtype=np.uint8)  # 0 libre, 1 obstáculo
    for (xmin, ymin, xmax, ymax) in obstacles:
        xmin = max(0, int(xmin)); ymin = max(0, int(ymin))
        xmax = min(w-1, int(xmax)); ymax = min(h-1, int(ymax))
        grid[ymin:ymax+1, xmin:xmax+1] = 1
    return grid

# A* en grilla 4-conectada
def astar(grid, start, goal):
    h, w = grid.shape
    def h_cost(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])
    open_set = []
    heapq.heappush(open_set, (0 + h_cost(start,goal), 0, start, None))
    came_from = {}
    gscore = {start: 0}
    while open_set:
        f, g, current, parent = heapq.heappop(open_set)
        if current in came_from:
            continue
        came_from[current] = parent
        if current == goal:
            # reconstruir ruta
            path = []
            cur = current
            while cur is not None:
                path.append(cur)
                cur = came_from[cur]
            return path[::-1]
        x,y = current
        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < w and 0 <= ny < h and grid[ny,nx] == 0:
                neighbor = (nx,ny)
                tentative_g = g + 1
                if neighbor in gscore and tentative_g >= gscore[neighbor]:
                    continue
                gscore[neighbor] = tentative_g
                heapq.heappush(open_set, (tentative_g + h_cost(neighbor,goal), tentative_g, neighbor, current))
    return None  # no path

# Genera patrón lawnmower por filas considerando obstáculos: intentamos barrer fila por fila y usar A* para sortear huecos
def lawnmower_with_obstacles(grid, start):
    h, w = grid.shape
    path = []
    # ordenar filas de abajo a arriba
    row_order = range(h)
    cur = start
    for ri in row_order:
        # decidir dirección en la fila
        if ri % 2 == 0:
            col_range = range(0, w)
        else:
            col_range = range(w-1, -1, -1)
        # construir lista de celdas objetivo en esta fila que no son obstáculo
        targets = [(c, ri) for c in col_range if grid[ri, c] == 0]
        for t in targets:
            # si ya estamos en el objetivo, aseguramos que esté en la ruta
            if cur == t:
                if not path or path[-1] != cur:
                    path.append(cur)
                continue
            # planear camino local con A*
            p = astar(grid, cur, t)
            if p is None:
                # si no hay camino directo (sección inaccesible), saltamos ese objetivo
                continue
            # añadir siempre los pasos del camino A* (evitar saltos visuales por
            # haber ignorado nodos previamente visitados). Evitamos duplicados
            # consecutivos comprobando el último elemento.
            for step in p[1:]:
                if not path or path[-1] != step:
                    path.append(step)
            cur = path[-1] if path else cur
    return path

def cells_covered_by_path(path, sensor_radius, cell_size):
    """Return set of covered cell coordinates.

    Now accepts grid bounds implicitly via GRID_W/GRID_H or the caller can
    pass a path that is within those bounds.
    """
    covered = set()
    if not path:
        return covered
    r_cells = int(np.ceil(sensor_radius / cell_size))
    for (x, y) in path:
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                cx = x + dx; cy = y + dy
                if 0 <= cx < GRID_W and 0 <= cy < GRID_H:
                    # distancia euclidea
                    if np.hypot(dx * cell_size, dy * cell_size) <= sensor_radius + 1e-9:
                        covered.add((cx, cy))
    return covered

def save_path_csv(path, prefix='lawnmower'):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    fname = os.path.join(OUTPUT_DIR, f'{prefix}_path.csv')
    with open(fname, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x_cell','y_cell','x_m','y_m'])
        if not path:
            print('Aviso: la ruta está vacía. Se guardó sólo el encabezado en', fname)
        for (x, y) in path:
            writer.writerow([x, y, x * CELL_SIZE + CELL_SIZE / 2.0, y * CELL_SIZE + CELL_SIZE / 2.0])
    print('Guardado:', fname)

def plot_grid_and_path(grid, path, covered_cells):
    fig, ax = plt.subplots(figsize=(10,7))
    ax.set_title('Coverage - LawnMower con Obstáculos')
    # dibujar obstaculos
    ax.imshow(grid, origin='lower', cmap='gray_r', extent=(0, GRID_W, 0, GRID_H))
    # cubrir celdas cubiertas
    cov_x = [c[0] + 0.5 for c in covered_cells]
    cov_y = [c[1] + 0.5 for c in covered_cells]
    if covered_cells:
        ax.scatter(cov_x, cov_y, s=10)
    # trayectoria
    if path:
        xs = [p[0] + 0.5 for p in path]
        ys = [p[1] + 0.5 for p in path]
        ax.plot(xs, ys, linewidth=1)
        ax.scatter(xs[0], ys[0], marker='o', label='start')
        ax.scatter(xs[-1], ys[-1], marker='x', label='end')
    else:
        ax.text(0.5 * GRID_W, 0.5 * GRID_H, 'No path generated', ha='center', va='center', color='red')
    ax.set_xlim(0, GRID_W)
    ax.set_ylim(0, GRID_H)
    ax.set_xlabel('X (cells)'); ax.set_ylabel('Y (cells)')
    ax.legend()
    plt.show()

def main():
    grid = make_grid(GRID_W, GRID_H, OBSTACLES)
    # si start es obstáculo, buscar nearest free
    sx, sy = START
    if grid[sy, sx] == 1:
        # buscar la celda libre más cercana por BFS
        from collections import deque
        q = deque()
        q.append((sx, sy))
        seen = set([(sx, sy)])
        found = None
        while q:
            x, y = q.popleft()
            for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < GRID_W and 0 <= ny < GRID_H and (nx, ny) not in seen:
                    if grid[ny, nx] == 0:
                        found = (nx, ny)
                        q.clear()
                        break
                    seen.add((nx, ny))
                    q.append((nx, ny))
        if found is None:
            raise ValueError('No free start cell found; toda la grilla está ocupada')
        else:
            print(f'Inicio {START} estaba dentro de obstáculo; usando start cercano {found}')
            start = found
    else:
        start = START
    path = lawnmower_with_obstacles(grid, start)
    covered = cells_covered_by_path(path, SENSOR_RADIUS, CELL_SIZE)
    coverage_ratio = len(covered) / np.sum(grid==0)
    print(f'Celulas libres: {np.sum(grid==0)}, cubiertas (aprox): {len(covered)}, cobertura: {coverage_ratio*100:.2f}%')
    save_path_csv(path)
    plot_grid_and_path(grid, path, covered)

if __name__ == '__main__':
    main()
