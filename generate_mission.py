import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path

# 1. Configuración
min_x, max_x = -19, 19
min_y, max_y = -19, 19
cell_size = 2.0
num_agents = 5
min_separation = 5.0  # Distancia mínima entre agentes en metros

# Calcular número de filas y columnas
# El rango es max - min. Si es 22 y cell_size es 2, son 11 celdas.
cols = int(round((max_x - min_x) / cell_size))
rows = int(round((max_y - min_y) / cell_size))

print(f"Grid dimensions: {rows} rows x {cols} cols")
print(f"Cell size: {cell_size} m")

# Inicializar grid (0 = libre, 1 = obstáculo)
grid = np.zeros((rows, cols), dtype=int)

# 2. Generación del área segura (elipsoide irregular)
a = 16.0  # radio en x
b = 14.0  # radio en y
center_x, center_y = 2.0, 2.0
num_boundary_points = 100

theta = np.linspace(0, 2*np.pi, num_boundary_points, endpoint=False)
x_boundary = center_x + a * np.cos(theta)
y_boundary = center_y + b * np.sin(theta)

# Añadir ruido (irregularidad)
np.random.seed(42)
noise_scale = 0.8  # Un poco más de ruido dado el tamaño de celda
x_boundary += np.random.normal(0, noise_scale, num_boundary_points)
y_boundary += np.random.normal(0, noise_scale, num_boundary_points)

# Crear polígono para chequeo "dentro/fuera"
boundary_points = np.vstack((x_boundary, y_boundary)).T
poly_path = Path(boundary_points)

# 3. Mapeo a Celdas: Llenar obstáculos fuera del elipsoide
# Iteramos por cada celda para ver si su centro está dentro o fuera
for r in range(rows):
    for c in range(cols):
        # Calcular centro de la celda
        # x = min_x + (c * cell_size) + (cell_size / 2)
        cx = min_x + c * cell_size + cell_size / 2.0
        cy = min_y + r * cell_size + cell_size / 2.0
        
        # Si el centro está FUERA del polígono, es obstáculo
        if not poly_path.contains_point((cx, cy)):
            grid[r, c] = 1

# 4. Generación de Blobs Internos (Obstáculos dentro del área segura)
blobs = []

# Función auxiliar para añadir blob
def add_blob(center, radius, num_points):
    points = []
    for _ in range(num_points):
        r_dist = np.random.uniform(0, radius)
        angle = np.random.uniform(0, 2*np.pi)
        px = center[0] + r_dist * np.cos(angle)
        py = center[1] + r_dist * np.sin(angle)
        points.append((px, py))
    return points

# Definir blobs (ajustados para no estar muy cerca del borde si es posible)
# Blob 1: Cuadrante superior derecho
blobs.extend(add_blob((4, 3), 3, 10))
# Blob 2: Cuadrante inferior izquierdo
blobs.extend(add_blob((-4, -3), 3, 10))
# Blob 3: Cerca del centro arriba
blobs.extend(add_blob((0, 5), 2, 5))

# Mapear blobs a celdas
for bx, by in blobs:
    # Encontrar índices de celda
    c = int((bx - min_x) / cell_size)
    r = int((by - min_y) / cell_size)
    
    # Validar límites
    if 0 <= c < cols and 0 <= r < rows:
        grid[r, c] = 1

# 5. Extraer coordenadas finales de obstáculos (Centros de celdas)
final_x = []
final_y = []

for r in range(rows):
    for c in range(cols):
        if grid[r, c] == 1:
            cx = min_x + c * cell_size + cell_size / 2.0
            cy = min_y + r * cell_size + cell_size / 2.0
            final_x.append(round(cx, 2))
            final_y.append(round(cy, 2))

print(f"Total obstacle cells: {len(final_x)}")
print("obstacles_positions_x:", final_x)
print("obstacles_positions_y:", final_y)

# 6. Generación de posiciones iniciales de agentes
agent_positions = []
free_cells = []

# Identificar celdas libres
for r in range(rows):
    for c in range(cols):
        if grid[r, c] == 0:
            cx = min_x + c * cell_size + cell_size / 2.0
            cy = min_y + r * cell_size + cell_size / 2.0
            free_cells.append((cx, cy))

# Seleccionar posiciones aleatorias separadas
attempts = 0
max_attempts = 1000

while len(agent_positions) < num_agents and attempts < max_attempts:
    attempts += 1
    candidate = free_cells[np.random.randint(len(free_cells))]
    
    # Verificar separación con agentes ya seleccionados
    valid = True
    for pos in agent_positions:
        dist = np.sqrt((candidate[0] - pos[0])**2 + (candidate[1] - pos[1])**2)
        if dist < min_separation:
            valid = False
            break
    
    if valid:
        agent_positions.append(candidate)

if len(agent_positions) < num_agents:
    print(f"Warning: Could only place {len(agent_positions)} agents with min separation {min_separation}m")

agent_x = [round(p[0], 2) for p in agent_positions]
agent_y = [round(p[1], 2) for p in agent_positions]

print(f"Initial positions generated: {len(agent_positions)}")
print("initial_positions_x:", agent_x)
print("initial_positions_y:", agent_y)

# 7. Visualización
plt.figure(figsize=(10, 10))

# Extent para imshow: [left, right, bottom, top]
# Grid se dibuja desde abajo (origin='lower')
plt.imshow(grid, origin='lower', extent=[min_x, max_x, min_y, max_y], 
           cmap='Greys', vmin=0, vmax=2, alpha=0.5)

# Dibujar centros de obstáculos
plt.scatter(final_x, final_y, c='red', s=30, marker='s', label='Obstacle Centers')

# Dibujar posiciones iniciales de agentes
plt.scatter(agent_x, agent_y, c='blue', s=100, marker='*', label='Agent Start Positions')
for i, (ax, ay) in enumerate(zip(agent_x, agent_y)):
    plt.text(ax + 0.5, ay + 0.5, f"A{i+1}", color='blue', fontweight='bold')

# Dibujar contorno del elipsoide generado
plt.plot(np.append(x_boundary, x_boundary[0]), np.append(y_boundary, y_boundary[0]), 
         'b--', linewidth=2, label='Boundary (Area Limit)')

# Configuración de ejes
plt.xlim(min_x, max_x)
plt.ylim(min_y, max_y)
plt.xticks(np.arange(min_x, max_x + 1, cell_size))
plt.yticks(np.arange(min_y, max_y + 1, cell_size))
plt.grid(True, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
plt.title(f'Obstacle Map (Cell Size: {cell_size}m)')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.legend(loc='upper right')
plt.gca().set_aspect('equal')

plt.show()