

# Script para calcular regiones de Voronoi en un rectángulo usando solo Python estándar
# No requiere Matplotlib ni SciPy

def distancia(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

def voronoi_text(rect, points, grid_size=(20, 10)):
    xmin, xmax, ymin, ymax = rect
    nx, ny = grid_size
    dx = (xmax - xmin) / (nx - 1)
    dy = (ymax - ymin) / (ny - 1)
    grid = []
    for j in range(ny):
        row = []
        y = ymin + j * dy
        for i in range(nx):
            x = xmin + i * dx
            dists = [distancia((x, y), p) for p in points]
            owner = dists.index(min(dists))
            row.append(str(owner))
        grid.append(row)
    return grid

def print_voronoi_grid(grid):
    for row in grid[::-1]:  # Imprimir de arriba a abajo
        print(' '.join(row))

if __name__ == "__main__":
    # Definir rectángulo y puntos
    rect = (0, 10, 0, 5)  # xmin, xmax, ymin, ymax
    points = [
        (2, 2),
        (8, 1),
        (5, 4)
    ]
    grid = voronoi_text(rect, points, grid_size=(20, 10))
    print("Regiones de Voronoi (cada número representa el punto más cercano):")
    print_voronoi_grid(grid)