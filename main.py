import heapq
import itertools
import math

def distanciaEuclidiana(x1, y1, x2, y2):
    return math.sqrt(pow((x2 - x1),2) + pow((y2 - y1),2))

def dijkstra(puntosPlano, empezar):
    distances = {node: float('infinity') for node in puntosPlano}
    distances[empezar] = 0
    priority_queue = [(0, empezar)]

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)
        for neighbor, weight in puntosPlano[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))

    return distances

def encontrarCamino(graph, start, end):
    nodes = list(graph.keys())
    nodes.remove(start)
    nodes.remove(end)
    min_cost = float('infinity')
    best_path = None

    for intermediates in itertools.permutations(nodes, 2):
        path_cost = 0
        current_start = start
        for node in intermediates + (end,):
            distances = dijkstra(graph, current_start)
            path_cost += distances[node]
            current_start = node
        if path_cost < min_cost:
            min_cost = path_cost
            best_path = (start,) + intermediates + (end,)

    return min_cost, best_path

# Coordenadas de los puntos
coordenadas = {
    'P1': (1, 1),
    'P2': (2, 6),
    'P3': (4, 7),
    'P4': (6, 4),
    'P5': (5, -2)
}

# Crear el grafo con las distancias Euclidianas
puntosPlano = {point: {} for point in coordenadas}
nodos = [('P1', 'P2'), ('P1', 'P3'), ('P1', 'P4'), ('P1', 'P5'), ('P2', 'P1'), ('P2', 'P3'), ('P2', 'P4'), ('P2', 'P5'), ('P3', 'P1'), ('P3', 'P2'), ('P3', 'P4'), ('P3', 'P5'), ('P4', 'P1'), ('P4', 'P2'), ('P4', 'P3'),('P4', 'P5'), ('P5', 'P1'), ('P5', 'P2'), ('P5', 'P3'),('P5', 'P4')]

for (p1, p2) in nodos:
    distancia = distanciaEuclidiana(*coordenadas[p1], *coordenadas[p2])
    puntosPlano[p1][p2] = distancia
    puntosPlano[p2][p1] = distancia

# Calcula el mejor camino desde P1 a P5
cost, path = encontrarCamino(puntosPlano, 'P2', 'P1')
print(f"El camino más corto es {path} con una distancia de {cost:.2f} cm")