import heapq
import itertools
import math

def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt(pow((x2 - x1),2) + pow((y2 - y1),2))

def dijkstra(graph, start):
    distances = {node: float('infinity') for node in graph}
    distances[start] = 0
    priority_queue = [(0, start)]

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))

    return distances

def find_best_path(graph, start, end):
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
coordinates = {
    'P1': (1, 1),
    'P2': (2, 6),
    'P3': (4, 7),
    'P4': (6, 4),
    'P5': (5, -2)
}

# Crear el grafo con las distancias Euclidianas
graph = {point: {} for point in coordinates}
edges = [('P1', 'P2'), ('P1', 'P3'), ('P1', 'P4'), ('P1', 'P5'), ('P2', 'P1'), ('P2', 'P3'), ('P2', 'P4'), ('P2', 'P5'), ('P3', 'P1'), ('P3', 'P2'), ('P3', 'P4'), ('P3', 'P5'), ('P4', 'P1'), ('P4', 'P2'), ('P4', 'P3'),('P4', 'P5'), ('P5', 'P1'), ('P5', 'P2'), ('P5', 'P3'),('P5', 'P4')]

for (p1, p2) in edges:
    distance = euclidean_distance(*coordinates[p1], *coordinates[p2])
    graph[p1][p2] = distance
    graph[p2][p1] = distance

# Calcula el mejor camino desde P1 a P5
cost, path = find_best_path(graph, 'P2', 'P1')
print(f"El camino más corto es {path} con una distancia de {cost:.2f} cm")