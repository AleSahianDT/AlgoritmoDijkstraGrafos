import heapq
import itertools
import math

def distanciaEuclidiana(x1, y1, x2, y2):
    return math.sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2))

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

    for intermediates in itertools.permutations(nodes):
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

def crearGrafo(coordenadas):
    puntosPlano = {point: {} for point in coordenadas}
    nodos = list(itertools.permutations(coordenadas.keys(), 2))
    for (p1, p2) in nodos:
        distancia = distanciaEuclidiana(*coordenadas[p1], *coordenadas[p2])
        puntosPlano[p1][p2] = distancia
    return puntosPlano

coordenadas = {}
for i in range(1, 6):
    x = float(input(f"Ingrese la coordenada x del punto P{i}: "))
    y = float(input(f"Ingrese la coordenada y del punto P{i}: "))
    coordenadas[f'P{i}'] = (x, y)

start = input("Ingrese el punto de inicio (P1, P2, P3, P4, P5): ")
end = input("Ingrese el punto final (P1, P2, P3, P4, P5): ")

puntosPlano = crearGrafo(coordenadas)

cost, path = encontrarCamino(puntosPlano, start, end)
print(f"El camino más corto es {path} con una distancia de {cost:.2f} cm")