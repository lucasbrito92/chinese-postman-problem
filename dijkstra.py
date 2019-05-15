import numpy as np
from copy import deepcopy


def minDistance(dist, queue):

    minimum = float("Inf")
    min_index = -1

    for i in range(len(dist)):
        if dist[i] < minimum and i in queue:
            minimum = dist[i]
            min_index = i
    return min_index


def getParent(parent, j, path):

    if parent[j] == -1:
        path.append(j,)
        return
    getParent(parent, parent[j], path)
    path.append(j,)

    return path


def getSolution(dist, parent, src, path):

    paths = []
    # print("Vertex \t\tDistance from Source\tPath")
    for i in range(len(dist)):
        # print(dist[i])
        list = []
        # print("\n%d --> %d \t\t%d \t\t\t\t\t" % (src, i, dist[i])),
        if i != src:
            list = getParent(parent, i, deepcopy(path))
            paths.append(deepcopy(list))

    return paths


def dijkstra(graph, src):

    row = len(graph)
    col = len(graph[0])

    dist = [float("Inf")] * row
    parent = [-1] * row
    dist[src] = 0

    queue = []
    for i in range(row):
        queue.append(i)

    while queue:
        u = minDistance(dist, queue)
        queue.remove(u)
        for i in range(col):
            if graph[u][i] and i in queue:
                if dist[u] + graph[u][i] < dist[i]:
                    dist[i] = dist[u] + graph[u][i]
                    parent[i] = u
    path = []
    all_paths = getSolution(dist, parent, src, path)
    aux_dist = deepcopy(dist)

    # removing redudant distances
    for i in range(len(dist)):
        if i < src+1:
            aux_dist.remove(dist[i])

    # return path from src to sink and remove redudant paths
    return aux_dist, all_paths[src:]
