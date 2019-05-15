# -*- coding: utf-8 -*-

import math
import itertools
import numpy as np
import random
import os
import time
import pulp as lp
from copy import deepcopy
from datetime import timedelta
from dijkstra import *
from fleury import *
from collections import defaultdict

# you must to change folder path to working properly
path = "your path"
os.chdir(path)


def cpp(E, graph):

    status = False

    # find odd vertexes to matching problem
    odd_vertexes = find_odd_vertexes(E)
    if len(odd_vertexes) == 0:

        status = True
        print 'Graph is Eulerian yet...'

        match_result = 0
        #tour = find_euler_tour(E)

        # Fleury algo to find Eulerian Path
        tour = find_euler_tour(E)
        final_path, final_cost = computeCosts(graph, tour)

    else:

        edge_set, weight_edge_set, path_set = split_odd_graph(
            graph, odd_vertexes)

        # formulate and solve matching problem in odd vertexes with Linear Problem Solver
        match_edges = linear_matching(edge_set, weight_edge_set, odd_vertexes)

        match_result = 0
        # search for paths in path_set
        for i in range(len(match_edges)):
            path_size = len(path_set[match_edges[i]])
            if path_size < 3:
                E.append(
                    (path_set[match_edges[i]][0], path_set[match_edges[i]][1], graph[path_set[match_edges[i]][0]][path_set[match_edges[i]][1]]))
                match_result += graph[path_set[match_edges[i]]
                                      [0]][path_set[match_edges[i]][1]]
            else:
                for j in range(path_size-1):
                    # aux_edge_list.append(
                    #    (path_set[match_edges[i]][j], path_set[match_edges[i]][j+1], graph[path_set[match_edges[i]][j]][path_set[match_edges[i]][j+1]]))
                    E.append(
                        (path_set[match_edges[i]][j], path_set[match_edges[i]][j+1], graph[path_set[match_edges[i]][j]][path_set[match_edges[i]][j+1]]))
                    match_result += graph[path_set[match_edges[i]]
                                          [j]][path_set[match_edges[i]][j+1]]

        # Fleury algo to find Eulerian Path
        tour = find_euler_tour(E)
        final_path, final_cost = computeCosts(graph, tour)

    return status, final_path, final_cost, final_cost-match_result, match_result


def computeCosts(graph, tour):

    cost = 0
    path = []
    for i in range(len(tour)-1):
        path.append((tour[i], tour[i+1]))
        cost += graph[tour[i]][tour[i+1]]

    return path, cost


def find_tour(u, tour):
    for e in range(len(E)):
        if E[e] == 0:
            continue
        if u == E[e][0]:
            u, v, w = E[e]
            E[e] = 0
            find_tour(v, tour)
        elif u == E[e][1]:
            v, u, w = E[e]
            E[e] = 0
            find_tour(v, tour)
    tour.insert(0, u)

    return tour


def find_euler_tour(graph):

    E = graph
    tour = []
    numEdges = defaultdict(int)

    for i, j, k in graph:
        numEdges[i] += 1
        numEdges[j] += 1

    start = graph[0][0]
    for i, j in numEdges.iteritems():
        if j % 2 > 0:
            start = i
            break

    current = start
    tour = find_tour(current, tour)

    if tour[0] != tour[-1]:
        return None

    return tour


def load_file(filename):

    f = open(filename)
    header = f.readline()
    n_vertex, n_edges = header.split()

    #weight_sum = 0
    # instantiate distance matrix for build graph
    graph = [[0 for j in range(int(n_vertex))]
             for i in range(int(n_vertex))]

    # instatiate set of edges E
    E = []
    flag = False
    for i in range(int(n_edges)):
        line = f.readline()
        index, v_in, v_out, wt = line.split(',')

        if i == 0 and int(v_in) == 0:
            flag = True

        if flag:
            v_in = int(v_in)
            v_out = int(v_out)
        else:
            v_in = int(v_in)-1
            v_out = int(v_out)-1

        graph[v_in][v_out] = int(wt)
        graph[v_out][v_in] = graph[v_in][v_out]
        E.append((v_in, v_out, int(wt)))

        #weight_sum += int(wt)
        if line.find("EOF") != -1 or not line:
            break

    return E, graph


def find_odd_vertexes(E):

    tmp_g = {}
    vertexes = []
    for edge in E:
        if edge[0] not in tmp_g:
            tmp_g[edge[0]] = 0

        if edge[1] not in tmp_g:
            tmp_g[edge[1]] = 0

        tmp_g[edge[0]] += 1
        tmp_g[edge[1]] += 1

    for vertex in tmp_g:
        if tmp_g[vertex] % 2 == 1:
            vertexes.append(vertex)

    return vertexes


def split_odd_graph(graph, odd_vert):

    path_list = []
    edge_list = []
    odd_G = []

    # search in every pair of odd vertexes: distances from source to sink and path between them
    for vert in odd_vert:
        # odd_graph_dist, path = dijkstra(odd_graph_list, vert)
        odd_graph_dist, path = dijkstra(graph, vert)
        path_list.append(path)
        edge = [(path[i][0], path[i][-1]) for i in range(len(path))]
        edge_list.append(edge)
        odd_G.append(odd_graph_dist)

    edge_odd_list = [
        val for sublist in edge_list for val in sublist]
    weight_odd_list = [
        val for sublist in odd_G for val in sublist]
    path_odd_list = [
        val for sublist in path_list for val in sublist]

    return edge_odd_list, weight_odd_list, path_odd_list


def linear_matching(edge_set, weight_edge_set, odd_vert):

    prob = lp.LpProblem("ChinesePostmanProblem", lp.LpMinimize)

    # edge_vars: list of edges index for LP model
    edge_vars = [i for i in range(len(edge_set))]

    # edge_use_var: equals 1 if only edge is present in matching
    edge_use_vars = lp.LpVariable.dicts(
        "UseEdge", edge_vars, 0, 1, lp.LpBinary)

    # create a objective function
    prob += lp.lpSum(weight_edge_set[i] * edge_use_vars[i] for i in edge_vars)

    # deep copy from odd_vert
    aux_list = deepcopy(odd_vert)
    source = aux_list[:: -1]

    # build a subject matrix by selecting edge indexes
    edge_split_set = []
    while source:
        index = source.pop()
        edge_aux = []
        for i in edge_vars:
            # if (edge_set[i][0] == index) or (edge_set[i][1] == index):
            if (((edge_set[i][0] == index) and (edge_set[i][1] in odd_vert)) or ((edge_set[i][1] == index) and edge_set[i][0] in odd_vert)):
                edge_aux.append(i)

        edge_split_set.append(edge_aux)

    # insert connected edges into prob subjects
    for row in edge_split_set:
        prob += lp.lpSum(edge_use_vars[row[i]]
                         for i in range(len(row))) == 1

    prob.solve()

    solution = []
    for i in range(len(edge_set)):
        if edge_use_vars[i].varValue:
            solution.append(i)
            print "You must use", edge_set[i], "to beat a perfect minimum match"

    return solution


if __name__ == "__main__":

    #filename = "g_08_13"
    #filename = "g_09_18"
    #filename = "g_10_26"
    #filename = "g_10_38"
    #filename = "g_20_42"
    filename = "g_100_3008"  # recursion limit

    E, G = load_file(filename+".txt")

    start_time = time.time()
    status, route, total_cost, route_cost, match_result = cpp(E, G)
    elapsed_time_secs = time.time() - start_time

    with open(filename+'_route.txt', 'a+') as f:
        f.write("Instance: " + str(filename)+".txt" + "\n")
        f.write("Is already Eulerian? " + str(status) + "\n")
        f.write("Elapsed time: " + str(elapsed_time_secs) + " seconds \n")
        f.write("Total cost found: " + str(total_cost) + " ")
        f.write("Route cost: " + str(route_cost) + " ")
        f.write("Match cost: " + str(match_result) + "\n")
        f.write("Sugested route:")
        for i in range(len(route)):
            f.write(" "+str(route[i]))
        f.write("\n\n")

    f.close()
