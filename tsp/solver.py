#!/usr/bin/python
# -*- coding: utf-8 -*-

import math
from collections import namedtuple

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

Point = namedtuple("Point", ['x', 'y', 'index'])

def length(point1, point2):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)


def compute_distance_matrix(nodes):
    num_points = len(nodes)
    dist_matrix = [[0.0] * num_points for i in range(num_points)]
    for n_1 in nodes:
        for n_2 in nodes:
            dist = length(n_1, n_2)
            dist_matrix[n_1.index][n_2.index] = dist
            dist_matrix[n_1.index][n_2.index] = dist
    return dist_matrix


def optimize(data, num_vehicles=1, depot=0):
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data[from_node][to_node]

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data), num_vehicles, depot)

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    '''
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
       routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    '''
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.TABU_SEARCH)
    search_parameters.time_limit.seconds = 300
    #search_parameters.log_search = True

    # Solve the problem.
    obj_value = 0
    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        obj_value = solution.ObjectiveValue()
        path = []
        index = routing.Start(0)
        while not routing.IsEnd(index):
            path.append(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
    return obj_value, path

def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    nodeCount = int(lines[0])

    points = []
    for i in range(1, nodeCount+1):
        line = lines[i]
        parts = line.split()
        points.append(Point(float(parts[0]), float(parts[1]), i-1))

    # build a trivial solution
    # visit the nodes in the order they appear in the file
    data = compute_distance_matrix(points)
    obj, solution = optimize(data)

    # calculate the length of the tour
    #obj = length(points[solution[-1]], points[solution[0]])
    #for index in range(0, nodeCount-1):
    #    obj += length(points[solution[index]], points[solution[index+1]])

    # prepare the solution in the specified output format
    output_data = '%.2f' % obj + ' ' + str(0) + '\n'
    output_data += ' '.join(map(str, solution))

    return output_data


import sys

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        print(solve_it(input_data))
    else:
        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/tsp_51_1)')

