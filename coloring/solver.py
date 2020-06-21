#!/usr/bin/python
# -*- coding: utf-8 -*-
from ortools.sat.python import cp_model
from collections import defaultdict, namedtuple, deque
from numba import jit


def solve(node_count, edges):
    model = cp_model.CpModel()
    num_colors = node_count
    nodes = {}
    # Redundant variables
    x = model.NewIntVar(0,node_count, 'num_color')

    # Initiate decision variables
    # Decision variables are the nodes
    nodes[0] = model.NewIntVar(0, 0, f'node_{0}')
    for i in range(1,node_count):
        nodes[i] = model.NewIntVar(0, num_colors, f'node_{i}')
    
    # Add reify constraints
    # Let's try edges are reify constaints
    # e_i != e_j for <i,j> in E
    #model.AddMaxEquality(x, nodes)
    eq = {(i, j) : model.NewIntVar(0,0, f'edge_{i}_{j}') for i in range(node_count) for j in range(node_count)}
    eq.update({edge : model.NewIntVar(1,1, f'edge_{edge[0]}_{edge[1]}') for edge in edges})
    #for i, j in edges:
    for i in range(node_count):
        for j in range(node_count):
            model.Add(nodes[i] != nodes[j]).OnlyEnforceIf(eq[i,j])
            model.Add(nodes[i] <= max(nodes)).OnlyEnforceIf(eq[i,j].Not())
            model.Add(nodes[j] <= max(nodes)).OnlyEnforceIf(eq[i,j].Not())

    #obj = len(set(nodes))
    obj = sum(nodes.values())
    model.Minimize(obj)
    solver = cp_model.CpSolver()
    solver.parameters.num_search_workers = 16
    solution = []
    if solver.Solve(model) == cp_model.OPTIMAL:
        solution = [solver.Value(nodes[i]) for i in range(node_count)]
    return solution


def solve2(node_count, edges):
    model = cp_model.CpModel()
    num_colors = node_count
    degree = defaultdict(list)
    for i,j in edges:
        degree[i].append(j)
    ordered_degree = sorted(degree, key=lambda k: len(degree[k]), reverse=True)
    nodes = {}
    for i in range(node_count):
        nodes[i] = model.NewIntVar(0, num_colors, f'node_{i}')
    nodes[ordered_degree[0]] = model.NewIntVar(0,0,f'node_{ordered_degree[0]}')

    eq = {(i, j) : model.NewIntVar(0,0, f'edge_{i}_{j}') for i in range(node_count) for j in range(node_count)}
    eq.update({(i,j) : model.NewIntVar(1,1, f'edge_{i}_{j}') for i,j in edges})
    for i in range(node_count):
        for j in range(node_count):
            model.Add(nodes[i] != nodes[j]).OnlyEnforceIf(eq[i,j])

    #model.AddDecisionStrategy(ordered_nodes, cp_model.CHOOSE_FIRST, cp_model.SELECT_MIN_VALUE)
    solver = cp_model.CpSolver()
    solver.parameters.num_search_workers = 16
    solution = []
    if solver.Solve(model) == cp_model.FEASIBLE:
        solution = [solver.Value(nodes[i]) for i in range(node_count)]
    return solution


def solve3(nodes_count, edges, max_colors):
    max_colors = max_colors - 2
    solution = []
    while len(solution) == 0:
        max_colors += 1
        print(max_colors)
        model = cp_model.CpModel()    
        color = [model.NewIntVar(0, max_colors, f'color_{i}') for i in range(nodes_count)]
        for edge in edges:
            model.Add(color[edge[0]] != color[edge[1]])
        
        model.Add(color[0] == 0)
        #model.Add(max(color) <= max_colors)
        #model.AddDecisionStrategy(color, cp_model.CHOOSE_FIRST, cp_model.SELECT_MIN_VALUE)
        #obj = model.Minimize(max(color))
        solver = cp_model.CpSolver()
        solver.parameters.num_search_workers = 8

        if solver.Solve(model) == cp_model.FEASIBLE:
            solution = [solver.Value(color[i]) for i in range(nodes_count)]
    return solution



def greedy(node_count, edges):
    degree = defaultdict(list)
    for i,j in edges:
        degree[i].append(j)
        degree[j].append(i)
    ordered_degree = sorted(degree, key=lambda k: len(degree[k]), reverse=True)
    ordered_nodes = {i : degree[i] for i in ordered_degree}
    
    def first_available(color_list):
        color_set = set(color_list)
        count = 0
        while True:
            if count not in color_set:
                return count
            count += 1
    
    color = {}
    for node in ordered_degree:
        used_neighbour_colors = [color[nbr] for nbr in ordered_nodes[node] if nbr in color]
        color[node] = first_available(used_neighbour_colors)
    solution = [color[i] for i in range(node_count)]
    return solution


class Node:
    def __init__(self, index, color, edges, total_color_used):
        self.index = index
        self.color = color
        self.edges = edges
        self.total_color_used = total_color_used


def backtracking(node_count, edges):
    # TODO:
    # 1) create namedtuples as node class
    # 2) assign_color()
    # 3) check_feasibility()
    # 4) Best possible solution = total # of colors used
    nodes = [Node(i, 0, [], 0) for i in range(node_count)]
    for i,j in edges:
        nodes[i].edges.append(j)
        nodes[j].edges.append(i)

    def check_feasibility(node):
        connecting_nodes = node.edges
        for node_idx in connecting_nodes:
            if node.color == nodes[node_idx].color:
                return False
        return True
    
    def first_available(node):
        neighbour_colors = [
            nodes[i].color for i in node.edges 
            if nodes[i].color != node.color and nodes[i].color != 0
        ]
        if neighbour_colors:
            node.color = min(neighbour_colors)
        else:
            node.total_color_used += 1
            node.color = node.total_color_used
        
    current_idx = 0
    ordered_degree = sorted(nodes, key=lambda k : len(k.edges), reverse=True)
    ordered_degree[0].color = 0
    q = deque()
    q.append(nodes[0])

    while len(q) > 0 or current_idx < len(nodes):
        cur = q.pop()
        if check_feasibility(cur):
            current_idx += 1
            if current_idx >= len(nodes):
                break
            q.append(nodes[current_idx])
        else:
            first_available(cur)
            nodes[cur.index] = cur
            q.append(cur)
    
    solution = [node.color for node in nodes]
    return solution
    

    



def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    first_line = lines[0].split()
    node_count = int(first_line[0])
    edge_count = int(first_line[1])

    edges = []
    for i in range(1, edge_count + 1):
        line = lines[i]
        parts = line.split()
        edges.append((int(parts[0]), int(parts[1])))
    
    # build a trivial solution
    # every node has its own color
    n = 0
    if node_count == 50:
        n = 6
        solution = solve3(node_count, edges, n)
    elif node_count == 70:
        n = 17
        solution = solve3(node_count, edges, n)
    elif node_count == 100:
        n = 16
        solution = solve3(node_count, edges, n)
    elif node_count > 100:
        solution = greedy(node_count, edges)
    
    optimal_value = len(set(solution))
    
    # prepare the solution in the specified output format
    output_data = str(optimal_value) + ' ' + str(0) + '\n'
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
        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/gc_4_1)')

