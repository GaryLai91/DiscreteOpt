#!/usr/bin/python
# -*- coding: utf-8 -*-

from collections import namedtuple, deque
#from numba import jit, int32
import numpy as np
import os
import timeit
from queue import Queue
from itertools import permutations
from LPSolver import KnapsackSolver

Item = namedtuple("Item", ['index', 'value', 'weight', 'density'])


class Node:
    def __init__(self, value, weight, estimate, level, taken):
        self.value = value
        self.weight = weight
        self.estimate = estimate
        self.level = level
        self.taken = taken


def greedy(items, capacity):
    value = 0
    weight = 0
    taken = [0]*len(items)

    for item in items:
        if weight + item.weight <= capacity:
            taken[item.index] = 1
            value += item.value
            weight += item.weight
    return value, taken


def recursion(items, capacity):
    items = tuple(items)
    value = 0
    weight = 0
    taken = [0]*len(items)
    table = np.array([[0] * (len(items)+1) for i in range(capacity+1)])

    @jit()
    def recurse(k, j):
        if (j == 0):
            return 0
        elif (items[j-1].weight <= k):
            max_ = max(
                        table[k][j-1],
                        items[j-1].value + table[k-items[j-1].weight][j-1]
                    )
            return max_
        else:
            return table[k][j-1]
    
    for j in range(1, len(items)+1):
        for k in range(1, capacity+1):
            table[k][j] = recurse(k, j)
    
    row, col = capacity, len(items)
    while capacity >= 0 and row > 0 and col > 0:
        if table[row][col] != table[row][col-1]:
            capacity -= items[col-1].weight
            row -= items[col-1].weight
            col -= 1
            taken[items[col].index] = 1
            value += items[col].value
        else:
            col -= 1
    return value, taken


def bound(node, capacity, items):
    if node.weight <= 0:
        return 0
    value_bound = node.value
    j = node.level
    total_weight = node.weight 
    while (j < len(items)) and (total_weight - items[j].weight >= 0):
        total_weight -= items[j].weight 
        value_bound += items[j].value
        j += 1
    if (j < len(items)):
        value_bound += (total_weight) * items[j].value / items[j].weight
    return value_bound


def branch(items, capacity):
    value = 0
    taken = [0] * len(items)
    root = Node(0,capacity,0,0,taken)
    root.estimate = bound(root, capacity, items)
    q = deque()
    q.append(root)
    while len(q) > 0:
        u = q.pop()
        if u.level == len(items) or u.estimate <= value: 
            continue
        
        #left means take item
        left_node = Node(0,0,0,0,[])
        left_node.value = u.value + items[u.level].value
        left_node.weight = u.weight - items[u.level].weight
        left_node.level = u.level + 1
        left_node.estimate = bound(left_node, left_node.weight, items)
        left_node.taken = u.taken[:]
        left_node.taken[items[u.level].index] = 1

        # right means don't take item
        right_node = Node(0,0,0,0,[])
        right_node.value = u.value
        right_node.weight = u.weight
        right_node.level = u.level + 1
        right_node.estimate = bound(right_node, right_node.weight, items)
        right_node.taken = u.taken[:]
        right_node.taken[items[u.level].index] = 0

        if right_node.weight >= 0 :
            q.append(right_node)
            if right_node.value > value:
                value = right_node.value
                taken = right_node.taken

        if left_node.weight >= 0 :
            q.append(left_node)
            if left_node.value > value:
                value = left_node.value
                taken = left_node.taken

    return value, taken


def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    firstLine = lines[0].split()
    item_count = int(firstLine[0])
    capacity = int(firstLine[1])

    items = []

    for i in range(1, item_count+1):
        line = lines[i]
        parts = line.split()
        index = i-1
        value = int(parts[0])
        weight = int(parts[1])
        density = value / weight
        items.append(Item(index, value, weight, density))

    # sort items by density in descending order
    #items = sorted(items, key=lambda x: x.density, reverse = True)

    # a trivial algorithm for filling the knapsack
    # it takes items in-order until the knapsack is full
    #value, taken = branch(items, capacity)
    
    item_value = [item.value for item in items]
    item_weight = [item.weight for item in items]
    lp = KnapsackSolver(item_value, item_weight, capacity)
    value, taken = lp.solve()
    
    
    # prepare the solution in the specified output format
    output_data = str(value) + ' ' + str(0) + '\n'
    output_data += ' '.join(map(str, taken))
    return output_data


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        print(solve_it(input_data))
    else:
        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/ks_4_0)')

