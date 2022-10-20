#!/usr/bin/env python3
'''
@INPROCEEDINGS{8011183,  author={Wiandt, Bernát and Simon, Vilmos and Kőkuti, András},  booktitle={IEEE EUROCON 2017 -17th International Conference on Smart Technologies},   title={Self-organized graph partitioning approach for multi-agent patrolling in generic graphs},   year={2017},  volume={},  number={},  pages={605-610},  doi={10.1109/EUROCON.2017.8011183}}
'''

import networkx as nx
import math
import numpy as np
import random as rn


class Agent_Part:

    def __init__(self, id, vertex, graph):
        self.id = id
        self.graph = graph
        self.vertices = [vertex]
        self.neigh = []
        self.err = 0
        self.grad = {}
        self.utility = {}
        self.free_nodes = []
        self.con_nodes = []
        self.non_free_nodes = []

    def update_connectivity(self):
        pass

    def update_utility(self):
        pass

    def update_grad(self):
        pass

    def check_neighbour(self, other_part):
        for u in self.vertices:
            for v in other_part.vertices:
                if (u, v) in self.graph.edges():
                    return  True

        return False

    def update_neighbours(self, parts):
        self.neigh = []
        for p in parts:
            if p.id != self.id and self.check_neighbour(p):
                self.neigh.append(p)

def pbps(graph, num_robots):
    '''
    Returns num_robots partitions of graph 
    '''

    assigned_nodes = []
    free_nodes = list(graph.nodes())
    parts = []
    for i in range(num_robots):
        node = rn.sample(free_nodes, 1)
        parts.append(Agent_Part(i, node, graph))
        free_nodes.remove(node)
        assigned_nodes.append(node)

    done = False
    while not done:
        for p in parts:
            p.update_neighbours(parts)

        
    