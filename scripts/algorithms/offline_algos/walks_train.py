#!/usr/bin/env python3


'''
Parameters:
1. Graph
2. Number of episodes
3. Number of threads
4. Max number of bots
5. algo_name == 'ant_q_tsp'
6. random_string

Outputs:
1. 'random_string'_vis.html - Visualization of optimal cycle across episodes
2. 'random_string'_seq.in - sequence of node visits
'''

import networkx as nx
import sys, os
import random as rn
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt

class Ant_Q:

    def __init__(self, graph, num_threads, init_nodes, sim_dir, q_0 = 0.8, alpha = 0.1, gamma = 0.3, delta = 3, beta = 1, W = 10000):
        
        self.graph = graph.copy()
        self.edges_og = list(graph.edges())
        self.nodes = list(graph.nodes())
        self.paths = {}
        self.num_threads = min(num_threads, len(self.nodes))
        # building complete graph
        for i in self.nodes:
            for j in self.nodes:
                if i != j and not (i, j) in self.edges_og:
                    self.graph.add_edge(i, j)
                    self.graph[i][j]['name'] = '{}to{}'.format(i, j)
                    self.graph[i][j]['length'] = nx.dijkstra_path_length(graph, i, j, 'length')
                    self.paths[(i, j)] = nx.dijkstra_path(graph, i, j, 'length')
                # elif i==j and not (i, j) in self.graph.edges():
                #     self.graph.add_edge(i, j)
                #     self.graph[i][j]['name'] = '{}to{}'.format(i, j)
                #     self.graph[i][j]['length'] = 0
                elif i != j:
                    self.paths[(i, j)] = [i, j]

        self.edges = list(self.graph.edges())
        self.lengths = [float(self.graph[e[0]][e[1]]['length']) for e in self.edges]

        avg_len = sum(self.lengths)/len(self.edges)
        max_len = max(self.lengths)
        for e in self.edges:
            self.graph[e[0]][e[1]]['h_value'] = max_len/float(self.graph[e[0]][e[1]]['length'])
            self.graph[e[0]][e[1]]['q_value'] = max_len/(avg_len)

        self.q = q_0
        self.q_rem = 1 - q_0
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.delta = delta
        self.W = W

        self.init_nodes = init_nodes
        self.na = len(self.init_nodes)      # num of agents = num_init_nodes
        self.best_walk = []
        self.best_len = np.inf
        self.best_walk_directions = []
        self.best_walk_edges = []

        self.sim_dir = sim_dir
    
    def run(self):
        actions = self.nodes.copy()
        actions = list(set(actions).difference(set(self.init_nodes)))

        walk_so_far = [[self.init_nodes[i]] for i in range(self.na)]                            # it will be a list of lists of size num_agents == num_init_nodes
        
        cur_nodes = self.init_nodes.copy()
        
        while len(actions) != 0:
            values = []
            max_vals = []

            for a in range(self.na):
                value_list = [(self.graph[cur_nodes[a]][node]['q_value'] ** self.delta) * (self.graph[cur_nodes[a]][node]['h_value'] ** self.beta) for node in actions]                    
                values.append(value_list)
                max_vals.append(max(value_list))

            a_ind = np.argmax(max_vals)                     # agent which has the max q_value
            a_node = actions[np.argmax(values[a_ind])]               # node which has the max q_value for this agent

            # values = [self.graph[cur_node][node]['q_value'] for node in actions]
            
            if rn.random() <= self.q:
                node = str(a_node)

            else:
                # sample a random angent (uniformly)
                a_ind = rn.randint(int(self.init_nodes[0]), int(self.init_nodes[-1]))
                tot_val = sum(values[a_ind])

                a_values = [v/tot_val for v in values[a_ind]]
                #random-proportional
                r_val = rn.random()
                cum_sum = 0.
                i = -1
                while r_val < cum_sum:
                    i += 1
                    cum_sum += a_values[i]
                node = str(actions[i])
                
            walk_so_far[a_ind].append(node)
            actions.remove(node)
            
            # update Q_values after the action taken
            if actions != []:
                next_val = max([self.graph[node][nex]['q_value'] for nex in actions])
                self.graph[cur_nodes[a_ind]][node]['q_value'] *= (1 - self.alpha)
                self.graph[cur_nodes[a_ind]][node]['q_value'] += self.alpha * self.gamma * next_val
            
            cur_nodes[a_ind] = node

        le = []
        for a in range(self.na):
            walk = walk_so_far[a]
            agent_l = 0
            for i in range(len(walk)-1):
                agent_l += self.graph[walk[i]][walk[i + 1]]['length']
            le.append(agent_l)

        final_nodes = self.return_to_base(walk_so_far, le)

        for a in range(self.na):
            walk_so_far[a].append(final_nodes[a])
            node1 = walk_so_far[a][-1]
            node2 = walk_so_far[a][-2]

            if node1 != node2:
                le[a] += self.graph[node2][node1]['length']

        return (walk_so_far, le)
    
    def return_to_base(self, walk_so_far, le):
        # final_nodes = ['0' for _ in range(self.na)]
        # for node in self.init_nodes:
        #     min = le[0] + self.graph[walk_so_far[0][-1]][node]['length']
        #     agent = 0
        #     for a in range(self.na):
        #         print("# # # # #  {}".format(walk_so_far[a][-1]))
        #         l = le[a] + self.graph[walk_so_far[a][-1]][node]['length']
        #         if l <= min:
        #             min = l
        #             agent = a
            
        #     final_nodes[agent] = str(node)
        
        return self.init_nodes
    
    def episode(self, ep_count):
        
        if ep_count > 100 and self.q < 0.95:
            self.q += (self.q_rem/2) ** 2
            self.q_rem = 1 - self.q 
        
        # takes actions -> till all nodes have been covered -> return back -> \\ return the walks of the agents and their lengths as lists
        walks, lengths = self.run()

        worst_idleness = max(lengths)

        # we need to use the worst path since it indicates worst idleness of our patrolling --- not iter_best --> iter_worst 
        del_aq = self.W / worst_idleness

        for w in walks:
            for i in range(len(w) - 1):
                e0, e1 = w[i], w[i+1]
                if e0 != e1:
                    self.graph[e0][e1]['q_value'] += self.alpha * del_aq

        return (walks, worst_idleness)


if __name__ == '__main__':
    
    now = datetime.now()
    date_time = now.strftime("%m_%d_%Y_%H%M%S")
    
    # for current purposes only
    params = {
        "graph": "/Users/saigangadhar/Desktop/SEM_7/MRPP/mrpp_sumo/graph_ml/grid_5_5.graphml",
        "num_episodes": 5000,
        "num_threads": 1,
        "init_nodes": ['0','1','2'],
        "algo_name": "ant_q_tsp",
        "random_string": date_time,
        "output_path": "/Users/saigangadhar/Desktop/SEM_7/MRPP/ant_q/outputs",
    }

    graph_name = params['graph']
    num_episodes = params['num_episodes']
    num_threads = params['num_threads']                 # this is = 1 in our case
    init_nodes = params['init_nodes']
    algo = params['algo_name']
    sim_name = params['random_string']
    sim_dir = params["output_path"]

    log_data = True

    # os.mkdir(sim_dir)

    if algo != 'ant_q_tsp':
        print('Not this algorithm')
        exit

    g = nx.read_graphml(graph_name)
    lengths = []

    # best_walk = test.best_walk_directions
    with open(sim_dir + '/{}.txt'.format(sim_name), 'w') as f:
        
        test = Ant_Q(g, num_threads, init_nodes, sim_dir)

        for i in range(num_episodes):
            w, l = test.episode(i + 1)
            lengths.append(l)

            if log_data == True:
                f.write('Epsiode' + str(i + 1) + ': ' + str(l) + '\n')
                f.write('\n')
                for n in w:
                    f.write(str(n)+ '\n')
                f.write('\n')

    x_axis = [i for i in range(num_episodes)]
    plt.plot(x_axis, lengths)
    plt.title("train_error error")