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

from mimetypes import init
import networkx as nx
import rospkg
import sys, os
import random as rn
import numpy as np
import plotly.graph_objects as go
import plotly.io as pio
pio.kaleido.scope.mathjax = None
import rosparam
# import matplotlib.animation as anplot
# import matplotlib.pyplot as plt

class Ant_Q:

    def __init__(self, graph, num_threads, sim_dir, na, q_0 = 0.8, alpha = 0.1, gamma = 0.3, delta = 3, beta = 1, W = 10000):
        
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
                elif i != j:
                    self.paths[(i, j)] = [i, j]

        self.edges = list(self.graph.edges())
        self.lengths = [float(self.graph[e[0]][e[1]]['length']) for e in self.edges]

        avg_len = sum(self.lengths)/len(self.edges)
        max_len = max(self.lengths)
        
        # initialisation of 'q' and 'h' values of the edges
        for e in self.edges:
            self.graph[e[0]][e[1]]['h_value'] = max_len/float(self.graph[e[0]][e[1]]['length'])
            self.graph[e[0]][e[1]]['q_value'] = max_len/(avg_len)
        
        self.node_qvalue = {}
        for n in self.nodes:
            self.node_qvalue[n] = max_len / avg_len
        
        # no h-value for node values
        self.node_hvalue = {}
        for n in self.nodes:
            self.node_hvalue[n]

        # for node episodes
        self.q = q_0
        self.q_rem = 1 - q_0
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.delta = delta
        self.W = W

        ### need to implement this ###
        self.na = na      # num of agents = num_init_nodes
        self.best_nodes = []
        self.best_walk = []
        self.best_len = np.inf
        self.best_walk_directions = []
        self.best_walk_edges = []

        self.sim_dir = sim_dir
        self.init_plot()    
        # self.num_threads = self.nodes
        # self.fig.show()

    def init_plot(self):

        d = 20 #radius of nodes

        # edge_x = []
        # edge_y = []
        self.edge_traces = [] 
        self.edges_plot = []
        for edge in self.edges_og:
            if not (edge[1], edge[0]) in self.edges_plot:
                self.edges_plot.append(edge)
                x0, y0 = self.graph.nodes[edge[0]]['x'], self.graph.nodes[edge[0]]['y']
                x1, y1 = self.graph.nodes[edge[1]]['x'], self.graph.nodes[edge[1]]['y']
                self.edge_traces.append(go.Scatter(x=[x0, x1, None], y=[y0, y1, None], line=dict(width=1, color = 'white'), opacity = 1, hoverinfo='none', mode='lines'))
        
        node_x = []
        node_y = []

        for node in list(self.nodes):
            x, y = self.graph.nodes[node]['x'], self.graph.nodes[node]['y']
            node_x.append(x)
            node_y.append(y)

        self.node_trace = go.Scatter(
            x=node_x, y=node_y,
            mode='markers',
            # hoverinfo='text',
            marker=dict(
                showscale=False,
                reversescale=False,
                color='white',
                size=d,
                line_width=0))


        fig = go.Figure(data=[self.node_trace] + self.edge_traces,
            layout=go.Layout(
            title='Graph \'{}\''.format(graph_name),
            title_x = 0.4,
            # titlefont_size=16,
            showlegend=False,
            hovermode='closest',
            margin=dict(b=20,l=5,r=5,t=40),
            xaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
            yaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
            plot_bgcolor='black'
            # updatemenus=[dict(type="buttons", buttons=[dict(label="Play", method="animate", args=[None, {"frame": {"duration": 5, "redraw": True}}])])]),
            # frames = test.frames
            ))
        fig.update_yaxes(scaleanchor = 'x', scaleratio = 1)
        fig.to_image(format="png", engine="kaleido")
        fig.write_image('{}/img_0.png'.format(self.sim_dir))
        # self.frames = [go.Frame(data=[self.node_trace] + self.edge_traces.copy())]

        # return fig
    
    def walks_run(self, init_nodes):
        actions = self.nodes.copy()
        actions.remove(init_nodes)
        walk_so_far = [[init_nodes[i]] for i in range(self.na)]                            # it will be a list of lists of size num_agents == num_init_nodes
        
        cur_nodes = init_nodes.copy()
        
        while len(actions) != 0:
            values = []
            max_vals = []

            for a in range(self.na):
                value_list = [(self.graph[cur_nodes[a]][node]['q_value'] ** self.delta) * (self.graph[cur_nodes[a]][node]['h_value'] ** self.beta) for node in actions]                    
                values.append(value_list)
                max_vals.append(max(value_list))

            a_ind = np.argmax(max_vals)                     # agent which has the max q_value
            a_node = np.argmax(values[a_ind])               # node which has the max q_value for this agent

            # values = [self.graph[cur_node][node]['q_value'] for node in actions]
            tot_val = sum(values[a_ind])
            a_values = [v/tot_val for v in values[a_ind]]
            
            if rn.random() <= self.q:
                node = a_node

            else:
                #random-proportional
                r_val = rn.random()
                cum_sum = 0.
                i = -1
                while r_val < cum_sum:
                    i += 1
                    cum_sum += a_values[i]
                node = actions[i]
                
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

        final_nodes = self.return_to_base(walk_so_far, le, init_nodes)

        for a in range(self.na):
            walk_so_far[a].append(final_nodes[a])
            le[a] += self.graph[walk_so_far[a][-2]][walk_so_far[a][-1]]['length']

        return (walk_so_far, le)
    
    def return_to_base(self, walk_so_far, le, init_nodes):
        final_nodes = [0 for _ in range(self.na)]
        for node in init_nodes:
            min = le[0] + self.graph[walk_so_far[0][-1]][node]['length']
            agent = 0
            for a in range(self.na):
                l = le[a] + self.graph[walk_so_far[a][-1]][node]['length']
                if l <= min:
                    min = l
                    agent = a
            
            final_nodes[agent] = node
        
        return final_nodes
    
    def walks_episode(self, init_nodes):

        # takes actions -> till all nodes have been covered -> return back -> \\ return the walks of the agents and their lengths as lists
        walks, lengths = self.walks_run(init_nodes)
        worst_idleness = max(lengths)

        # we need to use the worst path since it indicates worst idleness of our patrolling --- not iter_best --> iter_worst 
        del_aq = self.W / worst_idleness
        for w in walks:
            for i in range(len(w) - 1):
                self.graph[w[i]][w[i + 1]]['q_value'] += self.alpha * del_aq

        return (walks, worst_idleness)

    def nodes_run(self):

        node_values = [(self.node_qvalue[n] ** self.delta) * (self.node_hvalue[n] ** self.beta) for n in self.nodes]
        node_avail = self.nodes.copy()
        init_nodes = []

        for a in range(self.na):
            if rn.random() <= self.q:
                node_index = node_values.index(max(node_values))

            else:
                #random-proportional
                r_val = rn.random()
                cum_sum = 0.
                i = -1
                while r_val < cum_sum:
                    i += 1
                    cum_sum += node_values[i]
                
                node_index = i
                
            init_nodes.append(node_avail[node_index])
            node_values.pop(node_index)
            node_avail.pop(node_index)
        
        # run the training for the given set of init_nodes
        # returns the best walk and best length possible for this set of nodes

        w_node, l_node = self.walks_episode(init_nodes)

        return w_node, l_node
    
    def nodes_episode(self, ep_count):
        
        if ep_count > 100 and self.q < 0.95:
            self.q += (self.q_rem/2) ** 2
            self.q_rem = 1 - self.q
        
        # make the node_value of the picked node = 0
        for a in range(self.na):
            if rn.random() <= self.q:
                node = np.argmax(self.node_values)
                init_nodes.append(node)
            else:
                #random-proportional
                r_val = rn.random()
                cum_sum = 0.
                i = -1
                while r_val < cum_sum:
                    i += 1
                    cum_sum += self.node_values[i]
                
                node = self.nodes[i]
                init_nodes.append(node)

        best_idleness = np.Infinity
        for e2 in range(self.num_episodes_walk):
            w, l = self.walks_episode(init_nodes)
            if l <= best_idleness:
                best_idleness = l
        
        del_Q = self.W / best_idleness

        for i in init_nodes:
            self.node_values[i] = (1-self.alpha)*self.node_values[i] + self.alpha * del_Q

        return init_nodes, best_idleness               


if __name__ == '__main__':
    dirname = rospkg.RosPack().get_path('mrpp_sumo')
    sim_file = sys.argv[1]

    params = rosparam.load_file(sim_file)[0][0]
    graph_name = params['graph']
    num_episodes_walk = params['num_episodes_walk']
    num_episodes_node = params['num_episodes_node']
    num_threads = params['num_threads']                 # this is = 1 in our case
    num_agents = params['num_agents']
    algo = params['algo_name']
    sim_name = params['random_string']
    sim_dir = dirname + '/outputs/' + sim_name
    os.mkdir(sim_dir)
    
    # init_nodes = params['init_nodes']
    

    if algo != 'ant_q_tsp':
        print('Not this algorithm')
        exit

    g = nx.read_graphml(dirname + '/graph_ml/' + graph_name + '.graphml')

    # best_walk = test.best_walk_directions
    with open(dirname + '/outputs/{}_visit_seq.in'.format(sim_name), 'w') as f:
        
        test = Ant_Q(g, num_threads, sim_dir, num_agents)

        for i in range(num_episodes_node):
            # here w, l -> best walk and best len for the given set of nodes
            w, l = test.nodes_episode(i + 1)
            
            f.write('Epsiode' + str(i + 1) + ': ' + str(l) + '\n')
            f.write('\n')
            for n in w:
                f.write(str(n)+ '\n')
            f.write('\n')            
        
        best_nodes = test.best_init_nodes
        best_walk = test.best_walk_directions
        best_len = test.best_len

        f.write('Best init nodes:' + str(best_nodes) + '\n')
        f.write('Shortest Length: ' + str(best_len) + '\n')
        f.write('\n')
        for n in best_walk:
            f.write(str(n)+ '\n')
