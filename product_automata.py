
#!/usr/bin/env python

# construct synchronization buchi structure for each robot using method of creating one product automaton with all bindings

# importing sys
import sys
  
# adding Folder_2 to the system path
sys.path.insert(0, '../../Mission_Switching/scripts')

import spot
import graphviz
import networkx as nx
import matplotlib.pyplot as plt
from networkx.drawing.nx_agraph import write_dot
import numpy as np
import re
import networkx as nx
import matplotlib.pyplot as plt
import create_agents as af
from agent_module import Agent
import find_optimal_run as fr
import find_optimal_run as fir
import bool_parsing
import testing_methods as method1
import testing_methods2 as method2
import buchi_functions as bf
import psi_parser
import centralized_synth
from buchi_bindings_module import BuchiBindings
import math
import itertools
import synthesize_team_behavior as stb
import generate_figures as gf
import time
from collections import defaultdict

def create_rooms(weighting_factor):
    K = nx.DiGraph()
    
    l1 = 'room1'
    not_l1 = '!room1'

    l2 = 'room2'
    not_l2 = '!room2'

    l3 = 'room3'
    not_l3 = '!room3'

    # l4 = 'room4'
    # not_l4 = '!room4'

    # l5 = 'room5'
    # not_l5 = '!room5'

    ap = [l1,l2,l3]#,l4, l5]#, l6, l7, l8, l9, l10]

    s1 = ' & '.join([l1, not_l2, not_l3])#, not_l4, not_l5])#, not_l6, not_l7, not_l8, not_l9, not_l10])
    s2 = ' & '.join([not_l1, l2, not_l3])#, not_l4, not_l5])#, not_l6, not_l7, not_l8, not_l9, not_l10])
    s3 = ' & '.join([not_l1, not_l2, l3])#, not_l4, not_l5])#, not_l6, not_l7, not_l8, not_l9, not_l10])
    s4 = ' & '.join([not_l1, not_l2, not_l3])#, l4, not_l5])#, not_l6, not_l7, not_l8, not_l9, not_l10])
    s5 = ' & '.join([not_l1, not_l2, not_l3])#, not_l4, l5])#, not_l6, not_l7, not_l8, not_l9, not_l10])

    K.add_node(s1)
    K.add_node(s2)
    K.add_node(s3)
    # K.add_node(s4)
    # K.add_node(s5)

    K.add_edge(s1, s3, weight = 0.3*weighting_factor); 
    K.add_edge(s3, s1, weight = 0.3*weighting_factor); 
    # K.add_edge(s1, s5, weight = 0.3*weighting_factor); 
    # K.add_edge(s2, s5, weight = 0.3*weighting_factor); 
    # K.add_edge(s4, s5, weight = 0.3*weighting_factor);

    K.add_edge(s3, s2, weight = 0.3*weighting_factor); 
    K.add_edge(s2, s3, weight = 0.3*weighting_factor); 
    # K.add_edge(s5, s1, weight = 0.3*weighting_factor); 
    # K.add_edge(s5, s2, weight = 0.3*weighting_factor); 
    # K.add_edge(s5, s4, weight = 0.3*weighting_factor);

    K.add_edge(s1, s1, weight = 0);
    K.add_edge(s2, s2, weight = 0);
    K.add_edge(s3, s3, weight = 0);
    # K.add_edge(s4, s4, weight = 0);
    # K.add_edge(s5, s5, weight = 0);

    return K, ap

def create_rooms9(weighting_factor):
    K = nx.DiGraph()
    
    l1 = 'room1'
    not_l1 = '!room1'

    l2 = 'room2'
    not_l2 = '!room2'

    l3 = 'room3'
    not_l3 = '!room3'

    l4 = 'room4'
    not_l4 = '!room4'

    l5 = 'room5'
    not_l5 = '!room5'

    l6 = 'room6'
    not_l6 = '!room6'

    l7 = 'room7'
    not_l7 = '!room7'

    l8 = 'room8'
    not_l8 = '!room8'

    l9 = 'room9'
    not_l9 = '!room9'

    ap = [l1,l2,l3 ,l4, l5, l6, l7, l8, l9]#, l10]

    s1 = ' & '.join([l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7, not_l8, not_l9])
    s2 = ' & '.join([not_l1, l2, not_l3, not_l4, not_l5, not_l6, not_l7, not_l8, not_l9])
    s3 = ' & '.join([not_l1, not_l2, l3, not_l4, not_l5, not_l6, not_l7, not_l8, not_l9])
    s4 = ' & '.join([not_l1, not_l2, not_l3, l4, not_l5, not_l6, not_l7, not_l8, not_l9])
    s5 = ' & '.join([not_l1, not_l2, not_l3, not_l4, l5, not_l6, not_l7, not_l8, not_l9])
    s6 = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, l6, not_l7, not_l8, not_l9])
    s7 = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, l7, not_l8, not_l9])
    s8 = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7, l8, not_l9])
    s9 = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7, not_l8, l9])

    K.add_node(s1)
    K.add_node(s2)
    K.add_node(s3)
    K.add_node(s4)
    K.add_node(s5)
    K.add_node(s6)
    K.add_node(s7)
    K.add_node(s8)
    K.add_node(s9)


    K.add_edge(s1, s2, weight = 0.3*weighting_factor); 
    K.add_edge(s2, s3, weight = 0.3*weighting_factor); 
    K.add_edge(s2, s5, weight = 0.3*weighting_factor); 
    K.add_edge(s3, s6, weight = 0.3*weighting_factor); 
    K.add_edge(s4, s5, weight = 0.3*weighting_factor); 
    K.add_edge(s4, s7, weight = 0.3*weighting_factor); 
    K.add_edge(s7, s8, weight = 0.3*weighting_factor); 
    K.add_edge(s8, s9, weight = 0.3*weighting_factor); 

    K.add_edge(s2, s1, weight = 0.3*weighting_factor); 
    K.add_edge(s3, s2, weight = 0.3*weighting_factor); 
    K.add_edge(s5, s2, weight = 0.3*weighting_factor); 
    K.add_edge(s6, s3, weight = 0.3*weighting_factor); 
    K.add_edge(s5, s4, weight = 0.3*weighting_factor); 
    K.add_edge(s7, s4, weight = 0.3*weighting_factor); 
    K.add_edge(s8, s7, weight = 0.3*weighting_factor); 
    K.add_edge(s9, s8, weight = 0.3*weighting_factor); 



    K.add_edge(s1, s1, weight = 0);
    K.add_edge(s2, s2, weight = 0);
    K.add_edge(s3, s3, weight = 0);
    K.add_edge(s4, s4, weight = 0);
    K.add_edge(s5, s5, weight = 0);
    K.add_edge(s6, s6, weight = 0);
    K.add_edge(s7, s7, weight = 0);
    K.add_edge(s8, s8, weight = 0);
    K.add_edge(s9, s9, weight = 0);

    return K, ap


def create_grid_rooms30(weighting_factor):
    K = nx.DiGraph()
    ap = []
    for i in range(1, 31):
        for j in range(1, 31):
            room = f'room{i}{j}'
            ap.append(room)
            K.add_node(room)
            
    for i in range(1, 30):
        for j in range(1, 30):
            current_room = f'room{i}{j}'
            K.add_edge(current_room, f'room{i+1}{j}', weight = weighting_factor)
            K.add_edge(current_room, f'room{i}{j+1}', weight = weighting_factor)
            
    return K, ap

def create_rooms_wh_c(weighting_factor):
    K = nx.DiGraph()
    
    l1 = 'room1'
    not_l1 = '!room1'
    l1c = 'room1c'
    not_l1c = '!room1c'

    l2 = 'room2'
    not_l2 = '!room2'
    l2c = 'room2c'
    not_l2c = '!room2c'

    l3 = 'room3'
    not_l3 = '!room3'
    l3c = 'room3c'
    not_l3c = '!room3c'

    l4 = 'room4'
    not_l4 = '!room4'
    l4c = 'room4c'
    not_l4c = '!room4c'

    l5 = 'room5'
    not_l5 = '!room5'
    l5c = 'room5c'
    not_l5c = '!room5c'

    l6 = 'room6'
    not_l6 = '!room6'
    l6c = 'room6c'
    not_l6c = '!room6c'

    l7 = 'room7'
    not_l7 = '!room7'
    l7c = 'room7c'
    not_l7c = '!room7c'

    l8 = 'room8'
    not_l8 = '!room8'
    l8c = 'room8c'
    not_l8c = '!room8c'

    l9 = 'room9'
    not_l9 = '!room9'
    l9c = 'room9c'
    not_l9c = '!room9c'

    ap = [l1, l2, l3, l4, l5, l6, l7, l8, l9, l1c, l2c, l3c, l4c, l5c, l6c, l7c, l8c, l9c]  # , l10]
    ap0 = [l1, l2, l3, l4, l5, l6, l7, l8, l9]
    apc = [l1c, l2c, l3c, l4c, l5c, l6c, l7c, l8c, l9c]
    not_ap0 = [not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7, not_l8, not_l9]
    not_apc = [not_l1c, not_l2c, not_l3c, not_l4c, not_l5c, not_l6c, not_l7c, not_l8c, not_l9c]

    # completion props
    state_dict_c = {}

    for idx in range(len(apc)):
        state_dict_c['s' + str(idx + 1) + 'c'] = ' & '.join(
            [apc[idx]] + [x for i, x in enumerate(not_apc) if i != idx] + not_ap0)

    s1c = state_dict_c['s1c']
    s2c = state_dict_c['s2c']
    s3c = state_dict_c['s3c']
    s4c = state_dict_c['s4c']
    s5c = state_dict_c['s5c']
    s6c = state_dict_c['s6c']
    s7c = state_dict_c['s7c']
    s8c = state_dict_c['s8c']
    s9c = state_dict_c['s9c']

    K.add_node(s1c)
    K.add_node(s2c)
    K.add_node(s3c)
    K.add_node(s4c)
    K.add_node(s5c)
    K.add_node(s6c)
    K.add_node(s7c)
    K.add_node(s8c)
    K.add_node(s9c)

    K.add_edge(s1c, s1c, weight=0)
    K.add_edge(s2c, s2c, weight=0)
    K.add_edge(s3c, s3c, weight=0)
    K.add_edge(s4c, s4c, weight=0)
    K.add_edge(s5c, s5c, weight=0)
    K.add_edge(s6c, s6c, weight=0)
    K.add_edge(s7c, s7c, weight=0)
    K.add_edge(s8c, s8c, weight=0)
    K.add_edge(s9c, s9c, weight=0)

    # create state where you are currently in room s_c and start moving towards room s_0
    def create_start_state(s_c, s_0, apc, not_apc, not_ap0, state_dict_c, K):
        state = []
        for idx in range(len(apc)):
            if idx == s_c - 1:
                state.append(apc[idx])
            elif idx == s_0 - 1:
                state.append(apc[idx][:-1])
            else:
                state.append(not_apc[idx])
        state += not_ap0
        state = ' & '.join(state)

        # add to DiGraph
        K.add_node(state)

        K.add_edge(state_dict_c['s'+str(s_c)+'c'], state, weight = 0.3*weighting_factor)
        K.add_edge(state, state_dict_c['s' + str(s_0) + 'c'], weight=0.3 * weighting_factor)
        K.add_edge(state, state, weight=0.3 * weighting_factor)
        return state, K

    s1c2, K = create_start_state(1, 2, apc, not_apc, not_ap0, state_dict_c, K)
    s2c3, K = create_start_state(2, 3, apc, not_apc, not_ap0, state_dict_c, K)
    s2c6, K = create_start_state(2, 6, apc, not_apc, not_ap0, state_dict_c, K)
    s3c4, K = create_start_state(3, 4, apc, not_apc, not_ap0, state_dict_c, K)
    s4c5, K = create_start_state(4, 5, apc, not_apc, not_ap0, state_dict_c, K)
    s4c6, K = create_start_state(4, 6, apc, not_apc, not_ap0, state_dict_c, K)
    s6c8, K = create_start_state(6, 8, apc, not_apc, not_ap0, state_dict_c, K)
    s7c8, K = create_start_state(7, 8, apc, not_apc, not_ap0, state_dict_c, K)
    s8c9, K = create_start_state(8, 9, apc, not_apc, not_ap0, state_dict_c, K)

    s2c1, K = create_start_state(2, 1, apc, not_apc, not_ap0, state_dict_c, K)
    s3c2, K = create_start_state(3, 2, apc, not_apc, not_ap0, state_dict_c, K)
    s6c2, K = create_start_state(6, 2, apc, not_apc, not_ap0, state_dict_c, K)
    s4c3, K = create_start_state(4, 3, apc, not_apc, not_ap0, state_dict_c, K)
    s5c4, K = create_start_state(5, 4, apc, not_apc, not_ap0, state_dict_c, K)
    s6c4, K = create_start_state(6, 4, apc, not_apc, not_ap0, state_dict_c, K)
    s8c7, K = create_start_state(8, 7, apc, not_apc, not_ap0, state_dict_c, K)
    s8c6, K = create_start_state(8, 6, apc, not_apc, not_ap0, state_dict_c, K)
    s9c8, K = create_start_state(9, 8, apc, not_apc, not_ap0, state_dict_c, K)

    #
    #
    # s1 = ' & '.join([l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7, not_l8, not_l9,
    #                  not_l1c, not_l2c, not_l3c, not_l4c, not_l5c, not_l6c, not_l7c, not_l8c, not_l9c])
    # s2 = ' & '.join([not_l1, l2, not_l3, not_l4, not_l5, not_l6, not_l7, not_l8, not_l9,
    #                  not_l1c, not_l2c, not_l3c, not_l4c, not_l5c, not_l6c, not_l7c, not_l8c, not_l9c])
    # s3 = ' & '.join([not_l1, not_l2, l3, not_l4, not_l5, not_l6, not_l7, not_l8, not_l9,
    #                  not_l1c, not_l2c, not_l3c, not_l4c, not_l5c, not_l6c, not_l7c, not_l8c, not_l9c])
    # s4 = ' & '.join([not_l1, not_l2, not_l3, l4, not_l5, not_l6, not_l7, not_l8, not_l9,
    #                  not_l1c, not_l2c, not_l3c, not_l4c, not_l5c, not_l6c, not_l7c, not_l8c, not_l9c])
    # s5 = ' & '.join([not_l1, not_l2, not_l3, not_l4, l5, not_l6, not_l7, not_l8, not_l9,
    #                  not_l1c, not_l2c, not_l3c, not_l4c, not_l5c, not_l6c, not_l7c, not_l8c, not_l9c])
    # s6 = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, l6, not_l7, not_l8, not_l9,
    #                  not_l1c, not_l2c, not_l3c, not_l4c, not_l5c, not_l6c, not_l7c, not_l8c, not_l9c])
    # s7 = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, l7, not_l8, not_l9,
    #                  not_l1c, not_l2c, not_l3c, not_l4c, not_l5c, not_l6c, not_l7c, not_l8c, not_l9c])
    # s8 = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7, l8, not_l9,
    #                  not_l1c, not_l2c, not_l3c, not_l4c, not_l5c, not_l6c, not_l7c, not_l8c, not_l9c])
    # s9 = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7, not_l8, l9,
    #                  not_l1c, not_l2c, not_l3c, not_l4c, not_l5c, not_l6c, not_l7c, not_l8c, not_l9c])
    #
    # s1c = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7, not_l8, not_l9,
    #                  l1c, not_l2c, not_l3c, not_l4c, not_l5c, not_l6c, not_l7c, not_l8c, not_l9c])
    # s2c = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7, not_l8, not_l9,
    #                  not_l1c, l2c, not_l3c, not_l4c, not_l5c, not_l6c, not_l7c, not_l8c, not_l9c])
    # s3c = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7, not_l8, not_l9,
    #                  not_l1c, not_l2c, l3c, not_l4c, not_l5c, not_l6c, not_l7c, not_l8c, not_l9c])
    # s4c = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7, not_l8, not_l9,
    #                  not_l1c, not_l2c, not_l3c, l4c, not_l5c, not_l6c, not_l7c, not_l8c, not_l9c])
    # s5c = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7, not_l8, not_l9,
    #                  not_l1c, not_l2c, not_l3c, not_l4c, l5c, not_l6c, not_l7c, not_l8c, not_l9c])
    # s6c = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7, not_l8, not_l9,
    #                  not_l1c, not_l2c, not_l3c, not_l4c, not_l5c, l6c, not_l7c, not_l8c, not_l9c])
    # s7c = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7, not_l8, not_l9,
    #                  not_l1c, not_l2c, not_l3c, not_l4c, not_l5c, not_l6c, l7c, not_l8c, not_l9c])
    # s8c = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7, not_l8, not_l9,
    #                  not_l1c, not_l2c, not_l3c, not_l4c, not_l5c, not_l6c, not_l7c, l8c, not_l9c])
    # s9c = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7, not_l8, not_l9,
    #                  not_l1c, not_l2c, not_l3c, not_l4c, not_l5c, not_l6c, not_l7c, not_l8c, l9c])
    #
    # s1 = l1
    # s2 = l2
    # s3 = l3
    # s4 = l4
    # s5 = l5
    # s6 = l6
    # s7 = l7
    # s8 = l8
    # s9 = l9
    #
    # s1c = l1c
    # s2c = l2c
    # s3c = l3c
    # s4c = l4c
    # s5c = l5c
    # s6c = l6c
    # s7c = l7c
    # s8c = l8c
    # s9c = l9c
    #
    # K.add_node(s1)
    # K.add_node(s2)
    # K.add_node(s3)
    # K.add_node(s4)
    # K.add_node(s5)
    # K.add_node(s6)
    # K.add_node(s7)
    # K.add_node(s8)
    # K.add_node(s9)
    #
    # K.add_node(s1c)
    # K.add_node(s2c)
    # K.add_node(s3c)
    # K.add_node(s4c)
    # K.add_node(s5c)
    # K.add_node(s6c)
    # K.add_node(s7c)
    # K.add_node(s8c)
    # K.add_node(s9c)
    #
    #
    # K.add_edge(s1c, s2, weight = 0.3*weighting_factor);
    # K.add_edge(s2c, s3, weight = 0.3*weighting_factor);
    # K.add_edge(s2c, s6, weight = 0.3*weighting_factor);
    # K.add_edge(s3c, s4, weight = 0.3*weighting_factor);
    # K.add_edge(s4c, s5, weight = 0.3*weighting_factor);
    # K.add_edge(s4c, s6, weight = 0.3*weighting_factor);
    # K.add_edge(s6c, s8, weight = 0.3*weighting_factor);
    # K.add_edge(s7c, s8, weight = 0.3*weighting_factor);
    # K.add_edge(s8c, s9, weight = 0.3*weighting_factor);
    #
    # K.add_edge(s2c, s1, weight = 0.3*weighting_factor);
    # K.add_edge(s3c, s2, weight = 0.3*weighting_factor);
    # K.add_edge(s6c, s2, weight = 0.3*weighting_factor);
    # K.add_edge(s4c, s3, weight = 0.3*weighting_factor);
    # K.add_edge(s5c, s4, weight = 0.3*weighting_factor);
    # K.add_edge(s6c, s4, weight = 0.3*weighting_factor);
    # K.add_edge(s8c, s7, weight = 0.3*weighting_factor);
    # K.add_edge(s8c, s6, weight = 0.3*weighting_factor);
    # K.add_edge(s9c, s8, weight = 0.3*weighting_factor);
    #
    # K.add_edge(s1, s1c, weight=0.3*weighting_factor);
    # K.add_edge(s2, s2c, weight=0.3*weighting_factor);
    # K.add_edge(s3, s3c, weight=0.3*weighting_factor);
    # K.add_edge(s4, s4c, weight=0.3*weighting_factor);
    # K.add_edge(s5, s5c, weight=0.3*weighting_factor);
    # K.add_edge(s6, s6c, weight=0.3*weighting_factor);
    # K.add_edge(s7, s7c, weight=0.3*weighting_factor);
    # K.add_edge(s8, s8c, weight=0.3*weighting_factor);
    # K.add_edge(s9, s9c, weight=0.3*weighting_factor);
    #
    # # K.add_edge(s1c, s1, weight=0.3 * weighting_factor);
    # # K.add_edge(s2c, s2, weight=0.3 * weighting_factor);
    # # K.add_edge(s3c, s3, weight=0.3 * weighting_factor);
    # # K.add_edge(s4c, s4, weight=0.3 * weighting_factor);
    # # K.add_edge(s5c, s5, weight=0.3 * weighting_factor);
    # # K.add_edge(s6c, s6, weight=0.3 * weighting_factor);
    # # K.add_edge(s7c, s7, weight=0.3 * weighting_factor);
    # # K.add_edge(s8c, s8, weight=0.3 * weighting_factor);
    # # K.add_edge(s9c, s9, weight=0.3 * weighting_factor);
    #
    # # K.add_edge(s1, s1, weight = 0);
    # # K.add_edge(s2, s2, weight = 0);
    # # K.add_edge(s3, s3, weight = 0);
    # # K.add_edge(s4, s4, weight = 0);
    # # K.add_edge(s5, s5, weight = 0);
    # # K.add_edge(s6, s6, weight = 0);
    # # K.add_edge(s7, s7, weight = 0);
    # # K.add_edge(s8, s8, weight = 0);
    # # K.add_edge(s9, s9, weight = 0);
    #
    # K.add_edge(s1c, s1c, weight=0);
    # K.add_edge(s2c, s2c, weight=0);
    # K.add_edge(s3c, s3c, weight=0);
    # K.add_edge(s4c, s4c, weight=0);
    # K.add_edge(s5c, s5c, weight=0);
    # K.add_edge(s6c, s6c, weight=0);
    # K.add_edge(s7c, s7c, weight=0);
    # K.add_edge(s8c, s8c, weight=0);
    # K.add_edge(s9c, s9c, weight=0);
    state_labels = {}
    for state_name in K.nodes():
        state_labels[state_name] = ' & '.join(prop for prop in state_name.split(' & ') if prop[0] != '!')

    return K, ap, state_dict_c


def create_rooms_wh_small_c(weighting_factor):
    K = nx.DiGraph()

    l1 = 'room1'
    not_l1 = '!room1'
    l1c = 'room1c'
    not_l1c = '!room1c'

    l2 = 'room2'
    not_l2 = '!room2'
    l2c = 'room2c'
    not_l2c = '!room2c'

    l3 = 'room3'
    not_l3 = '!room3'
    l3c = 'room3c'
    not_l3c = '!room3c'

    l4 = 'room4'
    not_l4 = '!room4'
    l4c = 'room4c'
    not_l4c = '!room4c'

    l5 = 'room5'
    not_l5 = '!room5'
    l5c = 'room5c'
    not_l5c = '!room5c'

    l6 = 'room6'
    not_l6 = '!room6'
    l6c = 'room6c'
    not_l6c = '!room6c'

    l7 = 'room7'
    not_l7 = '!room7'
    l7c = 'room7c'
    not_l7c = '!room7c'

    ap = [l1, l2, l3, l4, l5, l6, l7, l1c, l2c, l3c, l4c, l5c, l6c, l7c]  # , l10]
    ap0 = [l1, l2, l3, l4, l5, l6, l7]
    apc = [l1c, l2c, l3c, l4c, l5c, l6c, l7c]
    not_ap0 = [not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7]
    not_apc = [not_l1c, not_l2c, not_l3c, not_l4c, not_l5c, not_l6c, not_l7c]

    # completion props
    state_dict_c = {}

    for idx in range(len(apc)):
        state_dict_c['s' + str(idx + 1) + 'c'] = ' & '.join(
            [apc[idx]] + [x for i, x in enumerate(not_apc) if i != idx] + not_ap0)

    s1c = state_dict_c['s1c']
    s2c = state_dict_c['s2c']
    s3c = state_dict_c['s3c']
    s4c = state_dict_c['s4c']
    s5c = state_dict_c['s5c']
    s6c = state_dict_c['s6c']
    s7c = state_dict_c['s7c']

    K.add_node(s1c)
    K.add_node(s2c)
    K.add_node(s3c)
    K.add_node(s4c)
    K.add_node(s5c)
    K.add_node(s6c)
    K.add_node(s7c)

    K.add_edge(s1c, s1c, weight=0)
    K.add_edge(s2c, s2c, weight=0)
    K.add_edge(s3c, s3c, weight=0)
    K.add_edge(s4c, s4c, weight=0)
    K.add_edge(s5c, s5c, weight=0)
    K.add_edge(s6c, s6c, weight=0)
    K.add_edge(s7c, s7c, weight=0)

    # create state where you are currently in room s_c and start moving towards room s_0
    def create_start_state(s_c, s_0, apc, not_apc, not_ap0, state_dict_c, K):
        state = []
        for idx in range(len(apc)):
            if idx == s_c - 1:
                state.append(apc[idx])
            elif idx == s_0 - 1:
                state.append(apc[idx][:-1])
            else:
                state.append(not_apc[idx])
        state += not_ap0
        state = ' & '.join(state)

        # add to DiGraph
        K.add_node(state)

        K.add_edge(state_dict_c['s' + str(s_c) + 'c'], state, weight=0.3 * weighting_factor)
        K.add_edge(state, state_dict_c['s' + str(s_0) + 'c'], weight=0.3 * weighting_factor)
        K.add_edge(state, state, weight=0.3 * weighting_factor)
        return state, K

    s1c2, K = create_start_state(1, 2, apc, not_apc, not_ap0, state_dict_c, K)
    s2c3, K = create_start_state(2, 3, apc, not_apc, not_ap0, state_dict_c, K)
    s2c6, K = create_start_state(2, 6, apc, not_apc, not_ap0, state_dict_c, K)
    s4c5, K = create_start_state(4, 5, apc, not_apc, not_ap0, state_dict_c, K)
    s4c6, K = create_start_state(4, 6, apc, not_apc, not_ap0, state_dict_c, K)
    s6c7, K = create_start_state(6, 7, apc, not_apc, not_ap0, state_dict_c, K)

    s2c1, K = create_start_state(2, 1, apc, not_apc, not_ap0, state_dict_c, K)
    s3c2, K = create_start_state(3, 2, apc, not_apc, not_ap0, state_dict_c, K)
    s6c2, K = create_start_state(6, 2, apc, not_apc, not_ap0, state_dict_c, K)
    s5c4, K = create_start_state(5, 4, apc, not_apc, not_ap0, state_dict_c, K)
    s6c4, K = create_start_state(6, 4, apc, not_apc, not_ap0, state_dict_c, K)
    s7c6, K = create_start_state(7, 6, apc, not_apc, not_ap0, state_dict_c, K)

    state_labels = {}
    for state_name in K.nodes():
        state_labels[state_name] = ' & '.join(prop for prop in state_name.split(' & ') if prop[0] != '!')

    return K, ap, state_dict_c
def create_rooms_wh(weighting_factor):
    K = nx.DiGraph()

    l1 = 'room1'
    not_l1 = '!room1'

    l2 = 'room2'
    not_l2 = '!room2'

    l3 = 'room3'
    not_l3 = '!room3'

    l4 = 'room4'
    not_l4 = '!room4'

    l5 = 'room5'
    not_l5 = '!room5'

    l6 = 'room6'
    not_l6 = '!room6'

    l7 = 'room7'
    not_l7 = '!room7'

    l8 = 'room8'
    not_l8 = '!room8'

    l9 = 'room9'
    not_l9 = '!room9'

    ap = [l1, l2, l3, l4, l5, l6, l7, l8, l9]  # , l10]

    s1 = ' & '.join([l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7, not_l8, not_l9])
    s2 = ' & '.join([not_l1, l2, not_l3, not_l4, not_l5, not_l6, not_l7, not_l8, not_l9])
    s3 = ' & '.join([not_l1, not_l2, l3, not_l4, not_l5, not_l6, not_l7, not_l8, not_l9])
    s4 = ' & '.join([not_l1, not_l2, not_l3, l4, not_l5, not_l6, not_l7, not_l8, not_l9])
    s5 = ' & '.join([not_l1, not_l2, not_l3, not_l4, l5, not_l6, not_l7, not_l8, not_l9])
    s6 = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, l6, not_l7, not_l8, not_l9])
    s7 = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, l7, not_l8, not_l9])
    s8 = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7, l8, not_l9])
    s9 = ' & '.join([not_l1, not_l2, not_l3, not_l4, not_l5, not_l6, not_l7, not_l8, l9])

    K.add_node(s1)
    K.add_node(s2)
    K.add_node(s3)
    K.add_node(s4)
    K.add_node(s5)
    K.add_node(s6)
    K.add_node(s7)
    K.add_node(s8)
    K.add_node(s9)

    K.add_edge(s1, s2, weight=0.3 * weighting_factor);
    K.add_edge(s2, s3, weight=0.3 * weighting_factor);
    K.add_edge(s2, s6, weight=0.3 * weighting_factor);
    K.add_edge(s3, s4, weight=0.3 * weighting_factor);
    K.add_edge(s4, s5, weight=0.3 * weighting_factor);
    K.add_edge(s4, s6, weight=0.3 * weighting_factor);
    K.add_edge(s6, s8, weight=0.3 * weighting_factor);
    K.add_edge(s7, s8, weight=0.3 * weighting_factor);
    K.add_edge(s8, s9, weight=0.3 * weighting_factor);

    K.add_edge(s2, s1, weight=0.3 * weighting_factor);
    K.add_edge(s3, s2, weight=0.3 * weighting_factor);
    K.add_edge(s6, s2, weight=0.3 * weighting_factor);
    K.add_edge(s4, s3, weight=0.3 * weighting_factor);
    K.add_edge(s5, s4, weight=0.3 * weighting_factor);
    K.add_edge(s6, s4, weight=0.3 * weighting_factor);
    K.add_edge(s8, s7, weight=0.3 * weighting_factor);
    K.add_edge(s8, s6, weight=0.3 * weighting_factor);
    K.add_edge(s9, s8, weight=0.3 * weighting_factor);

    K.add_edge(s1, s1, weight=0);
    K.add_edge(s2, s2, weight=0);
    K.add_edge(s3, s3, weight=0);
    K.add_edge(s4, s4, weight=0);
    K.add_edge(s5, s5, weight=0);
    K.add_edge(s6, s6, weight=0);
    K.add_edge(s7, s7, weight=0);
    K.add_edge(s8, s8, weight=0);
    K.add_edge(s9, s9, weight=0);

    return K, ap


def create_beep(weighting_factor):
    K = nx.DiGraph()
    idx = '_'

    cam = 'beep'
    no_cam = '!beep'

    ap = [cam]

    K.add_node(cam)
    K.add_node(no_cam)

    K.add_edge(cam, no_cam, weight=0*weighting_factor); 
    K.add_edge(no_cam, cam, weight=0.2*weighting_factor);
    K.add_edge(cam, cam, weight=0.2*weighting_factor); 
    K.add_edge(no_cam, no_cam, weight=0*weighting_factor);


    return K, ap


def create_arm_c(weighting_factor):
    K = nx.DiGraph()

    p = 'pickup'
    d = 'dropoff'
    l = 'push'

    p_c = 'pickupc'
    d_c = 'dropoffc'
    l_c = 'pushc'

    not_p = '!pickup'
    not_d = '!dropoff'
    not_l = '!push'

    not_p_c = '!pickupc'
    not_d_c = '!dropoffc'
    not_l_c = '!pushc'

    ap = [p, d, l, p_c, d_c, l_c]

    s_idle = ' & '.join([not_p, not_d, not_l, not_p_c, not_d_c, not_l_c])
    s_p = ' & '.join([p, not_d, not_l, not_p_c, not_d_c, not_l_c])
    s_d = ' & '.join([not_p, d, not_l, not_p_c, not_d_c, not_l_c])
    s_l = ' & '.join([not_p, not_d, l, not_p_c, not_d_c, not_l_c])

    s_p_c = ' & '.join([not_p, not_d, not_l, p_c, not_d_c, not_l_c])
    s_d_c = ' & '.join([not_p, not_d, not_l, not_p_c, d_c, not_l_c])
    s_l_c = ' & '.join([not_p, not_d, not_l, not_p_c, not_d_c, l_c])

    K.add_node(s_idle)
    K.add_node(s_p)
    K.add_node(s_d)
    K.add_node(s_l)

    K.add_node(s_p_c)
    K.add_node(s_d_c)
    K.add_node(s_l_c)

    # idle -> idle
    # p -> p_c
    # d -> d_c
    # l -> l_c
    # p_c -> p_c (aka carry)
    # p_c -> d
    # d_c -> p
    # l_c -> p
    # idle -> p
    # idle -> l
    # d_c -> idle
    # l_c -> idle

    # add tiny bit of weight to (to push agent to go back to idle)
    # d_c -> d_c
    # l_c -> l_c

    K.add_edge(s_idle, s_idle, weight=0 * weighting_factor)
    K.add_edge(s_p, s_p_c, weight=0.25 * weighting_factor)
    K.add_edge(s_d, s_d_c, weight=0.25 * weighting_factor)
    K.add_edge(s_l, s_l_c, weight=0.2 * weighting_factor)
    K.add_edge(s_p_c, s_p_c, weight=0.25 * weighting_factor)
    K.add_edge(s_p_c, s_d, weight=0.3 * weighting_factor)
    K.add_edge(s_d_c, s_p, weight=0.01 * weighting_factor)
    K.add_edge(s_l_c, s_p, weight=0.01 * weighting_factor)
    K.add_edge(s_idle, s_p, weight=0.01 * weighting_factor)
    K.add_edge(s_idle, s_l, weight=0.01 * weighting_factor)
    K.add_edge(s_d_c, s_idle, weight=0 * weighting_factor)
    K.add_edge(s_l_c, s_idle, weight=0 * weighting_factor)

    K.add_edge(s_d_c, s_d_c, weight=0.001 * weighting_factor)
    K.add_edge(s_l_c, s_l_c, weight=0.001 * weighting_factor)

    return K, ap

def generate_agents_c(ex):
    # loc1, ap_loc1 = af.create_mobility(1)

    cam1, ap_cam1 = af.create_camera(1)
    arm1, ap_arm1 = create_arm_c(2)
    scan1, ap_scan1 = af.create_scan(1)
    beep1, ap_beep1 = create_beep(1)

    if ex==3:
        loc1, ap_loc1, ap_loc_dict = create_rooms_wh_c(1)
        caps1 = [loc1, arm1, cam1]#, track1]
        ap1 = [ap_loc1, ap_arm1, ap_cam1]#, ap_track1]

        ag1 = Agent(ap1, caps1, 1)

        ag1.cap_names = ['loc', 'arm', 'camera']
        ag1.init_state = (ap_loc_dict['s7c'],
                          '!pickup & !dropoff & !push & !pickupc & !dropoffc & !pushc', '!camera')

        # agent 2
        loc2, ap_loc2 = loc1, ap_loc1 
        cam2, ap_cam2 = af.create_camera(1)
        beep2, ap_beep2 = create_beep(1)
        scan2, ap_scan2 = af.create_scan(1)

        caps2 = [loc2, beep2]#, track2]#, downlink2, batt2]
        ap2 = [ap_loc2, ap_beep2]#, ap_track2]#, ap_dl2, ap_batt2]

        ag2 = Agent(ap2, caps2, 2)

        ag2.cap_names = ['loc', 'beep']#,'track']#, 'downlink','batt']
        # ag2.init_state = ('!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & room6c & !room7c & !room8c & !room9c',
        #                   '!beep')
        ag2.init_state = (ap_loc_dict['s5c'], '!beep')
        # ag2.m_curr = '(GF(room1 & camera) & GF(room4 & camera))'

        # agent 3
        loc3, ap_loc3 = loc1, ap_loc1 
        cam3, ap_cam3 = af.create_camera(0.5)
        arm3, ap_arm3 = create_arm_c(0.8)
        caps3 = [loc3, arm3] #, downlink2, batt2, track2]
        ap3 = [ap_loc3, ap_arm3]

        ag3 = Agent(ap3, caps3, 3)

        ag3.cap_names = ['loc', 'arm']
        # ag3.init_state = ('!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c',
        #                   '!pickup & !dropoff & !carry & !lever')

        ag3.init_state = (ap_loc_dict['s3c'],
        '!pickup & !dropoff & !push & !pickupc & !dropoffc & !pushc')
        # ag3.m_curr = '(F(room3 & camera))'

        # agent 4
        loc4, ap_loc4 = loc1, ap_loc1 
        cam4, ap_cam4 = af.create_camera(0.9)
        arm4, ap_arm4 = create_arm_c(1)
        beep4, ap_beep4 = create_beep(1)
        scan4, ap_scan4 = af.create_scan(1)
        caps4 = [loc4, cam4, scan4, beep4] #, downlink2, batt2, track2]
        ap4 = [ap_loc4, ap_cam4, ap_scan4, ap_beep4]

        ag4 = Agent(ap4, caps4, 4)

        ag4.cap_names = ['loc', 'camera','scan', 'beep']
        # ag4.init_state = ('!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & room9c',
        #                   '!camera','!scan', '!beep')

        ag4.init_state = (ap_loc_dict['s8c'],
        '!camera', '!scan', '!beep')


        agent_list = [ag1, ag2, ag3, ag4]

        ap_list = set()

        for a in agent_list:
            ap_list = ap_list | set(a.ap_list)

        return agent_list, ap_list


    else:
        loc1, ap_loc1, ap_loc_dict = create_rooms_wh_small_c(1)
        caps1 = [loc1, arm1, cam1]  # , track1]
        ap1 = [ap_loc1, ap_arm1, ap_cam1]  # , ap_track1]

        ag1 = Agent(ap1, caps1, 1)

        ag1.cap_names = ['loc', 'arm', 'camera']
        ag1.init_state = (ap_loc_dict['s4c'],
                          '!pickup & !dropoff & !push & !pickupc & !dropoffc & !pushc', '!camera')

        # agent 2
        loc2, ap_loc2 = loc1, ap_loc1
        cam2, ap_cam2 = af.create_camera(1)
        beep2, ap_beep2 = create_beep(1)
        scan2, ap_scan2 = af.create_scan(1)

        caps2 = [loc2, beep2]  # , track2]#, downlink2, batt2]
        ap2 = [ap_loc2, ap_beep2]  # , ap_track2]#, ap_dl2, ap_batt2]

        ag2 = Agent(ap2, caps2, 2)

        ag2.cap_names = ['loc', 'beep']  # ,'track']#, 'downlink','batt']
        # ag2.init_state = ('!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & room6c & !room7c & !room8c & !room9c',
        #                   '!beep')
        ag2.init_state = (ap_loc_dict['s3c'], '!beep')
        # ag2.m_curr = '(GF(room1 & camera) & GF(room4 & camera))'

        # agent 3
        loc3, ap_loc3 = loc1, ap_loc1
        cam3, ap_cam3 = af.create_camera(0.5)
        arm3, ap_arm3 = create_arm_c(0.8)
        caps3 = [loc3, arm3]  # , downlink2, batt2, track2]
        ap3 = [ap_loc3, ap_arm3]

        ag3 = Agent(ap3, caps3, 3)

        ag3.cap_names = ['loc', 'arm']
        # ag3.init_state = ('!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c',
        #                   '!pickup & !dropoff & !carry & !lever')

        ag3.init_state = (ap_loc_dict['s5c'],
                          '!pickup & !dropoff & !push & !pickupc & !dropoffc & !pushc')
        # ag3.m_curr = '(F(room3 & camera))'

        # agent 4
        loc4, ap_loc4 = loc1, ap_loc1
        cam4, ap_cam4 = af.create_camera(0.9)
        arm4, ap_arm4 = create_arm_c(1)
        beep4, ap_beep4 = create_beep(1)
        scan4, ap_scan4 = af.create_scan(1)
        caps4 = [loc4, cam4, scan4, beep4]  # , downlink2, batt2, track2]
        ap4 = [ap_loc4, ap_cam4, ap_scan4, ap_beep4]

        ag4 = Agent(ap4, caps4, 4)

        ag4.cap_names = ['loc', 'camera', 'scan', 'beep']
        # ag4.init_state = ('!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & room9c',
        #                   '!camera','!scan', '!beep')

        ag4.init_state = (ap_loc_dict['s5c'],
                          '!camera', '!scan', '!beep')

        agent_list = [ag1, ag2, ag3, ag4]

        ap_list = set()

        for a in agent_list:
            ap_list = ap_list | set(a.ap_list)

        return agent_list, ap_list

def generate_agents(ex):
    # loc1, ap_loc1 = af.create_mobility(1)
    loc1, ap_loc1 = create_rooms_wh(1)

    cam1, ap_cam1 = af.create_camera(1)
    arm1, ap_arm1 = af.create_arm(2)
    scan1, ap_scan1 = af.create_scan(1)
    beep1, ap_beep1 = create_beep(1)

    if ex==3 or ex==7:
        caps1 = [loc1, arm1, scan1]#, track1]
        ap1 = [ap_loc1, ap_arm1, ap_scan1]#, ap_track1]

        ag1 = Agent(ap1, caps1, 1)

        ag1.cap_names = ['loc', 'arm', 'scan']
        ag1.init_state = ('!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & room7 & !room8 & !room9', '!pickup & !dropoff & !carry & !lever', 'scan')

        # agent 2
        loc2, ap_loc2 = loc1, ap_loc1
        cam2, ap_cam2 = af.create_camera(1)
        beep2, ap_beep2 = create_beep(1)
        scan2, ap_scan2 = af.create_scan(1)

        caps2 = [loc2, beep2]#, track2]#, downlink2, batt2]
        ap2 = [ap_loc2, ap_beep2]#, ap_track2]#, ap_dl2, ap_batt2]

        ag2 = Agent(ap2, caps2, 2)

        ag2.cap_names = ['loc', 'beep']#,'track']#, 'downlink','batt']
        ag2.init_state = ('!room1 & !room2 & !room3 & !room4 & !room5 & room6 & !room7 & !room8 & !room9', '!beep')
        # ag2.m_curr = '(GF(room1 & camera) & GF(room4 & camera))'

        # agent 3
        loc3, ap_loc3 = loc1, ap_loc1
        cam3, ap_cam3 = af.create_camera(0.5)
        arm3, ap_arm3 = af.create_arm(0.8)
        caps3 = [loc3, arm3] #, downlink2, batt2, track2]
        ap3 = [ap_loc3, ap_arm3]

        ag3 = Agent(ap3, caps3, 3)

        ag3.cap_names = ['loc', 'arm']
        ag3.init_state = ('!room1 & !room2 & room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9', '!pickup & !dropoff & !carry & !lever')
        # ag3.m_curr = '(F(room3 & camera))'

        # agent 4
        loc4, ap_loc4 = loc1, ap_loc1
        cam4, ap_cam4 = af.create_camera(0.9)
        arm4, ap_arm4 = af.create_arm(1)
        beep4, ap_beep4 = create_beep(1)
        scan4, ap_scan4 = af.create_scan(1)
        caps4 = [loc4, cam4, scan4, beep4] #, downlink2, batt2, track2]
        ap4 = [ap_loc4, ap_cam4, ap_scan4, ap_beep4]

        ag4 = Agent(ap4, caps4, 4)

        ag4.cap_names = ['loc', 'camera','scan', 'beep']
        ag4.init_state = ('!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & room9', '!camera','!scan', '!beep')

        if ex == 3:

            agent_list = [ag1, ag2, ag3, ag4]

            ap_list = set()

            for a in agent_list:
                ap_list = ap_list | set(a.ap_list)

            return agent_list, ap_list

        if ex == 7:

            # agent 5
            loc5, ap_loc5 = loc1, ap_loc1
            cam5, ap_cam5 = af.create_camera(1)
            arm5, ap_arm5 = af.create_arm(0.5)
            scan5, ap_scan5 = af.create_scan(1)
            beep5, ap_beep5 = create_beep(2)

            caps5 = [loc5, arm5, scan5]#, track1]
            ap5 = [ap_loc5, ap_arm5, ap_scan5]#, ap_track1]

            ag5= Agent(ap5, caps5, 5)

            ag5.cap_names = ['loc', 'arm', 'scan']
            ag5.init_state = ('!room1 & !room2 & !room3 & !room4 & !room5 & room6 & !room7 & !room8 & !room9', '!pickup & !dropoff & !carry & !lever', '!scan')

            # agent 6
            loc6, ap_loc6 = loc1, ap_loc1
            cam6, ap_cam6 = af.create_camera(1.5)
            arm6, ap_arm6 = af.create_arm(0.75)
            scan6, ap_scan6 = af.create_scan(2)
            beep6, ap_beep6 = create_beep(1.5)

            caps6 = [loc6, arm6, beep6]#, track2]#, downlink2, batt2]
            ap6 = [ap_loc6, ap_arm6, ap_beep6]#, ap_track2]#, ap_dl2, ap_batt2]

            ag6 = Agent(ap6, caps6, 6)

            ag6.cap_names = ['loc', 'arm', 'beep']#,'track']#, 'downlink','batt']
            ag6.init_state = ('!room1 & !room2 & !room3 & !room4 & room5 & !room6 & !room7 & !room8 & !room9', '!pickup & !dropoff & !carry & !lever', '!beep')
            # ag2.m_curr = '(GF(room1 & camera) & GF(room4 & camera))'

            # agent 7
            # loc7, ap_loc7 = create_rooms_wh(1.1)
            # cam7, ap_cam7 = af.create_camera(2.5)
            # arm7, ap_arm7 = af.create_arm(1.3)
            # caps7 = [loc7, arm7] #, downlink2, batt2, track2]
            # ap7 = [ap_loc7, ap_arm7]

            # ag7 = Agent(ap7, caps7, 7)

            # ag7.cap_names = ['loc', 'arm']
            # ag7.init_state = ('!room1 & !room2 & room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9', '!pickup & !dropoff & !carry & !lever')
            # ag3.m_curr = '(F(room3 & camera))'

            # agent 8
            loc8, ap_loc8 = create_rooms_wh(1)
            cam8, ap_cam8 = af.create_camera(0.1)
            scan8, ap_scan8 = af.create_scan(0.1)
            arm8, ap_arm8 = af.create_arm(0.1)
            beep8, ap_beep8 = create_beep(1.1)
            caps8 = [loc8, cam8, scan8, beep8] #, downlink2, batt2, track2]
            ap8 = [ap_loc8, ap_cam8, ap_scan8, ap_beep8]

            ag8 = Agent(ap8, caps8, 7)

            ag8.cap_names = ['loc', 'cam','scan,' 'beep']
            ag8.init_state = ('!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & room8 & !room9', '!camera','!scan','!beep')

            agent_list = [ag1, ag2, ag3, ag4, ag5, ag6, ag8]

            ap_list = set()

            for a in agent_list:
                ap_list = ap_list | set(a.ap_list)

            return agent_list, ap_list


    else:
        # caps1 = [loc1, arm1]#, scan1]#, track1]
        # ap1 = [ap_loc1, ap_arm1]#, ap_scan1]#, ap_track1]

        # ag1 = Agent(ap1, caps1, 1)

        # ag1.cap_names = ['loc','arm', 'scan']
        # ag1.init_state = ('!room1 & room2 & !room3', '!pickup & !dropoff & !carry & !lever')#, '!scan')
        # ag1.m_curr = '(GF(room3 & lever) & GF(room4 & lever))'


        caps1 = [loc1, arm1]#, track1]
        ap1 = [ap_loc1, ap_arm1]#, ap_track1]

        ag1 = Agent(ap1, caps1, 1)

        ag1.cap_names = ['loc', 'arm'] #scan
        ag1.init_state = ('!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & room7 & !room8 & !room9', '!pickup & !dropoff & !carry & !lever')

        # agent 2
        loc2, ap_loc2 = create_rooms_wh(1)
        cam2, ap_cam2 = af.create_camera(1)
        scan2, ap_scan2 = af.create_scan(1)

        caps2 = [loc2, cam2]#, track2]#, downlink2, batt2]
        ap2 = [ap_loc2, ap_cam2]#, ap_track2]#, ap_dl2, ap_batt2]

        ag2 = Agent(ap2, caps2, 2)

        ag2.cap_names = ['loc', 'cam']#,'track']#, 'downlink','batt']
        ag2.init_state = ('!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & room8 & !room9', '!camera')
        # ag2.m_curr = '(GF(room1 & camera) & GF(room4 & camera))'

        # agent 3
        loc3, ap_loc3 = create_rooms_wh(1)
        cam3, ap_cam3 = af.create_camera(0.5)
        arm3, ap_arm3 = af.create_arm(1)
        caps3 = [loc3, arm3] #, downlink2, batt2, track2]
        ap3 = [ap_loc3, ap_arm3]

        ag3 = Agent(ap3, caps3, 3)

        ag3.cap_names = ['loc', 'arm']
        ag3.init_state = ('!room1 & !room2 & room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9', '!pickup & !dropoff & !carry & !lever')
        # ag3.m_curr = '(F(room3 & camera))'

        # agent 4
        loc4, ap_loc4 = create_rooms_wh(1)
        cam4, ap_cam4 = af.create_camera(1)
        arm4, ap_arm4 = af.create_arm(1)
        beep4, ap_beep4 = create_beep(1)
        caps4 = [loc4, cam4, beep4] #, downlink2, batt2, track2]
        ap4 = [ap_loc4, ap_cam4, ap_beep4]

        ag4 = Agent(ap4, caps4, 4)

        ag4.cap_names = ['loc', 'cam','beep']
        ag4.init_state = ('!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & room9', '!camera','!beep')
        # ag4.m_curr = '(F(room2 & lever))'

        #agent 5
        loc5, ap_loc5 = af.create_mobility(1)
        cam5, ap_cam5 = af.create_camera(30)
        arm5, ap_arm5 = af.create_arm(5)
        scan5, ap_scan5 = af.create_scan(0.6)

        caps5 = [loc5, arm5, scan5]#, track1]
        ap5 = [ap_loc5, ap_arm5, ap_scan5]#, ap_track1]

        ag5 = Agent(ap5, caps5, 5)

        ag5.cap_names = ['loc','arm', 'scan']
        ag5.init_state = ('!room1 & !room2 & !room3 & !room4 & room5', '!pickup & !dropoff & !carry & !lever', '!scan')
        # ag5.m_curr = '(GF(scan & room3))'

    agent_list = [ag1, ag2, ag3, ag4, ag5, ag6, ag8]

    ap_list = set()

    for a in agent_list:
        ap_list = ap_list | set(a.ap_list)

    return agent_list, ap_list

def assign_agent_values(agent_state):
    ap_dict = {}

    ap_values = ' & '.join(agent_state).split(' & ')
    for ap in ap_values:
        if ap[0] == '~' or ap[0] == '!':
            value = "False"
            ap = ap[1:]
        else:
            value = "True"
        ap_dict[ap] = value
    return ap_dict

def assign_buchi_values(buchi_formula):
    ''' given a buchi transition, generate dictionary
    {binding: {ap: True/False/True_ex/False_ex}}
    '''
    ap_dict = {}
    bf_og = buchi_formula
    # buchi_formula = buchi_formula.replace('(1)','')
    buchi_formula = buchi_formula.replace('"','')
    ap_values = buchi_formula.replace('(','').replace(')','').replace('&','').replace('|','')
    ap_values = ap_values.split('  ')

    buchi_state_upd = buchi_formula

    if ap_values == ['1']:
        return {}

    for ap_binding in ap_values:
        ap, binding = ap_binding.split('.')
        if ap != '' and ap != ' ':
            if ap != '&' and ap != '|':
                if ap[0] == '~' or ap[0] == '!':
                    if 'not_' not in ap:
                        value = "False_ex"
                        ap = ap[1:]
                    else:
                        value = "True_ex"
                        ap = ap.split('not_')[1]
                else:
                    if 'not_' not in ap:
                        value = "True"
                    else:
                        value = "False"
                        ap = ap.split('not_')[1]
                        
            binding_num = int(binding[1:-1])
            if binding_num in ap_dict:
                ap_dict[binding_num][ap] = value
            else:
                ap_dict[binding_num] = {ap: value}
            # ap_dict[(ap, int(binding[1:-1]))] = value
    return ap_dict

def assign_binding_formula_values(buchi_formula, binding):
    ''' given a formula consisting of only &'s over aps, extract aps related to the specified binding
        generate dictionary {ap: {binding: True/False}}
    '''
    buchi_dict = assign_buchi_values(buchi_formula)

    if binding in buchi_dict:
        buchi_dict = assign_buchi_values(buchi_formula)[binding]
    else:
        return {}

    binding_ap_dict = {}

    for ap in buchi_dict:
        # ap, b = ap_binding
        # if b == binding:
        # binding_ap_dict[ap + '.['+ str(binding) + ']'] = buchi_dict[ap_binding]
        binding_ap_dict[ap] = buchi_dict[ap]

    # binding_ap_dict = {ap:buchi_dict[(ap,b)] for ap, b in buchi_dict if b == binding}

    return binding_ap_dict


##################################################################################################################################################################
# agent_state_dict = {'camera': 'True', 'room1':'True', 'scan':'True', 'pick':'True'}
# buchi_formula = "(camera.[2] & room1.[1] & !room1.[3] & scan.[3])"

# state_bindings = [1,2,3]
# for binding in [1,2,3]:

#     # look only at APs associated with that binding
#     binding_ap_dict = assign_binding_formula_values(buchi_formula, binding)
#     print(binding_ap_dict)

#     # since formula only consists of aps and &'s, if the aps of the agent state and the aps of the buchi binding formula match, then the state is possible
#     # in other words, the dictionary values of agent_state_dict must match ALL the values in binding_ap_dict
    
#     # find overlapping dict between binding_ap_dict and agent_state_dict
#     inters_dict = {x:agent_state_dict[x] for x in agent_state_dict if x in binding_ap_dict and agent_state_dict[x]==binding_ap_dict[x]}

#     # the agent state satisifies the binding for the buchi formula if inters_dict == binding_ap_dict
#     if inters_dict != binding_ap_dict:  
#         state_bindings.remove(binding)
#     else:
#         print('yay')
# print(state_bindings)


    # print(total_dict)

    # for b in total_dict:
    #     print(b, total_dict[b])
    #     print(b in buchi_formula)
    #     buchi_formula = re.sub(b, total_dict[b], buchi_formula)
    # print('p',buchi_formula)

    # buchi_formula = buchi_formula.replace('(1)', ' True ')

    # check_conflict = bool_parsing.nested_bool_eval(buchi_formula,True) # check_conflict = True -> robot can do that binding
    # print(binding, check_conflict, buchi_formula)

##################################################################################################################################################################
# @profile
def construct_prod_aut(agent_class, buchi_class, all_bindings, sub_buchi = False):

    if not sub_buchi:
        agent = agent_class.graph
        buchi = buchi_class.graph
        init_buchi = buchi_class.init_state
        accepting_states = buchi_class.accepting_states
    else:
        agent = agent_class
        buchi = buchi_class
        init_buchi = [x for x,y in buchi.nodes(data=True) if 'init_temp' in y][0]
        accepting_states = [x for x,y in buchi.nodes(data=True) if 'acc_temp' in y]
    
    agent_bindings = all_bindings

    agent_dict = {}
    for a in agent.nodes():
        agent_dict[a] = assign_agent_values(a)

    forward_prod = nx.MultiDiGraph()

    if not sub_buchi:
        state_list =[agent_class.init_state ]
    else:
        # pick a random state
        state_list = list(agent.nodes())

    new_list = set()
    count = 0

    temp = False


    # initialize init_state (i dont think i actually need this)
    forward_prod.add_node((state_list[0], init_buchi), buchi_state = init_buchi, bindings=agent_bindings, sync=" ")

    buchi_sync = buchi.copy()
    buchi_sync.remove_edges_from(list(buchi_sync.edges()))


    sync_transitions = {}

    buchi_formula_dict = nx.get_edge_attributes(buchi,'label')
    buchi_sync_dict = nx.get_edge_attributes(buchi,'sync')


    while len(state_list) != 0:
        # check_bindings = {b: False for b in all_bindings}
        
        for state in state_list:
            if count == 0:
                state_a = state
                buchi_state = init_buchi
                pi_current = " "
            else:
                # env_state_current = state[0][0]
                state_a = state[0]#[1]
                buchi_state = state[1]
                # pi_current = state[2]
            buchi_neighbors = list(buchi.successors(buchi_state))
            for neighbor_b in buchi_neighbors:
            # for state_a in agent_list:
                # sync_list = set()
                for neighbor_a in list(agent.successors(state_a)):

                    agent_state_dict = agent_dict[neighbor_a]
                    
                    
                    for edge_idx in buchi.get_edge_data(buchi_state,neighbor_b).keys():

                        buchi_formula = buchi_formula_dict[(buchi_state,neighbor_b, edge_idx)]
                        pi_sync = buchi_sync_dict[(buchi_state,neighbor_b, edge_idx)]
                        # buchi_formula = buchi.get_edge_data(buchi_state,neighbor_b)[edge_idx]['label']
                        # pi_sync = buchi.get_edge_data(buchi_state,neighbor_b)[edge_idx]['sync']

                    
                        neighbor_a_new = neighbor_a

                        buchi_binding_dict = assign_buchi_values(buchi_formula)

                        check_conflict = False
                        if buchi_formula == '1':
                            check_conflict = True
                            state_bindings = []
                        else:
                            # check if agent state satisifies each binding
                            state_bindings = agent_bindings.copy()
                            for binding in agent_bindings:
                                
                            
                                
                                # if binding is not in buchi_binding_dict, that means that there aren't required APs for that binding so any robot can do this binding at this state
                                if binding in buchi_binding_dict:
                                    # look only at APs associated with that binding
                                    binding_ap_dict = buchi_binding_dict[binding]
                                else:
                                    continue


                                # look only at APs associated with that binding
                                # binding_ap_dict = assign_binding_formula_values(buchi_formula, binding)
                                # # if binding_ap_dict is empty, that means that there aren't required APs for that binding so any robot can do this binding at this state
                                # if binding_ap_dict == {}:
                                #     continue

                                # since formula only consists of aps and &'s, if the truth values of the agent state and the truth values of the buchi binding formula match, 
                                # then the state is possible.
                                # in other words, the dictionary values of agent_state_dict must match ALL the values in binding_ap_dict

                                # method 1: if the buchi requires !scan and the robot does not have scan cap, it can still do it

                                inters_dict = {x: binding_ap_dict[x] for x in binding_ap_dict if (x in agent_state_dict and agent_state_dict[x] == binding_ap_dict[x].split('_ex')[0]) or (x not in agent_state_dict and binding_ap_dict[x].split('_ex')[0] == 'False')}



                                # method 2: find overlapping dict between binding_ap_dict and agent_state_dict. if there's no grounding, the robot can't do it
                                
                                # inters_dict = {x: agent_state_dict[x] for x in agent_state_dict if x in binding_ap_dict and agent_state_dict[x] == binding_ap_dict[x]}





                                # the agent state satisifies the binding for the buchi formula if inters_dict == binding_ap_dict
                                if inters_dict != binding_ap_dict:
                                    state_bindings.remove(binding)
                                # elif inters_dict == binding_ap_dict and buchi_state == neighbor_b:
                                #     print('a')

                            if len(state_bindings) != 0 and set(state_bindings).issubset(set(agent_bindings)):
                                check_conflict = True

                            
                        # if agent_class.id == 4 and buchi_formula =="(room1.[1] & pickup.[1] & scan.[2] & camera.[2] & room3.[2] & beep.[2])":
                        #     print('checking... ', state, buchi_formula, state_bindings, check_conflict) 
                        # # if state == (('!room1 & !room2 & !room3 & !room4 & room5', '!camera', '!scan'), '0', 'pi_a') and neighbor_a_new == ('!room1 & room2 & !room3 & !room4 & !room5', '!camera', '!scan') and neighbor_b == '1' and pi_sync == 'pi_a':
                        #     print('checking... ', state, buchi_formula, state_bindings, check_conflict)    
                            # sdfsdffffff   
                        '''
                        STILL NEED TO DO
                        if all possible states that transition to next buchi node cannot do removed_binding, then the robot overall cannot perform this binding
                        
                        for each removed_binding:
                        set agent_bindings = state_bindings
                            go and remove states in which their state bindings still contains removed_binding
                        '''


                        # add valid nodes/edges to product graph
                        if check_conflict:

                            # update buchi sync
                            # sync_transitions[(buchi_state, neighbor_b, pi_sync)] = (buchi_formula, state_bindings)
                            d = (buchi_state, neighbor_b, pi_sync)
                            v = (buchi_formula, state_bindings)
                            buchi_sync.add_edge(d[0], d[1], label=(v[0], d[2]), key=d[2], sync=d[2], bindings = v[1])

                            # for b in state_bindings:
                            #     check_bindings[b] = True
                                # sync_list.add(pi_sync)
                            #     # if neighbor_b in accepting_states:
                            #     #     buchi_sync.add_node(neighbor_b, accepting='F')
                            #     # else:
                            #     #     buchi_sync.add_node(neighbor_b)
                            #     if 
                            #     print(buchi_sync.get_edge_data(state_a,neighbor_a)["pi_sync"])
                            
                            #     buchi_sync.add_edge(buchi_state, neighbor_b, label=pi_sync)

                            if (neighbor_a_new, neighbor_b) not in forward_prod.nodes():
                                new_list.add((neighbor_a_new, neighbor_b))

                                if neighbor_b in accepting_states:
                                    # if buchi_state != neighbor_b:
                                    if not sub_buchi:
                                        forward_prod.add_node((neighbor_a_new, neighbor_b), buchi_state = neighbor_b, accepting='F')
                                    else:
                                        forward_prod.add_node((neighbor_a_new, neighbor_b), buchi_state=neighbor_b)
                                    # print(list(nx.get_node_attributes(forward_prod, 'accepting')))
                                    # asdfasdf
                                else:
                                    forward_prod.add_node((neighbor_a_new, neighbor_b), buchi_state = neighbor_b)#, bindings=state_bindings, sync=pi_sync)
                            weight = agent.get_edge_data(state_a,neighbor_a)["weight"]
                            # forward_prod.add_edge(((env_state_current, state_a), buchi_state),(neighbor_a_new, neighbor_b), weight=weight)
                            forward_prod.add_edge((state_a, buchi_state),(neighbor_a_new, neighbor_b), key=pi_sync,weight=weight, bindings=state_bindings, sync=pi_sync, formula=buchi_formula, buchi_transition = (buchi_state, neighbor_b))
                            
                            
                # print(sync_list)
                # for ap in sync_list:
                #     buchi_sync.add_edge(buchi_state, neighbor_b, label=pi_sync)

        
        state_list = new_list.copy()
        new_list = set()
        count += 1


    # remove any unreached states
    # temp = forward_prod.copy()
    # for node in temp.nodes():
    #     if node not in nx.get_node_attributes(temp, "bindings"):
    #         forward_prod.remove_node(node)

        
    # for d,v in sync_transitions.items():
    #     buchi_sync.add_edge(d[0], d[1], label=(v[0], d[2]), key=d[2], sync=d[2], bindings = v[1])

    return forward_prod, buchi_sync

def powerset(iterable):
    "powerset([1,2,3]) --> () (1,) (2,) (3,) (1,2) (1,3) (2,3) (1,2,3)"
    s = list(iterable)
    p_set = itertools.chain.from_iterable(itertools.combinations(s, r) for r in range(1,len(s)+1))
    return set(p_set)

from collections import deque 
def remove_unreachable_bindings2(prod_aut, init_state, buchi_state_list, all_bindings):
    binding_combos = list(powerset(all_bindings))
    binding_combos.sort(key=len, reverse=True)
    possible_binding_combos = set()
    not_bindings = set()

    queue = deque(binding_combos)
    while len(queue) != 0:
        bindings = queue.popleft()
        shortest_path, shortest_dist = find_path_attribute(prod_aut, init_state[0], init_state[1], 'bindings', bindings)

        if len(shortest_path) == 0:
            not_bindings.add(bindings)
        else:
            possible_binding_combos.add(bindings)
            bindings_powerset = powerset(bindings)
            for b in bindings_powerset:
                if b not in not_bindings and b not in possible_binding_combos:
                    queue.append(b)

    return possible_binding_combos
def check_state_with_buchi(buchi_formula, buchi_binding_dict, agent_state_dict, agent_bindings):
    check_conflict = False
    if buchi_formula == '1':
        check_conflict = True
        state_bindings = []
    else:
        # check if agent state satisifies each binding
        state_bindings = agent_bindings.copy()
        for binding_combo in agent_bindings:
            # if binding is not in buchi_binding_dict, that means that there aren't required APs for that binding so any robot can do this binding at this state

            if set(binding_combo).isdisjoint(set(buchi_binding_dict.keys())):
                continue

            bindings_relevant = set(binding_combo).intersection(set(buchi_binding_dict.keys()))

            binding_ap_dict = {}
            binding_ap_dict = {}
            for b in bindings_relevant:
                binding_ap_dict = {**binding_ap_dict, **buchi_binding_dict[b]}

            inters_dict = {x: binding_ap_dict[x] for x in binding_ap_dict if (
                    x in agent_state_dict and agent_state_dict[x] == binding_ap_dict[x].split('_ex')[0]) or (
                                   x not in agent_state_dict and binding_ap_dict[x].split('_ex')[0] == 'False')}

            # the agent state satisifies the binding for the buchi formula if inters_dict == binding_ap_dict
            if inters_dict != binding_ap_dict:
                state_bindings.remove(binding_combo)

        if len(state_bindings) != 0 and set(state_bindings).issubset(set(agent_bindings)):
            check_conflict = True

    return check_conflict
def remove_unreachable_bindings(prod_aut, buchi, init_state, path, all_bindings, final_state=None, combos=False):
    ''' given a product automaton, determine which bindings the agent cannot do. Remove the corresponding paths

    Need to check every combination of bindings.

    for every combination it can do, it can do subsets of those combinations
    start off by checking if robot can do all bindings. if yes, then it can do all combinations
    else:
        check every (n-1) combinations until we've exhausted everything

    '''

    # parse buchi path into self transitions
    self_transitions = {}
    path_copy = path.copy()
    for n1, n2, pi_sync in path_copy:
        if n1 == n2:
            # path.remove((n1, n2, pi_sync))
            self_transitions[n1] = pi_sync
    # include loop in accepting state
    # acc = path[-1][1]
    # path.append((acc, acc, self_transitions[acc]))

    # if all_bindings is a simple list, need to generate combos of bindings
    if not combos:
        binding_combos = list(powerset(all_bindings))
    else:
        binding_combos = all_bindings

    if not final_state:
        final_state = init_state[1]
    else:
        final_state, final_pi = final_state[1:]

    binding_combos.sort(key=len, reverse=True)
    binding_combos_checked = binding_combos.copy()


    possible_binding_combos = set()
    not_bindings = set()
    while len(binding_combos_checked) != 0:
        binding_combos_checked_set = set(binding_combos_checked.copy())
        bindings = binding_combos_checked[0]

        # for bindings in binding_combos:

        # if we're checking the very first self-transition, first check that agent's initial state does not violate it
        if len(path) == 1:
            buchi_formula = buchi.get_edge_data(path[0][0], path[0][1], key= path[0][2])['label']
            buchi_binding_dict = assign_buchi_values(buchi_formula)
            agent_state_dict = assign_agent_values(init_state[0])
            if not check_state_with_buchi(buchi_formula, buchi_binding_dict, agent_state_dict, [bindings]):
                shortest_path = []
                shortest_dist = float('inf')
            else:
                shortest_path = [init_state[0]]
                shortest_dist = 0
        else:
            shortest_path, shortest_dist = find_path_attribute(prod_aut, init_state[0], init_state[1],
                                                           'bindings', bindings, buchi_path=path)
        # shortest_path, shortest_dist = stb.find_shortest_path_robot(prod_aut, buchi, init_state[0], path_copy,
        #                                                             self_transitions, bindings)
        # print('printing', bindings, shortest_path,)

        if len(shortest_path) == 0:
            # binding_combos_checked.remove(bindings)
            not_bindings.add(bindings)
            binding_combos_checked_set.discard(bindings)
        else:
            bindings_powerset = powerset(bindings)
            possible_binding_combos.update(bindings_powerset)
            binding_combos_checked_set -= bindings_powerset
        binding_combos_checked = sorted(list(binding_combos_checked_set), key=len, reverse=True)
        # print('end', len(shortest_path), binding_combos_checked, binding_combos_checked_set)
    # print('possible', possible_binding_combos)

    # update prod_aut - remove any bindings that aren't possible
    # nodes = (
    #     node
    #     for node, data
    #     in prod_aut.nodes(data=True)
    #     if not_bindings.issubset(set(data["bindings"])) and node != init_state
    #     )

    # subgraph = prod_aut.subgraph(nodes).copy()

    # for b in not_bindings:
    #     for state in subgraph.nodes():
    #         try:
    #             binding_labels[state].remove(b)
    #         except ValueError as e:
    #             # print(e)
    #             pass  
    #         if len(binding_labels[state]) == 0 :
    #             # possible_bindings.pop(state, None)
    #             prod_aut.remove_node(state)
    
    # nx.set_node_attributes(prod_aut, binding_labels, name="bindings")

    return possible_binding_combos

# @profile
def update_sync_buchi_with_possible(prod_aut, buchi_sync_og, possible_binding_combos):
    ''' given possible bindings, remove unreachable nodes/edges
    '''

    def find_keys(dict_formula, dict_edge, dict_sync, formula, buchi_transition, pi_sync):
        keys1 = set(k for k, v in dict_formula.items() if v == formula)
        keys2 = set(k for k, v in dict_edge.items() if v == buchi_transition)
        keys3 = set(k for k, v in dict_sync.items() if v == pi_sync)
        return list(keys1 & keys2 & keys3)



    buchi_sync = buchi_sync_og.copy()
    sync_transitions = {}

    # all edges in prod_aut with relevant buchi formula
    # prod_aut_attributes = nx.get_edge_attributes(prod_aut)
    prod_formula_dict =nx.get_edge_attributes(prod_aut,'formula')
    prod_edge_dict = nx.get_edge_attributes(prod_aut,'buchi_transition')
    prod_bindings_sync = nx.get_edge_attributes(prod_aut,'sync')
    prod_bindings_dict = nx.get_edge_attributes(prod_aut,'bindings')

    for buchi_state in buchi_sync_og.nodes():
        buchi_neighbors = list(buchi_sync_og.successors(buchi_state))

        for buchi_neighbor in buchi_neighbors:
            # for each edge from (buchi_state, buchi_neighbor)
            for edge_idx in buchi_sync_og.get_edge_data(buchi_state, buchi_neighbor).keys():
                buchi_formula, pi_sync = buchi_sync_og.get_edge_data(buchi_state, buchi_neighbor)[edge_idx]['label']
                buchi_bindings = method2.extract_bindings(buchi_formula) 
                buchi_bindings = set([int(binding) for binding in buchi_bindings])


                
                prod_states = set()
                binding_combos = set()

                edges2check = find_keys(prod_formula_dict, prod_edge_dict, prod_bindings_sync, buchi_formula, (buchi_state, buchi_neighbor), pi_sync)
                for e in edges2check:
                    e1,e2,k = e
                    for combo in possible_binding_combos:
                        combo_set = set(combo)
                        if (combo_set.issubset(prod_bindings_dict[e]) or 
                             combo_set.isdisjoint(buchi_bindings)):
                            prod_states.add((e1, e2, pi_sync))

                            combo_ord = tuple(sorted(list(combo)))
                            binding_combos.add(combo_ord)


                '''
                # find prod_states in which 
                - transition to buchi_neighbor on the pi_sync edge
                - the bindings for that state can be satisfied by the assigned agent bindings OR do not matter to the assigned bindings
                '''
                # prod_states = set()
                # binding_combos = set()

                # for edge in prod_aut.edges(data=True):
                #     e1, e2, data = edge

                #     for combo in possible_binding_combos:
                #         combo_set = set(combo)
                #         if (data['formula'] == buchi_formula and 
                #             e2[1] == buchi_neighbor and 
                #             data['sync'] == pi_sync and 
                #             (combo_set.issubset(data['bindings']) or 
                #              combo_set.isdisjoint(buchi_bindings))):
                #             prod_states.add((e1, e2, pi_sync))
                #             binding_combos.add(combo)

                if prod_states:
                    sync_transitions[(buchi_state, buchi_neighbor, pi_sync)] = (buchi_formula, list(binding_combos))

    buchi_sync.remove_edges_from(list(buchi_sync.edges()))
    for d, v in sync_transitions.items():
        buchi_sync.add_edge(d[0], d[1], label=(v[0], d[2], v[1]), sync=d[2], key=d[2], binding_combos=v[1])

    return buchi_sync


def update_sync_buchi_with_possible_ineff(prod_aut, buchi_sync_og, possible_binding_combos):
    ''' given possible bindings, remove unreachable nodes/edges
    '''
    buchi_sync = buchi_sync_og.copy()
    buchi_sync.remove_edges_from(list(buchi_sync.edges()))
    sync_transitions={}



    for buchi_state in buchi_sync_og.nodes():
        buchi_neighbors = list(buchi_sync_og.successors(buchi_state))

        for buchi_neighbor in buchi_neighbors:
            
            # for each edge from (buchi_state, buchi_neighbor)
            for edge_idx in buchi_sync_og.get_edge_data(buchi_state,buchi_neighbor).keys():  
                buchi_formula, pi_sync = buchi_sync_og.get_edge_data(buchi_state,buchi_neighbor)[edge_idx]['label']
                buchi_bindings = method2.extract_bindings(buchi_formula) 
                buchi_bindings = [int(binding) for binding in buchi_bindings]

                '''
                # find prod_states in which 
                - transition to buchi_neighbor on the pi_sync edge
                - the bindings for that state can be satisfied by the assigned agent bindings OR do not matter to the assigned bindings
                '''
                prod_states = set()
                binding_combos = set()
                # for node, data in prod_aut.nodes(data=True):
                #     for combo in possible_binding_combos:
                #         print(node,data)
                #         if data['buchi_state'] == buchi_neighbor and data['key'] == pi_sync and (set(combo).issubset(data['bindings']) or set(combo).isdisjoint(set(buchi_bindings))):
                #             prod_states.add(node)
                #             binding_combos.add(combo)

                for edge in prod_aut.edges(data=True):
                    # print(prod_aut.edges(data=True))
                    # print(prod_aut.get_edge_data(edge[0], edge[1], key= 'pi_a'))
                    # sdf
                    e1, e2, data = edge
                    
                    for combo in possible_binding_combos:
                        # if edge == ('0', '0', 'pi_a'):
                        #     print('here', pi_sync)
                        #     sdf
                        
                            
                    # print(e1, e2, data)
                        # for edge_idx, attributes in data:
                        # edge_idx = pi_sync
                        # if e1== (('room1 & !room2 & !room3 & !room4 & !room5', '!camera', '!scan'), '0', 'pi_a') and e2 == (('room1 & !room2 & !room3 & !room4 & !room5', '!camera', '!scan'),'2','pi_a'):
                        #     print('checking edge')
                        #     if data['formula'] == buchi_formula and e2[1] == buchi_neighbor and data['key'] == pi_sync and (set(combo).issubset(data['bindings']) or set(combo).isdisjoint(set(buchi_bindings))):
                        #         print(True)
                        #         print(data['formula'], combo, data['bindings'], binding_combos)
                        #     else:
                        #         print(False)
                    

                        if data['formula'] == buchi_formula and e2[1] == buchi_neighbor and data['sync'] == pi_sync and (set(combo).issubset(data['bindings']) or set(combo).isdisjoint(set(buchi_bindings))):
                            prod_states.add((e1,e2,pi_sync))
                            binding_combos.add(combo)
                            # print('true')
                        # if test:
                        #     if data['formula'] == buchi_formula and e1[1] == '0' and e2[1] == '1' and e2[1] == buchi_neighbor and combo == (2,) and pi_sync == 'pi_a' and data['key'] == pi_sync:
                        #         print(edge)
                        #         print(buchi_formula)
                        #         print(pi_sync, combo, data['bindings'], buchi_bindings, (set(combo).issubset(data['bindings']) or set(combo).isdisjoint(set(buchi_bindings))))
                        #         print(binding_combos)
                

                    


                # prod_states = [n for n,v in prod_aut.nodes(data=True) if v['buchi_state'] == buchi_neighbor and v['sync'] == pi_sync and (set(assigned_bindings).issubset(v['bindings']) or set(assigned_bindings).isdisjoint(set(buchi_bindings)))] 
                
                # print(buchi_neighbor, pi_sync, prod_states )
                

                    sync_transitions[(buchi_state, buchi_neighbor, pi_sync)] = (buchi_formula, list(binding_combos))
                # for state in prod_states:
                #     state_bindings = nx.get_node_attributes(prod_aut, "bindings")[state]
                #     # print(state_bindings, buchi_bindings)
                #     if not set(assigned_bindings).isdisjoint(set(buchi_bindings)):
                #         sync_transitions[(buchi_state, buchi_neighbor, pi_sync)] = buchi_formula
    # print('sync_transitions',sync_transitions)
    for d,v in sync_transitions.items():
        buchi_sync.add_edge(d[0], d[1], label=(v[0], d[2], v[1]), sync=d[2], key=d[2], binding_combos = v[1] )


    # print('buchi_sync edges',buchi_sync.edges(data=True))
    # aaaaaaaa

    return buchi_sync



# def remove_unreachable_bindings(prod_aut, init_state, buchi_state_list, all_bindings):
#     ''' given a product automaton, determine which bindings the agent cannot do. Remove the corresponding paths

#     this method checks if there exists a binding that can reach each buchi state. otherwise, remove binding. THIS IS INCORRECT - ONLY CARE ABOUT PATH TO ACCEPTING STATE
#     '''
#     possible_bindings = all_bindings
#     binding_labels = nx.get_node_attributes(prod_aut, 'bindings')
    
#     for buchi_state in buchi_state_list:
#         check_bindings = {b: False for b in possible_bindings}


#         # check if there exists a state that has binding b. If not, that binding is not possible
#         for node, data in prod_aut.nodes(data=True):
#             if data.get("buchi_state") == buchi_state and node != init_state:
#                 for b in data.get("bindings"):
#                     check_bindings[b] = True
#         print(check_bindings)

#         for d, v in check_bindings.items():
#             if not v:
#                 possible_bindings.remove(d)
#                 # remove all edges with binding d

#                 for node, data in prod_aut.nodes(data=True):
#                     print(node, data.get("bindings"))
#                 print(d, len(prod_aut.nodes), len(binding_labels))

#                 nodes = (
#                     node
#                     for node, data
#                     in prod_aut.nodes(data=True)
#                     if d in data.get("bindings") and node != init_state
#                 )
#                 print('here')

#                 for state in nodes:
#                     try:
#                         binding_labels[state].remove(state)
#                     except ValueError:
#                         pass  
#                     if len(binding_labels[state]) == 0 :
#                         possible_bindings.pop(state, None)
#                         prod_aut.remove_node(state)





#     nx.set_node_attributes(prod_aut, binding_labels, "bindings")

#     return prod_aut, possible_bindings
    # buchi_state_dict = nx.get_node_attributes(prod_aut, 'buchi_state')
    # for state in buchi_state_list:


# @profile
def find_path_attribute(aut, start_state, init_buchi, attribute, attribute_list, final_state = None, buchi_path=None):
    '''
    start state: ('loc1 & !loc2 & !loc3', 'box' )

    attribute_list: only find path where attribute is in this list

    '''
    # TODO: the
    # issue is the
    # final_state
    # bit - how
    # to
    # find
    # a
    # path
    # to
    # a
    # NOT
    #  accepting state? currently assumes the final state only has one self-trans but 1) there's not a self-trans
    #  currently and 2) specify which pi

    if not final_state and not buchi_path:
        final_states = nx.get_node_attributes(aut, 'accepting')
    elif not buchi_path:    # find states with given buchi state
        final_states = [n for n,v in aut.nodes(data=True) if v['buchi_state'] == final_state]
    else:
        # int_state = (buchi_state, pi) to find a path to buchi_state with specific self-transition.
        # if int_state = (buchi_state,) then we don't need to find a specific self-transition
        if buchi_path:
            final_state, pi = buchi_path[-1][1:]

            # only look at prod aut with the specific path and self trans

            # get subgraph with the relevant edges
            aut_edges = [(from_node, to_node, data['sync']) for from_node, to_node, data in aut.edges(data=True)
                           if (from_node[1], to_node[1], data['sync']) in buchi_path]
            aut = aut.edge_subgraph(aut_edges)
            final_states = [n for n, v in aut.nodes(data=True) if
                            v['buchi_state'] == final_state]  # and v['sync'] == pi]


    init_state = (start_state, init_buchi)
    shortest_dist = float('inf')
    shortest_path  = []

    attribute_list_set = set(attribute_list)

    # subgraph = nx.MultiDiGraph()
    # edges_to_keep = (edge for edge in aut.edges(data=True) if attribute_list_set.issubset(set(edge[2]["bindings"])))
    # subgraph.add_edges_from(edges_to_keep)

    # # only look at part of the buchi that has binding b at every node
    # edges = []
    # # print('ed')
    # # print(aut.edges(data=True))
    G = nx.MultiDiGraph()

    # # for final in final_states:
    # #     # if nx.has_path(aut, init_state , final):
    # #     # if there's KeyError, means there is no path to the end state
    # #     # t = time.time()

    # #     # check if path can be found to this accepting state
    # #     num = 0
    # #     try:
    # #         if nx.has_path(aut, init_state, final):
    # #             for path in nx.all_simple_edge_paths(aut, init_state, final):  
    # #                 valid_path = True 
    # #                 for e1,e2,pi in path:
    # #                     data = aut[e1][e2][pi]
    # #                     if not attribute_list_set.issubset(set(data["bindings"])):
    # #                         valid_path = False
    # #                 if valid_path:
    # #                     num += 1


    # #     except Exception as e:
    # #         pass
    # #         # try the next accepting state 

    # # if num > 0:
    # #     return [1], 1
    # # else:
    # #     return [], 0

    # ######

    for edge in aut.edges(data=True):
        if attribute_list_set.issubset(set(edge[2]["bindings"])):

            G.add_edge(edge[0],edge[1],key=edge[2]['sync'], bindings = edge[2]["bindings"])



    # subgraph = aut.edge_subgraph(list(G.edges(keys=True)))
    



    # if attribute == "bindings":
    #     edges = [
    #         node
    #         for node, data
    #         in aut.nodes(data=True)
    #         if set(attribute_list).issubset(set(data[attribute])) or node == init_state
    #         # if set(data.get(attribute)).issubset(set(attribute_list)) or node == init_state
    #     ]
    # # elif attribute == "sync":
    # #     nodes = (
    # #     node
    # #         for node, data
    # #         in aut.nodes(data=True)
    # #         if (data[attribute] in attribute_list and data["buchi_state"] in [init_buchi,final_state]) or node == init_state
    # #     )
    # #     subgraph = aut.subgraph(nodes).copy()

    # for node, data in aut.nodes(data=True):
    #     print('data', node, init_state)
    #     print(data.get(attribute), attribute_list, data.get(attribute) in attribute_list)
    #     if set(data.get(attribute)).issubset(set(attribute_list)):
    #         print('yes')
    
    # print('all',len(list(nodes)), nodes)

    def has_path_to_targets(G, start, targets):
        target_nodes_set = set(targets)
        for node in nx.descendants(G, start):
            if node in target_nodes_set:
                return True
        return False

    if init_state in G:
        if has_path_to_targets(G, init_state, final_states):
            return [1], 1
    return [], 0

    
    

    # for final in final_states:
    #     # if nx.has_path(aut, init_state , final):
    #         # if there's KeyError, means there is no path to the end state
    #         # t = time.time()

    #         # check if path can be found to this accepting state
    #         try:
    #             if nx.has_path(subgraph, init_state, final):
    #                 return [1], 1
    #         except Exception as e:
    #             # try the next accepting state
    #             pass

    #         # try:
    #         #     dist, path = nx.single_source_dijkstra(subgraph, init_state , final, weight='weight')
    #         #     # print('path found', final)
    #         #     # print('time for one out of ', test, ' iteration of djikstra: ', time.time()-t)
    #         #     if dist < shortest_dist:
    #         #         shortest_path = path
    #         #         shortest_dist = dist
    #         # except Exception as e:
    #         #     # try the next accepting state
    #         #     pass
    # # return shortest_path, shortest_dist
    # return [], 0



import os
def buchi_to_dot(buchi,name):
    write_dot(buchi, name) # nx built-in function

    f = open(name,'r')
    filedata = f.read()
    f.close()

    newdata = filedata.replace(r'\"circle\"', 'circle')

    f = open(name,'w')
    f.write(newdata)
    f.close()

    os.system("dot -Tpng -Gdpi=300 " + name + " > " + name[:-4] + ".png")

    return


def main(spec, agent_list, bindings_list, ap_list, buchi_class, save_files = False, sims=True):
    
    max_time = 0
    buchi = buchi_class.graph

    # check if all bindings can be assigned. If not, no need to go through the rest of the computation
    check_possible = set(bindings_list)


    for ag in agent_list:
        ag.time = 0
        print(ag.id)
        t0_1 = time.time()
        ag.prod_aut, ag.buchi_sync = construct_prod_aut(ag, buchi_class, bindings_list)
        tf_1 = time.time()

        # plot portion of agent 1's product aut for the paper
        # write_dot(ag.prod_aut, 'prod_aut.dot')
        # G = nx.MultiDiGraph()
        # if ag.id == 1:
        #     aut = ag.prod_aut

        #     for edge in aut.edges(data=True):
        #         e1, e2, data = edge
        #         e_poss1 = [(('room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9', '!pickup & !dropoff & !carry & !lever', '!scan'), '1'), \
        #                     (('!room1 & room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9', 'pickup & !dropoff & !carry & !lever', '!scan'), '1'), \
        #                     (('!room1 & room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9', 'pickup & !dropoff & !carry & !lever', 'scan'), '1')]

        #         e_poss2 = [(('room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9', '!pickup & !dropoff & !carry & !lever', 'scan'), '2'), \
        #                     (('!room1 & room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9', 'pickup & !dropoff & !carry & !lever', '!scan'), '2'), \
        #                     (('!room1 & room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9', 'pickup & !dropoff & !carry & !lever', 'scan'), '2')]
        #         if e1 in e_poss1 and e2 in e_poss2 and data['sync'] == 'pi_2':


        #             G.add_edge(e1,e2,key=data['sync'])


        #     subgraph = aut.edge_subgraph(list(G.edges(keys=True)))
        #     print(len(subgraph.nodes()))
        #     write_dot(subgraph, 'prod_aut_partial.dot')
        #     aa


        # if save_files:
        #     buchi_to_dot(ag.buchi_sync, 'buchi_sync_before_ordered' + str(ag.id) + '.dot')

        t0_2 = time.time()
        # prune prod aut based on which bindings the agent can do
        ag.possible_bindings = remove_unreachable_bindings(ag.prod_aut,(ag.init_state, buchi_class.init_state), list(buchi.nodes()), bindings_list)

        # print(len(ag.prod_aut.nodes()),len(ag.prod_aut.edges()))
        tf_2 = time.time()

        # ag.possible_bindings = {(1,),(2,),(3,),(1,2),(2,3),(1,3),(1,2,3)}

        # check if all bindings can be assigned. If not, no need to go through the rest of the computation
        if check_possible:
            check_possible = check_possible.difference( {item for sublist in ag.possible_bindings for item in sublist})


        

        

        # if ag.id == 3:
        #     n1 = (('!room1 & !room2 & !room3 & !room4 & !room5 & room6 & !room7 & !room8 & !room9', '!pickup & !dropoff & !carry & !lever'), '2')
        #     n2 =(('!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & room9', 'pickup & !dropoff & !carry & !lever'), '0')
        #     print(ag.prod_aut.has_node(n1), ag.prod_aut.has_node(n2), ag.prod_aut.has_edge(n1,n2))
        #     dist, path = nx.single_source_dijkstra(ag.prod_aut, n1 , n2, weight='weight')
        #     print(path)
        #     sdffs

        # if ag.id == 1:
        #     ag.assigned_bindings = []
        #     test = False
        
        # elif ag.id == 2:
        #     ag.assigned_bindings = [2]
        #     # print(nx.get_node_attributes(ag.prod_aut, "bindings")[(('!room1 & room2 & !room3 & !room4 & !room5', '!camera', '!scan'), '1', 'pi_a')])
        #     test = False
        # elif ag.id == 3:
        #     ag.assigned_bindings = [1,3]
        # else:
        #     test = True

        # ag.buchi_possible = ag.buchi_sync
        t0_5 = time.time()
        ag.buchi_possible = update_sync_buchi_with_possible(ag.prod_aut, ag.buchi_sync, ag.possible_bindings)
        tf_5 = time.time()


        if save_files:
            buchi_to_dot(ag.buchi_possible, 'buchi_sync_possible_ordered' + str(ag.id) + str(ag.id) + '.dot')

        # print(ag.buchi_possible.edges(data=True))
        # print(ag.buchi_possible['0']['0'])
        # sdf
        print('possible: ',ag.possible_bindings)
        print('time to construct prod aut for agent', ag.id, tf_1-t0_1)
        print('time to prune prod aut for agent', ag.id, tf_2-t0_2)
        print('time to update buchi for agent', ag.id, tf_5-t0_5)

        ag.time += tf_1 - t0_1
        ag.time += tf_2 - t0_2
        ag.time += tf_5 - t0_5

        # theoretically, previous calculations were all in parallel. So only include the time that was the longest
        if ag.time > max_time:
            max_time = ag.time


    # not all bindings can be assigned, so no need to go through the rest of the computation
    if check_possible:
        print('1 no possible team assignment', max_time)
        
        return float('inf'), float('inf')

        


        

        
    t0_3 = time.time()
    sync_buchi_path, self_transitions, team_assignment = stb.assign_bindings2robots(agent_list, bindings_list, buchi_class.init_state, buchi_class.graph)
    tf_3 = time.time()
    time_dfs = tf_3-t0_3
    print('time to do dfs', tf_3-t0_3)

    if not team_assignment:
        # not all bindings can be assigned, so no need to go through the rest of the computation
        print('2 no possible team assignment', max_time + time_dfs)
        
        return float('inf'), float('inf')

    

    # trying a different team 
    # team_assignment_copy = team_assignment.copy()
    # for agent in team_assignment_copy:
    #     if agent.id != 5 and agent.id != 4:
    #         team_assignment.pop(agent)

    agent_test = {}
    for agent in team_assignment:
        agent_test[agent.id] = team_assignment[agent]
        if agent.id == 5:
            agent.assigned_bindings = {3}
            team_assignment[agent] = {3}
        print(agent.id, team_assignment[agent])

    print('dfs: ', sync_buchi_path, self_transitions, agent_test)

    # print(sync_buchi_path, self_transitions)

    max_time_synth = 0

    for ag in team_assignment: 
        ag.assigned_bindings = team_assignment[ag]
        print('\n ROBOT ', ag.id, '------------------------------------')
        t0_4 = time.time()
        ag.path_nosync, ag.cost = stb.find_shortest_path_robot(ag.prod_aut, buchi, ag.init_state, sync_buchi_path, self_transitions, ag.assigned_bindings)
        tf_4 = time.time()

        
        # print('time to synthesize non sync behavior for agent', ag.id, tf_4-t0_4)
        ag.time += tf_4-t0_4

        # theoretically, previous calculations were all in parallel. So only include the time that was the longest
        if tf_4-t0_4 > max_time_synth:
            max_time_synth = tf_4-t0_4
        

        # remove init state
        # ag.path_nosync.pop(0)

        # # for testing
        # if ag.id == 4:
        #     print('a')
        #     ag.path_nosync[-1] = [(('room1 & !room2 & !room3 & !room4 & !room5', '!camera', '!beep'), '1', 'pi_a')]


        # print(ag.path_nosync)
        # print('no sync', ag.id, ag.path_nosync)


    for ag in team_assignment:
        print(ag.id, ag.assigned_bindings)
        print(ag.path_nosync)

    print('\n \n final behavior')
    # stb.synthesize_team_behavior(team_assignment, buchi, sync_buchi_path)


    # reg_pretty = {'room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room1', \
    #               '!room1 & room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room2', \
    #               '!room1 & !room2 & room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room3', \
    #                '!room1 & !room2 & !room3 & room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room4', \
    #                '!room1 & !room2 & !room3 & !room4 & room5 & !room6 & !room7 & !room8 & !room9': 'room5', \
    #                '!room1 & !room2 & !room3 & !room4 & !room5 & room6 & !room7 & !room8 & !room9': 'room6', \
    #                '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & room7 & !room8 & !room9': 'room7', \
    #                '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & room8 & !room9': 'room8', \
    #                '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & room9': 'room9'
    #
    #                   }

    reg_pretty = {
        'room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'room1', \
        '!room1 & room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'room2', \
        '!room1 & !room2 & room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'room3', \
        '!room1 & !room2 & !room3 & room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'room4', \
        '!room1 & !room2 & !room3 & !room4 & room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'room5', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'room6', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'room7', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'room8', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'room9', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'room1c', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'room2c', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'room3c', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'room4c', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & room5c & !room6c & !room7c & !room8c & !room9c': 'room5c', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & room6c & !room7c & !room8c & !room9c': 'room6c', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & room7c & !room8c & !room9c': 'room7c', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & room8c & !room9c': 'room8c', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & room9c': 'room9c', \
        }

    arm_pretty = {  'pickup & !dropoff & !carry & !lever': 'pickup', \
                    '!pickup & dropoff & !carry & !lever': 'dropoff', \
                    '!pickup & !dropoff & carry & !lever': 'carry', \
                    '!pickup & !dropoff & !carry & lever': 'lever', \
                    '!pickup & !dropoff & !carry & !lever': 'arm_idle' , \
                    '!camera': '!camera', \
                    'camera': 'camera', \
                    '!beep': '!beep', \
                    'beep': 'beep', \
                    '!scan': '!scan', \
                    'scan': 'scan', \
                      }
    pretty_dict = {**reg_pretty, **arm_pretty}


    for agent in team_assignment:
        print(agent.id, agent.cost, sync_buchi_path)
        print('total time excluding dfs: ', agent.time)

        # for p in agent.path_sync:
        #     a_state, b_state, pi_sync = p
        #     a_list = []
        #     for a in a_state:
        #         a_list.append(pretty_dict[a])
        #     print(", ".join('"'+x+'"' for x in a_list) + ';')


    print('non sync behavior ------- ')
    for agent in team_assignment:
        print(agent.id)
        for z in range(len(agent.path_nosync)):
            e1, e2, pi_sync = sync_buchi_path[z]
            buchi_formula = buchi.get_edge_data(e1,e2)[pi_sync]['label']
            print('buchi path', sync_buchi_path[z], method2.extract_bindings(buchi_formula))
            for p in agent.path_nosync[z]:
                a_state, b_state, pi_sync = p
                a_list = []
                for a in a_state:
                    if a in pretty_dict:
                        a_list.append(pretty_dict[a])
                    else:
                        a_list.append(a)
                print(", ".join('"'+x+'"' for x in a_list) + ';')

    
    print('\n')

    print('time to do dfs', tf_3-t0_3, sync_buchi_path)
    

    print("total time (with parallel execution): ", max_time + max_time_synth + time_dfs)

    if sims:
        return max_time + max_time_synth, time_dfs
    else:
        return sync_buchi_path, self_transitions, team_assignment






if __name__ == '__main__':
    # G = nx.MultiDiGraph()
    # G.add_edge(1, 2, weight=4.7, sync = 'a')
    # G.add_edge(1, 2, weight=5.8, sync = 'b')

    # print(G.edges(data=True))
    # print(nx.get_edge_attributes(G, 'weight'))
    # sdf
    
    # spec = "GF(room1 & scan).[(1 & 3)] & (!scan.[1] U camera.[(2 | 3)])"
    # ap_list = ['room1', 'scan', 'camera']
    # # dict_form_id, dict_form_aps = method2.create_ap_ids(spec, ap_list)
    # spec_exp = centralized_synth.expand_spec_ap(spec)

    # print(spec_exp)
    # dict_form_id, dict_form_aps = method2.create_ap_ids(spec_exp, ap_list)
    # print('dict', dict_form_id)

    # print(centralized_synth.expand_spec_binding(spec_exp, dict_form_id))


    # asdf

    ################################################
    # spec = 'GF((room1_1 & room1_3) & (scan_1 & scan_3)) & (!scan_1 U (camera_2 | camera_3))'
    # f = spot.translate(spec, 'BA', 'sbacc', 'unambig')
    # file = open("check.dot", "w")
    # file.write(f.to_str('dot'))

    # file.close()

    ################

    filename = 'buchi_all.dot'
    spec = "GF(room1 & pickup).[(1 & 2)] & (!pickup.[1] U (camera.[(2 | 3)] & scan.[2]))"
    spec = 'GF(room1 & pickup).[1] & (!pickup.[1] U (scan & camera).[(2 | 3)])'
    spec = 'GF(room1 & pickup).[1] & (!pickup.[1] U (scan & camera).[(2 | 3)])'         # ex1
    spec = 'F((room9 & camera).[2] & (room1 & pickup).[1]) & (!pickup.[1] U (room1 & scan).[(2 | 3)])'         # ex3
    # spec = 'F((room91 & camera).[2] & (room11 & pickup).[1]) & (!pickup.[1] U (room11 & scan).[(2 | 3)])'         # ex3
    spec_iros = 'F((room9 & beep).[2] & (room1 & pickup).[1]) & (!pickup.[1] U ((room1 & ((scan | camera) & !(scan & camera))).[2] | (room1 & ((scan | camera) & !(scan & camera))).[3]))'  
    # spec_iros = 'F((room9 & beep).[2] & (room1 & pickup).[1]) & (!pickup.[1] U (((room1 & (scan | camera)).[2] | (room1 & (scan | camera)).[3]) & (!scan | !camera).[2] & (!scan | !camera).[3])'  
    # spec_iros = 'F((room9 & beep).[2] & (room1 & pickup).[1]) & (!pickup.[1] U ((room1 & (scan | camera)).[2] | (room1 & (scan | camera)).[3])) & G(scan -> !camera & camera -> !scan).[2] & G(scan -> !camera & camera -> !scan).[3]  '  
    # spec_iros = 'F((room9 & beep).[2] & (room1 & pickup).[1]) & (!pickup.[1] U ((room1 & ((scan | camera) & !(scan & camera))).[2] | (room1 & ((scan | camera) & !(scan & camera))).[3]))'  
    spec_iros = 'F((room9 & beep).[(2 & 3)] & (room1 & pickup).[1]) & (!pickup.[1] U (room1 & ((scan | camera) & !(scan & camera))).[2])'
    # spec_iros = 'F((room9.[3] & beep.[3]) & (room9.[2] & beep.[2]) & (room1 & pickup).[1]) & (!pickup.[1] U (room1 & ((scan | camera) & !(scan & camera))).[2])'

    spec_ordered = 'F((room9 & X lever).[2] & (room1 & X pickup).[1]) & (!pickup.[1] U (room1 & scan).[(2 | 3)])'
    spec_ordered = 'F((room9 & lever).[2] & (room1 & pickup).[1]) & (!pickup.[1] U (room1 & scan).[(2 | 3)])'
    spec_test = 'F(room.[1] & X(pickup.[1] & lever.[2])) & G(pickup.[1] -> lever.[2]) & G(lever.[2] -> pickup.[1])'
    # spec = 'GF(room1.[1] & pickup.[1] & room1.[2] & pickup.[2]) & (!pickup.[1] U (room1.[(2 & 3)] & scan.[2] & camera.[3]))'  # ex4
    # spec = 'F(room9 & camera).[2] & F(room1 & pickup).[1] & G(pickup.[1] => F(room3.[1] & beep.[1]) | F(room3.[2] & beep.[2]))'   # complex ex

    # spec = 'F((room1 & pickup).[1] & beep.[2]) & (!pickup.[1] U (scan & camera).[(2 | 3)]) & G(pickup.[1] => F(room3 & beep).[1 | 2])'     # ex2

    # spec_test = 'F(room1 & pickup).[1] & (!pickup.[1] U (scan & camera).[(2 | 3)]) & G(pickup.[1] => F(room3 & beep).[1 & 2])'
    # buchi_class = BuchiBindings('buchi_test.dot', spec_test)
    # buchi, init_buchi, accepting_states = buchi_class.construct_buchi(individual_buchi=True, sync_file='buchi_test1.dot')
    # sdf

    # Before a robot can pickup a box in room1, a different robot(s) needs to scan and take a picture of the box.

    spec_test = 'F((room1 & X pickup).[1] & (room2 & X lever).[2] )'

    spec_test = 'F((pickup & pantry) & F(dropoff & kitchen)).[1] & F(pour.[2] & mix.[3]) & G(kitchen.[(1 | (2 | 3))] -> kitchen.[4])'
    spec_test = 'F((pickup & pantry).[1] & F((dropoff & kitchen).[1] & F(pour.[2] & (mix.[3]) & (kitchen.[(2 & 3)])))) & G(kitchen.[(1 | (2 | 3))] -> kitchen.[4])'



    spec_iros = 'F((room9 & beep).[2 & 3] & (room1 & pickup).[1]) & (!pickup.[1] U (room1 & ((scan | camera) & !(scan & camera))).[2])'
    # spec_iros = 'F((room9 & beep).[2 & 3 & 14])'
    spec = 'F(room1.[(1 & 2)]) & G(room1.[(1 | 2)] -> (room3 & camera).[3])'

    spec = spec_iros
    t0 = time.time()
    buchi_class = BuchiBindings(filename, spec)
    buchi, init_buchi, accepting_states, all_bindings = buchi_class.construct_buchi(individual_buchi=False, sync_file='buchi_og_iros_sim.dot')
    tf = time.time()

    buchi_time = tf - t0


    gf.buchi_pretty('buchi_og_iros_sim.dot', 'ex_iros_sim')
    
    
    


    agent_list, ap_list = generate_agents(3)
    # agent_list = agent_list[0:4]

    # all_bindings = re.findall('[0-9]+',spec)


    synth_time, dfs_time = main(spec, agent_list, all_bindings, ap_list,buchi_class, save_files = True)

    print(synth_time + dfs_time+ buchi_time)









