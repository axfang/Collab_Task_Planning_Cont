#!/usr/bin/env python

# simulations for work in rejected ICRA '23 paper

# importing sys
import sys
  
# adding Folder_2 to the system path
sys.path.insert(0, '/Users/amy/usr/lib/python3.7/site-packages/')

sys.path.insert(0, '../../Mission_Switching/scripts')
# sys.path.insert(0, '../../Collab_Mission_Switching')
sys.path.insert(0, '../../')

import os
import spot
import graphviz
import networkx as nx
import matplotlib.pyplot as plt
from networkx.drawing.nx_agraph import write_dot
import numpy as np
import re
import networkx as nx
import matplotlib.pyplot as plt
import Mission_Switching.scripts.bool_parsing
import Mission_Switching.scripts.find_optimal_run as fr
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
import product_automata as pa_cont
import random
import statistics
import pandas as pd

import Mission_Switching.scripts.create_agents as af
from Mission_Switching.scripts.agent_module import Agent

import Collab_Mission_Switching.scripts.simulations as sim_prev
import Collab_Mission_Switching.scripts.product_automata as pa_prev
import Collab_Mission_Switching.scripts.synthesize_team_behavior as stb_prev
import intermediate_trace as int_trace


def create_rooms_wh_c_small(weighting_factor):
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

    ap = [l1, l2, l3, l4, l5, l1c, l2c, l3c, l4c, l5c]  # , l10]
    ap0 = [l1, l2, l3, l4, l5]
    apc = [l1c, l2c, l3c, l4c, l5c]
    not_ap0 = [not_l1, not_l2, not_l3, not_l4, not_l5]
    not_apc = [not_l1c, not_l2c, not_l3c, not_l4c, not_l5c]

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
    # s6c = state_dict_c['s6c']
    # s7c = state_dict_c['s7c']
    # s8c = state_dict_c['s8c']
    # s9c = state_dict_c['s9c']

    K.add_node(s1c)
    K.add_node(s2c)
    K.add_node(s3c)
    K.add_node(s4c)
    K.add_node(s5c)
    # K.add_node(s6c)
    # K.add_node(s7c)
    # K.add_node(s8c)
    # K.add_node(s9c)

    K.add_edge(s1c, s1c, weight=0)
    K.add_edge(s2c, s2c, weight=0)
    K.add_edge(s3c, s3c, weight=0)
    K.add_edge(s4c, s4c, weight=0)
    K.add_edge(s5c, s5c, weight=0)
    # K.add_edge(s6c, s6c, weight=0)
    # K.add_edge(s7c, s7c, weight=0)
    # K.add_edge(s8c, s8c, weight=0)
    # K.add_edge(s9c, s9c, weight=0)

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
    # s2c6, K = create_start_state(2, 6, apc, not_apc, not_ap0, state_dict_c, K)
    s3c4, K = create_start_state(3, 4, apc, not_apc, not_ap0, state_dict_c, K)
    s4c5, K = create_start_state(4, 5, apc, not_apc, not_ap0, state_dict_c, K)
    # s4c6, K = create_start_state(4, 6, apc, not_apc, not_ap0, state_dict_c, K)
    # s6c8, K = create_start_state(6, 8, apc, not_apc, not_ap0, state_dict_c, K)
    # s7c8, K = create_start_state(7, 8, apc, not_apc, not_ap0, state_dict_c, K)
    # s8c9, K = create_start_state(8, 9, apc, not_apc, not_ap0, state_dict_c, K)

    s2c1, K = create_start_state(2, 1, apc, not_apc, not_ap0, state_dict_c, K)
    s3c2, K = create_start_state(3, 2, apc, not_apc, not_ap0, state_dict_c, K)
    # s6c2, K = create_start_state(6, 2, apc, not_apc, not_ap0, state_dict_c, K)
    s4c3, K = create_start_state(4, 3, apc, not_apc, not_ap0, state_dict_c, K)
    s5c4, K = create_start_state(5, 4, apc, not_apc, not_ap0, state_dict_c, K)
    # s6c4, K = create_start_state(6, 4, apc, not_apc, not_ap0, state_dict_c, K)
    # s8c7, K = create_start_state(8, 7, apc, not_apc, not_ap0, state_dict_c, K)
    # s8c6, K = create_start_state(8, 6, apc, not_apc, not_ap0, state_dict_c, K)
    # s9c8, K = create_start_state(9, 8, apc, not_apc, not_ap0, state_dict_c, K)

    state_labels = {}
    for state_name in K.nodes():
        state_labels[state_name] = ' & '.join(prop for prop in state_name.split(' & ') if prop[0] != '!')

    return K, ap, state_dict_c


def main_pa_prev(spec, agent_list, bindings_list, buchi_class, save_files=False, sims=True):
    # from Collab_Mission_Switching
    max_time = 0
    max_time_prune = 0
    buchi = buchi_class.graph
    # nx.set_edge_attributes(buchi, pa_prev.powerset(bindings_list), "binding_combos")

    # check if all bindings can be assigned. If not, no need to go through the rest of the computation
    check_possible = set(bindings_list)

    for ag in agent_list:
        ag.time = 0

    for ag in agent_list:
        t0_2 = time.time()
        # prune prod aut based on which bindings the agent can do
        # ag.possible_bindings = pa_prev.remove_unreachable_bindings(ag.prod_aut, (ag.init_state, buchi_class.init_state),
        #                                                    list(buchi.nodes()), bindings_list)

        ag.possible_bindings = pa_prev.powerset(bindings_list)

        # print(len(ag.prod_aut.nodes()),len(ag.prod_aut.edges()))
        tf_2 = time.time()

        # check if all bindings can be assigned. If not, no need to go through the rest of the computation
        # if check_possible:
        #     check_possible = check_possible.difference({item for sublist in ag.possible_bindings for item in sublist})

        # ag.buchi_possible = ag.buchi_sync
        t0_5 = time.time()
        # ag.buchi_possible = buchi
        ag.buchi_possible = pa_prev.update_sync_buchi_with_possible(ag.prod_aut, ag.buchi_sync, ag.possible_bindings)
        tf_5 = time.time()

        if save_files:
            buchi_to_dot(ag.buchi_possible, 'buchi_sync_possible_ordered' + str(ag.id) + str(ag.id) + '.dot')

        # print(ag.buchi_possible.edges(data=True))
        # print(ag.buchi_possible['0']['0'])
        # sdf
        print('possible: ', ag.possible_bindings)
        print('time to prune prod aut for agent', ag.id, tf_2 - t0_2)
        print('time to update buchi for agent', ag.id, tf_5 - t0_5)
        print('time of pruning prod aut + updating buchi w possible', ag.id, tf_5 - t0_2)

        ag.time += tf_2 - t0_2
        ag.time += tf_5 - t0_5

        time_prune = tf_5 - t0_2


        # theoretically, previous calculations were all in parallel. So only include the time that was the longest
        if ag.time > max_time:
            max_time = ag.time
        if time_prune > max_time_prune:
            max_time_prune = time_prune

    # not all bindings can be assigned, so no need to go through the rest of the computation
    # if check_possible:
    #     print('1 no possible team assignment', max_time)
    #
    #     return float('inf'), float('inf')

    t0_3 = time.time()
    sync_buchi_path, self_transitions, team_assignment = stb_prev.assign_bindings2robots(agent_list, bindings_list,
                                                                                    buchi_class.init_state,
                                                                                    buchi)
    tf_3 = time.time()
    time_dfs = tf_3 - t0_3
    print('time to do dfs', tf_3 - t0_3)

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
        print(agent.id, team_assignment[agent])

    print('dfs: ', sync_buchi_path, self_transitions, agent_test)

    # print(sync_buchi_path, self_transitions)

    max_time_synth = 0

    for ag in team_assignment:
        ag.assigned_bindings = team_assignment[ag]
        print('\n ROBOT ', ag.id, '------------------------------------')
        t0_4 = time.time()
        ag.path_nosync, ag.cost = stb_prev.find_shortest_path_robot(ag.prod_aut, buchi, ag.init_state, sync_buchi_path,
                                                               self_transitions, ag.assigned_bindings)
        tf_4 = time.time()

        # print('time to synthesize non sync behavior for agent', ag.id, tf_4-t0_4)
        ag.time += tf_4 - t0_4

        # theoretically, previous calculations were all in parallel. So only include the time that was the longest
        if tf_4 - t0_4 > max_time_synth:
            max_time_synth = tf_4 - t0_4

    for ag in team_assignment:
        print(ag.id, ag.assigned_bindings)
        print(ag.path_nosync)

    print('\n \n final behavior')
    # stb.synthesize_team_behavior(team_assignment, buchi, sync_buchi_path)

    reg_pretty = {'room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room1', \
                  '!room1 & room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room2', \
                  '!room1 & !room2 & room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room3', \
                  '!room1 & !room2 & !room3 & room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room4', \
                  '!room1 & !room2 & !room3 & !room4 & room5 & !room6 & !room7 & !room8 & !room9': 'room5', \
                  '!room1 & !room2 & !room3 & !room4 & !room5 & room6 & !room7 & !room8 & !room9': 'room6', \
                  '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & room7 & !room8 & !room9': 'room7', \
                  '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & room8 & !room9': 'room8', \
                  '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & room9': 'room9', \
 \
                  'rega & !regb & !regc & !regd & !rege': 'regionA', \
                  '!rega & regb & !regc & !regd & !rege': 'regionB', \
                  '!rega & !regb & regc & !regd & !rege': 'regionC', \
                  '!rega & !regb & !regc & regd & !rege': 'regionD', \
                  '!rega & !regb & !regc & !regd & rege': 'regionE', \
 \
                  }
    arm_pretty = {'pickup & !dropoff & !carry & !lever': 'pickup', \
                  '!pickup & dropoff & !carry & !lever': 'dropoff', \
                  '!pickup & !dropoff & carry & !lever': 'carry', \
                  '!pickup & !dropoff & !carry & lever': 'lever', \
                  '!pickup & !dropoff & !carry & !lever': 'arm_idle', \
                  '!camera': '!camera', \
                  'camera': 'camera', \
                  '!beep': '!beep', \
                  'beep': 'beep', \
                  '!scan': '!scan', \
                  'scan': 'scan', \
                  '!light': '!UV', \
                  'light': 'UV', \
 \
                  'pickup & !dropoff & !carry & !lever': 'pickup', \
                  '!pickup & dropoff & !carry & !lever': 'dropoff', \
                  '!pickup & !dropoff & carry & !lever': 'carry', \
                  '!pickup & !dropoff & !carry & lever': 'lever', \
                  '!pickup & !dropoff & !carry & !lever': 'arm_idle', \
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
            buchi_formula = buchi.get_edge_data(e1, e2)[pi_sync]['label']
            print('buchi path', sync_buchi_path[z], method2.extract_bindings(buchi_formula))
            for p in agent.path_nosync[z]:
                a_state, b_state, pi_sync = p
                a_list = []
                for a in a_state:
                    if a in pretty_dict:
                        a_list.append(pretty_dict[a])
                    else:
                        a_list.append(a)
                print(", ".join('"' + x + '"' for x in a_list) + ';')

    print('\n')

    print('time to do dfs', tf_3 - t0_3, sync_buchi_path)

    print("total time (with parallel execution): ", max_time + max_time_synth + time_dfs)

    if sims:
        return max_time + max_time_synth, time_dfs
    else:
        return sync_buchi_path, self_transitions, team_assignment

def main_cont(spec, agent_list, ap_list, bindings_list, buchi_class, non_inst_props):

    buchi = buchi_class.graph
    init_buchi = buchi_class.init_state

    # from intermediate_trace.py

    # ----- from pa.main -----
    for ag in agent_list:
        ag.time = 0
        print(ag.id)

        # prune prod aut based on which bindings the agent can do
        ag.possible_bindings = pa_cont.powerset(bindings_list)
        ag.path_nosync = []


    print('================= intermediate ===============')

    # buchi_path, team_assignment, team_paths = int_trace.find_teaming_assignment(agent_list, all_bindings, buchi, init_buchi)

    t0_dfs = time.time()
    buchi_path, self_transitions, team_assignment, buchi_upd, binding_constraints = int_trace.assign_bindings2robots(
        agent_list,
        bindings_list,
        buchi_class.init_state,
        buchi, non_inst_props)
    tf_dfs = time.time()

    dfs_time = tf_dfs - t0_dfs

    print('time for dfs: ', tf_dfs - t0_dfs)

    print('final ======= ')
    print(buchi_path, self_transitions)
    print('constraints:', binding_constraints)

    return dfs_time, dfs_time


def random_combination(iterable, r):
    '''Random selection from itertools.combinations(iterable, r)
    Returns a random tuple with length r from the iterable'''
    pool = tuple(iterable)
    n = len(pool)
    indices = sorted(random.sample(range(n), r))
    return list(pool[i] for i in indices)



def generate_agents_c(num_agents, sensors, seed_num):
    ''' generate num_agents of robots with randomn capabilities and initial states 
    '''
    random.seed(seed_num)

    agent_list = []
    ap_all = set()

    arm_idx = sensors.index('arm')

    loc, ap_loc, ap_loc_dict = create_rooms_wh_c_small(1)  # keep all environment mobility the same
    loc_c = [l for l in ap_loc if l[-1] == 'c']

    for i in range(1,num_agents+1):

        ap_list = [ap_loc]
        cap_list = [loc]

        init_state = [random.choice(list(ap_loc_dict.values()))]

        if random.randint(1, len(sensors)) == 1:   # randomly decide if agent will have arm.
            sensor_list = list(set(['arm'] + random_combination(sensors, 1)))     # only add one additional sensor
            # sensor_list = ['arm']
        else:
            sensor_on = [0] * len(sensors)

            # Randomly choose the number of elements to set to 1 (between 1 and 2)
            num_ones = random.randint(1, 2)

            # Randomly select indices to set to 1
            indices = random.sample(range(len(sensor_on)), num_ones)

            while arm_idx in indices:
                indices = random.sample(range(len(sensor_on)), num_ones)

            # Set the selected indices to 1
            for index in indices:
                sensor_on[index] = 1

            # while sum(sensor_on) != 3 or sensor_on[arm_idx] == 1:   # ensure agent has a t least one capability but not more than 2
            #     sensor_on = [random.randrange(0, 2, 1) for i in range(len(sensors))]   # randomly choose which sensors the agent has


            sensor_list = sum([[s] * n for s, n in zip(sensors, sensor_on)], [])
        
        for s in sensor_list:
            if s == 'camera':
                weighting_factor = round(random.uniform(0, 10), 3)      # create sensor with random weight b/w 0 and 10
                cap, ap = af.create_camera(weighting_factor)
                cap_list.append(cap)
                ap_list.append(ap)
                init_state.append('!camera')
            elif s == 'scan':
                weighting_factor = round(random.uniform(0, 10), 3)      # create sensor with random weight b/w 0 and 10
                cap, ap = af.create_scan(weighting_factor)
                cap_list.append(cap)
                ap_list.append(ap)
                init_state.append('!scan')
            elif s == 'arm':
                weighting_factor = round(random.uniform(0, 10), 3)      # create sensor with random weight b/w 0 and 10
                cap, ap = pa_cont.create_arm_c(weighting_factor)
                cap_list.append(cap)
                ap_list.append(ap)
                init_state.append('!pickup & !dropoff & !push & !pickupc & !dropoffc & !pushc')
            elif s == 'beep':
                weighting_factor = round(random.uniform(0, 10), 3)      # create sensor with random weight b/w 0 and 10
                cap, ap = pa_cont.create_beep(weighting_factor)
                cap_list.append(cap)
                ap_list.append(ap)
                init_state.append('!beep')


        ag = Agent(ap_list, cap_list, i)
        print(ag.id, sensor_list, ag.graph.number_of_nodes())
        ag.cap_names = sensor_list
        # ag.init_state = random.choice(list(ag.graph.nodes()))
        ag.init_state = tuple(init_state)

        agent_list.append(ag)
        ap_all = ap_all | set(ag.ap_list)

    return agent_list, ap_all
    



def vary_num_agents(num_agents, num_sims, sensors, spec, seed_num):
    # create buchi
    filename = "buchi_sim"

    t0 = time.time()
    buchi_class = BuchiBindings(filename, spec)
    buchi, init_buchi, accepting_states, bindings_list = buchi_class.construct_buchi(individual_buchi=False, sync_file='buchi_sim.dot')
    tf = time.time()

    non_inst_props = ['pushc', 'room4c', 'room7c', 'room5c', 'room8c', 'room6c', 'room1c', 'room3c', 'dropoffc',
                      'room2c', 'room9c', 'pickupc']

    buchi_time = tf - t0

    time_list_discrete = []
    dfs_list_discrete = []
    time_list_cont = []
    dfs_list_cont = []
    sim = 1

    while sim <= num_sims:
    # for sim in range(1, num_sims+1):
        print("==============================================================================================================")
        print('====================== sim #', sim, "for ", num_agents, "agents", seed_num, "===============================")
        agent_list, ap_list = generate_agents_c(num_agents, sensors, seed_num)

        # construct prod_aut
        time_prod = 0
        for ag in agent_list:
            t0_1 = time.time()
            ag.prod_aut, ag.buchi_sync = pa_cont.construct_prod_aut(ag, buchi_class, bindings_list)
            tf_1 = time.time()
            print('time to construct prod aut for agent', ag.id, tf_1 - t0_1)
            if tf_1 - t0_1 > time_prod:
                time_prod = tf_1 - t0_1

        # synthesize behavior for discrete
        synth_time_discrete, dfs_time_discrete = main_pa_prev(spec, agent_list, bindings_list, buchi_class, save_files = False)

        # synthesize behavior for continuous
        synth_time_cont, dfs_time_cont = main_cont(spec, agent_list, ap_list, bindings_list, buchi_class, non_inst_props)

        # TODO: should I include sims where cont has no solution but discrete does?

        # only include if team was valid
        if synth_time_discrete < float('inf') and synth_time_cont < float('inf'):

            total_time_discrete = buchi_time+synth_time_discrete+time_prod
            time_list_discrete.append(synth_time_discrete)
            dfs_list_discrete.append(dfs_time_discrete)

            time_list_cont.append(synth_time_cont)
            dfs_list_cont.append(dfs_time_cont)

            

            dict_token = {'sim': [sim], 'seed':[seed_num], 'num_agents': [num_agents], 'buchi_time': [buchi_time],
                          'prod_aut_time': [time_prod],
                          'dfs_time_discrete':[dfs_time_discrete], 'dfs_time_cont':[dfs_time_cont],
                          'synth_time_discrete': [synth_time_discrete], 'synth_time_cont': [synth_time_cont],
                          'total_time_discrete':[dfs_time_discrete + buchi_time + time_prod],
                          'total_time_cont': [dfs_time_cont + buchi_time + time_prod]}

            if os.path.exists('varyAgentsCont_data.csv'):
                df = pd.DataFrame(dict_token) 
                df.to_csv('varyAgentsCont_data.csv', mode='a', header=False)
            
            else:
                df = pd.DataFrame(dict_token) 
                # saving the dataframe 
                df.to_csv('varyAgentsCont_data.csv')

            sim += 1
        seed_num += 1
        
    time_avg_discrete = statistics.mean(time_list_discrete)
    time_std_discrete = statistics.stdev(time_list_discrete, time_avg_discrete)
    time_avg_dfs_discrete = statistics.mean(dfs_list_discrete)
    time_std_dfs_discrete = statistics.stdev(dfs_list_discrete, time_avg_dfs_discrete)

    time_avg_cont = statistics.mean(time_list_cont)
    time_std_cont = statistics.stdev(time_list_cont, time_avg_cont)
    time_avg_dfs_cont = statistics.mean(dfs_list_cont)
    time_std_dfs_cont = statistics.stdev(dfs_list_cont, time_avg_dfs_cont)


    return [[time_avg_discrete, time_std_discrete], [time_avg_dfs_discrete, time_std_dfs_discrete]], \
        [[time_avg_cont, time_std_cont], [time_avg_dfs_cont, time_std_dfs_cont]], seed_num



def vary_num_bindings(num_bindings, num_sims, sensors, spec, seed_num):
    # create buchi
    filename = "buchi_sim"
    num_agents = 4

    t0 = time.time()
    buchi_class = BuchiBindings(filename, spec)
    buchi, init_buchi, accepting_states, bindings_list = buchi_class.construct_buchi(individual_buchi=False,
                                                                                     sync_file='buchi_sim.dot')
    tf = time.time()

    non_inst_props = ['pushc', 'room4c', 'room7c', 'room5c', 'room8c', 'room6c', 'room1c', 'room3c', 'dropoffc',
                      'room2c', 'room9c', 'pickupc']

    buchi_time = tf - t0

    time_list_discrete = []
    dfs_list_discrete = []
    time_list_cont = []
    dfs_list_cont = []
    sim = 1

    while sim <= num_sims:
    # for sim in range(1, num_sims+1):
        print("==============================================================================================================")
        print('====================== sim #', sim, "for ", num_bindings+2, "bindings", seed_num, "===============================")
    
        agent_list, ap_list = generate_agents_c(num_agents, sensors, seed_num)

        # construct prod_aut
        time_prod = 0
        for ag in agent_list:
            t0_1 = time.time()
            ag.prod_aut, ag.buchi_sync = pa_cont.construct_prod_aut(ag, buchi_class, bindings_list)
            tf_1 = time.time()
            print('time to construct prod aut for agent', ag.id, tf_1 - t0_1)
            if tf_1 - t0_1 > time_prod:
                time_prod = tf_1 - t0_1

        # synthesize behavior for discrete
        synth_time_discrete, dfs_time_discrete = main_pa_prev(spec, agent_list, bindings_list, buchi_class,
                                                              save_files=False)

        # synthesize behavior for continuous
        if synth_time_discrete < float('inf'):
            synth_time_cont, dfs_time_cont = main_cont(spec, agent_list, ap_list, bindings_list, buchi_class, non_inst_props)

        # TODO: should I include sims where cont has no solution but discrete does?

        # only include if team was valid
        if synth_time_discrete < float('inf') and synth_time_cont < float('inf'):

            total_time_discrete = buchi_time + synth_time_discrete + time_prod
            time_list_discrete.append(synth_time_discrete)
            dfs_list_discrete.append(dfs_time_discrete)

            time_list_cont.append(synth_time_cont)
            dfs_list_cont.append(dfs_time_cont)

            dict_token = {'sim': [sim], 'seed': [seed_num], 'num_bindings': [num_bindings + 3], 'buchi_time': [buchi_time],
                          'prod_aut_time': [time_prod],
                          'dfs_time_discrete': [dfs_time_discrete], 'dfs_time_cont': [dfs_time_cont],
                          'synth_time_discrete': [synth_time_discrete], 'synth_time_cont': [synth_time_cont],
                          'total_time_discrete': [dfs_time_discrete + buchi_time + time_prod],
                          'total_time_cont': [dfs_time_cont + buchi_time + time_prod]}

            if os.path.exists('varyBindingsCont_data.csv'):
                df = pd.DataFrame(dict_token) 
                df.to_csv('varyBindingsCont_data.csv', mode='a', header=False)
            
            else:
                df = pd.DataFrame(dict_token) 
                # saving the dataframe 
                df.to_csv('varyBindingsCont_data.csv')

            sim += 1
        seed_num += 1

    time_avg_discrete = statistics.mean(time_list_discrete)
    time_std_discrete = statistics.stdev(time_list_discrete, time_avg_discrete)
    time_avg_dfs_discrete = statistics.mean(dfs_list_discrete)
    time_std_dfs_discrete = statistics.stdev(dfs_list_discrete, time_avg_dfs_discrete)

    time_avg_cont = statistics.mean(time_list_cont)
    time_std_cont = statistics.stdev(time_list_cont, time_avg_cont)
    time_avg_dfs_cont = statistics.mean(dfs_list_cont)
    time_std_dfs_cont = statistics.stdev(dfs_list_cont, time_avg_dfs_cont)

    return [[time_avg_discrete, time_std_discrete], [time_avg_dfs_discrete, time_std_dfs_discrete]], \
        [[time_avg_cont, time_std_cont], [time_avg_dfs_cont, time_std_dfs_cont]], seed_num



def main(varying_agents):
    seed_num = 1
    num_agents = 20
    num_bindings = 10
    num_sims = 30
    sensors = ['arm','scan', 'beep', 'camera']
    spec = 'F((room1 & X pickup).[1] & (room2 & X scan).[2] )'
    spec = 'F((room9 & beep).[2] & (room1 & pickup).[1]) & (!pickup.[1] U ((room1 & ((scan | camera) & !(scan & camera))).[2] | (room1 & ((scan | camera) & !(scan & camera))).[3]))'  
    
    # spec = 'F((room9c & beep).[2 & 3] & (room1c & pickup).[1]) & (!pickup.[1] U (room1c & ((scan | camera) & !(scan & camera))).[2])'
    # spec = "F((pickup & room1c).[1] & F(beep & room9c).[1 | 2])"
    spec = 'F((room1c).[1]) & G(!((!room1c).[1]) -> (room2c & camera).[2 & 3])'
    num_b0 = 3
    
    
    if varying_agents:
        for num_a in range(3,num_agents+1):
            stats_discrete, stats_cont, seed_num = vary_num_agents(num_a, num_sims, sensors, spec, seed_num)

            total_stats_discrete, dfs_stats_discrete = stats_discrete
            total_stats_cont, dfs_stats_cont = stats_cont

            seed_num += 1

            dict_token = {'num_agents': [num_a],
                          'time_total_discrete': [total_stats_discrete[0]], 'time_total_std_discrete': [total_stats_discrete[1]],
                          'time_dfs_discrete': [dfs_stats_discrete[0]], 'time_dfs_std_discrete': [dfs_stats_discrete[1]],
                          'time_total_cont': [total_stats_cont[0]], 'time_total_std_cont': [total_stats_cont[1]],
                          'time_dfs_cont': [dfs_stats_cont[0]], 'time_dfs_std_cont': [dfs_stats_cont[1]],
                          }

            if os.path.exists('varyAgents_avg.csv'):
                df = pd.DataFrame(dict_token) 
                df.to_csv('varyAgents_avg.csv', mode='a', header=False)
            
            else:
                df = pd.DataFrame(dict_token) 
                # saving the dataframe 
                df.to_csv('varyAgents_avg.csv') 
    else:
        
        for num_b in range(num_bindings):
            # generate a specification with more bindings
            binding_form = ''
            for b in range(num_b0+1,num_b+num_b0+1):
                binding_form += ' & ' + str(b)
            # spec_new = 'F((room9 & beep).[2 & 3' + binding_form + '] & (room1 & pickup).[1]) & (!pickup.[1] U (room1 & ((scan | camera) & !(scan & camera))).[2])'
            # spec_new = "F((pickup & room1c).[1" + binding_form + "] & F(beep & room5c).[1 | 2])"
            spec_new = 'F((room1c).[1' + binding_form + ']) & G(!((!room1c).[1]) -> (room2c & camera).[2 & 3])'
            print('spec', spec_new)

            stats_discrete, stats_cont, seed_num = vary_num_bindings(num_b, num_sims, sensors, spec_new, seed_num)

            total_stats_discrete, dfs_stats_discrete = stats_discrete
            total_stats_cont, dfs_stats_cont = stats_cont

            seed_num += 1

            dict_token = {'num_bindings': [num_b + num_b0],
                       'time_total_discrete': [total_stats_discrete[0]],
                       'time_total_std_discrete': [total_stats_discrete[1]],
                       'time_dfs_discrete': [dfs_stats_discrete[0]],
                       'time_dfs_std_discrete': [dfs_stats_discrete[1]],
                       'time_total_cont': [total_stats_cont[0]], 'time_total_std_cont': [total_stats_cont[1]],
                       'time_dfs_cont': [dfs_stats_cont[0]], 'time_dfs_std_cont': [dfs_stats_cont[1]],
                       }

            if os.path.exists('varyBindings_avg.csv'):
                df = pd.DataFrame(dict_token) 
                df.to_csv('varyBindings_avg.csv', mode='a', header=False)
            
            else:
                df = pd.DataFrame(dict_token) 
                # saving the dataframe 
                df.to_csv('varyBindings_avg.csv')

def make_plots(varying_agents, dfs = False, save_fig = ''):
    # from avg csv

    if varying_agents:
        df = pd.read_csv(r'varyAgents_avg.csv')
        num = list(df['num_agents'])
    else:
        df = pd.read_csv(r'varyBindings_avg.csv')
        num = list(df['num_bindings'])

    if not dfs:
        time_mean_discrete = list(df['time_total_discrete'])
        time_mean_cont = list(df['time_total_cont'])
    else:
        time_mean_discrete = list(df['time_dfs_discrete'])
        time_mean_cont = list(df['time_dfs_cont'])

    # dfs_mean = list(df['time_dfs'])
    # dfs_stdev = list(df['time_dfs_std'])

    # total_mean = [time_mean[i] + dfs_mean[i] for i in range(len(time_mean))]
    # total_stdev = [np.sqrt((time_stdev[i]**2 + dfs_stdev[i]**2)) for i in range(len(time_mean))]

    # total_mean = time_mean
    # total_stdev = time_stdev

    err_list_discrete = get_minmax(num, 30, varying_agents, discrete = True, dfs = dfs)
    err_list_cont = get_minmax(num, 30, varying_agents, discrete = False, dfs = dfs)

    MEDIUM_SIZE = 10
    BIGGER_SIZE = 12

    plt.rc('axes', labelsize=BIGGER_SIZE)
    plt.rc('xtick', labelsize=BIGGER_SIZE)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=BIGGER_SIZE)    # fontsize of the tick labels
    plt.rc('legend', fontsize=BIGGER_SIZE)    # legend fontsize
    plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

    fig1 = plt.figure('Computation Time')
    line1, = plt.plot(num, time_mean_discrete, 'b-', markersize=5, markeredgewidth = 1, label = 'discrete')
    line2, = plt.plot(num, time_mean_cont, 'g-', markersize=5, markeredgewidth=1, label='continuous')

    markers1, caps1, bars1 = plt.errorbar(np.array(num), np.array(time_mean_discrete), err_list_discrete, fmt='bo',
                                          markersize=4, ecolor='blue', capsize=2)

    markers2, caps2, bars2 = plt.errorbar(np.array(num), np.array(time_mean_cont), err_list_cont, fmt='go',
                                          markersize=4, ecolor='green', capsize=2)

    [bar.set_alpha(0.5) for bar in bars1 + bars2]
    [cap.set_alpha(0.5) for cap in caps1 + caps2]

    plt.grid(axis='y', linestyle='--')
    ax = fig1.gca()

    ax.legend([line1, line2], ['discrete', 'continuous'])


    ax.set_xticks(np.arange(num[0]-1,num[-1]+1, 1))
    # ax.set_yticks(np.arange(0, 36, 5))
    # plt.xlim([num[0]-1,num[-1]+1])
    # plt.ylim([0.6, 0.5])

    # plt.title('Computation Time')
    if varying_agents:
        plt.xlabel('Number of Robots')
    else:
        plt.xlabel('Number of Bindings')
    plt.ylabel('Computation Time (s)')


    #set aspect ratio
    # ax.set_aspect(abs((18)/(25))*0.15)
    # ax.set_aspect(15)

    if save_fig:
        plt.savefig(save_fig,bbox_inches='tight')
        plt.close()
    else:
        plt.show()

def get_minmax(num_agents, num_sims, varying_agents, discrete = True, dfs = False):

    # from avg csv
   

    if varying_agents:
        df = pd.read_csv(r'varyAgentsCont_data.csv')
        num = list(df['num_agents'])
    else:
        df = pd.read_csv(r'varyBindingsCont_data.csv')
        num = list(df['num_bindings'])

    if discrete and not dfs:
        time_total = list(df['synth_time_discrete'])
    elif not discrete and not dfs:
        time_total = list(df['synth_time_cont'])
    elif discrete and dfs:
        time_total = list(df['dfs_time_discrete'])
    elif not discrete and dfs:
        time_total = list(df['dfs_time_cont'])

    err_list = []

    idx = 0
    for i in range(len(num_agents)):
        sims_data = time_total[idx:idx+num_sims]
        # min_list.append(min(sims_data))
        # max_list.append(max(sims_data)) 
        err_list.append([np.mean(sims_data) - min(sims_data), max(sims_data) - np.mean(sims_data) ])
        print(np.mean(sims_data))

        idx += num_sims
    err_list = np.transpose(err_list)
    return err_list


if __name__ == '__main__':
    varying_agents = False
    main(True)
    # main(False)
    make_plots(False, save_fig = 'vary_bindings_plot1.png')
    make_plots(True, dfs = False, save_fig = 'vary_agents_plot1.png')









