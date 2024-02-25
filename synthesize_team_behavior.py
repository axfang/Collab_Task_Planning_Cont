#!/usr/bin/env python

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
import product_automata as pa
import random


def update_sync_buchi_with_assign(prod_aut, buchi_sync_possible, assigned_bindings):
    ''' given assignment of bindings, remove unreachable nodes/edges
    '''
    buchi_sync = buchi_sync_possible.copy()
    buchi_sync.remove_edges_from(list(buchi_sync.edges()))
    sync_transitions={}

    for buchi_state in buchi_sync_possible.nodes():
        buchi_neighbors = list(buchi_sync_possible.successors(buchi_state))

        for buchi_neighbor in buchi_neighbors:
            
            # for each edge from (buchi_state, buchi_neighbor)
            for edge_idx in buchi_sync_possible.get_edge_data(buchi_state,buchi_neighbor).keys():     
                buchi_formula, pi_sync, possible_combos = buchi_sync_possible.get_edge_data(buchi_state,buchi_neighbor)[edge_idx]['label']
                buchi_bindings = method2.extract_bindings(buchi_formula) 
                buchi_bindings = [int(binding) for binding in buchi_bindings]

                '''
                # find prod_states in which 
                - transition to buchi_neighbor on the pi_sync edge
                - the bindings for that state can be satisfied by the assigned agent bindings OR do not matter to the assigned bindings
                '''
                prod_states = [n for n,v in prod_aut.nodes(data=True) if v['buchi_state'] == buchi_neighbor and v['sync'] == pi_sync and (set(assigned_bindings).issubset(v['bindings']) or set(assigned_bindings).isdisjoint(set(buchi_bindings)))] 
                # print(buchi_neighbor, pi_sync, prod_states )
                if len(prod_states) != 0:
                    sync_transitions[(buchi_state, buchi_neighbor, pi_sync)] = (buchi_formula, possible_combos)
                # for state in prod_states:
                #     state_bindings = nx.get_node_attributes(prod_aut, "bindings")[state]
                #     # print(state_bindings, buchi_bindings)
                #     if not set(assigned_bindings).isdisjoint(set(buchi_bindings)):
                #         sync_transitions[(buchi_state, buchi_neighbor, pi_sync)] = buchi_formula

    for d,v in sync_transitions.items():
        buchi_sync.add_edge(d[0], d[1], label=(v[0], d[2]), sync=d[2], key=d[2])

    # print('buchi_sync edges',buchi_sync.edges(data=True))
    # aaaaaaaa

    return buchi_sync


def construct_team_buchi(agent_list, buchi_og, possible=True):
    ''' 
    merge each agent's sync buchi by only keeping states/transitions that exist across all of the agents. Also keep track of all bindings that are assigned for each edge
    '''

    # agent_list = list(team_assignment.keys())
    sync_buchi_team = agent_list[0].buchi_possible

    counter = 0
    for ag in agent_list:
        sync_buchi_team_copy = sync_buchi_team.copy()
        if counter == 0:
            if possible:
                sync_buchi_team = ag.buchi_possible
            else:
                sync_buchi_team = ag.buchi_assign
        else:  
            if possible:
                buchi_sync_next = ag.buchi_possible
            else:
                buchi_sync_next = ag.buchi_assign
            

            # find any edges in team buchi that don't exist in agent's buchi
            # ('0', '1', {'label': ('(camera.[2] & pickup.[1] & room1.[1] & scan.[2])', 'pi_a'), 'sync': 'pi_a'})
            data_i = buchi_sync_next.edges(data=True)

            for edge in sync_buchi_team_copy.edges(data=True):
                e1, e2, attributes = edge
                pi_sync = attributes['sync']
                if not buchi_sync_next.has_edge(e1, e2, key = pi_sync):
                # if edge not in data_i:
                    # e1, e2, attributes = edge
                    sync_buchi_team.remove_edge(e1, e2, key = pi_sync)

                    # TODO: UPDATE ROBOT BINDING COMBOS
                else:
                    # keep track of if all the possible bindings the team of robots for that edge

                    if counter == 1:
                        combos_team = sync_buchi_team[e1][e2][pi_sync]['binding_combos'] 
                        bindings_team = set(itertools.chain.from_iterable(combos_team))
                        sync_buchi_team[e1][e2][pi_sync]['binding_combos'] = bindings_team



                     # get all bindings the agent can do 
                    combos_i = buchi_sync_next[e1][e2][pi_sync]['binding_combos']
                    bindings_i = set(itertools.chain.from_iterable(combos_i))

                    bindings_team = sync_buchi_team[e1][e2][pi_sync]['binding_combos'] | bindings_i

                    # print(1, sync_buchi_team[e1][e2])
                    # print(2, buchi_og[e1][e2][pi_sync])
                    sync_buchi_team[e1][e2][pi_sync]['binding_combos'] =  bindings_team
                    sync_buchi_team[e1][e2][pi_sync]['label'] =  (buchi_og[e1][e2][pi_sync]['label'], pi_sync, bindings_team)

        counter +=1

    return sync_buchi_team

#  ~/.local/bin/kernprof -l -v my_script.py
# @profile
def find_shortest_path_sync_binding2(aut, buchi_sync_og, agent_start, buchi_start, init_sync, pi_sync, final_buchi, self_transitions, binding_list, init = False):
    ''' find shortest path through prod_aut given the sync path and the bindings required
    '''

    self_sync = self_transitions[buchi_start]
    final_states = [n for n,v in aut.nodes(data=True) if n[1] == final_buchi or n[1] == pi_sync] 


    # if the very initial state in path
    if init: 
        init_state = (agent_start, buchi_start)
        # final_states.remove(init_state)
    else:
        init_state = (agent_start, buchi_start)   # assuming Buchi always starts at 0
    shortest_dist = float('inf')
    shortest_path  = []


    # nodes = (
    # node
    #     for node, data
    #     in aut.nodes(data=True)
    #        #set(binding_list).issubset(data.get("bindings"))
    #     if node == init_state or (set(binding_list).issubset(data["bindings"]) and ((data["sync"] == self_sync and data['buchi_state'] == buchi_start) or (data["sync"] == pi_sync and data.get("buchi_state") == final_buchi)))
    #     # if (data.get("sync") in [self_sync, pi_sync] and set(binding_list).issubset(data.get("bindings")) and data.get("buchi_state") in [buchi_start,final_buchi]) or node == init_state 
    # )


    # subgraph = aut.subgraph(nodes)
    subgraph = nx.MultiDiGraph()



    for edge in aut.edges(data=True):
        e1, e2, data = edge

        buchi_bindings = find_buchi_formulas(buchi_sync_og, e1[1], e2[1], data['sync'])

    #     if (e1 == init_state and ((set(binding_list).issubset(data["bindings"]) or set(binding_list).isdisjoint(buchi_bindings))) and ((data["sync"] == self_sync and e2[1] == buchi_start) or (data["sync"] == pi_sync and e2[1] == final_buchi and e1[1] != e2[1]))) or \
    #         ((set(binding_list).issubset(data["bindings"]) or set(binding_list).isdisjoint(buchi_bindings)) and \
    #                     ((data["sync"] == self_sync and e2[1] == buchi_start) or (data["sync"] == pi_sync and e2[1] == final_buchi and e1[1] != e2[1]))):

    #         # [(('!room1 & !room2 & room3 & !room4 & !room5', '!camera', '!beep'), '0', 'pi_d'), (('!room1 & !room2 & room3 & !room4 & !room5', '!camera', 'beep'), '0', 'pi_d')]
    #         # if e1 == (('!room1 & !room2 & room3 & !room4 & !room5', '!camera', '!beep', '!scan'), '0') and e2 == (('!room1 & !room2 & room3 & !room4 & !room5', '!camera', '!beep'), '0') and data['sync'] == 'pi_d':

    #         #     print('added')
            
    #         G.add_edge(e1,e2,key=data['sync'])


    # subgraph = aut.edge_subgraph(list(G.edges(keys=True)))


        if ((e1 == init_state and (set(binding_list).issubset(data["bindings"]) or set(binding_list).isdisjoint(buchi_bindings)) and ((data["sync"] == self_sync and e2[1] == buchi_start) or (data["sync"] == pi_sync and e2[1] == final_buchi and e1[1] != e2[1]))) or \
            (set(binding_list).issubset(data["bindings"]) or set(binding_list).isdisjoint(buchi_bindings)) and \
            ((data["sync"] == self_sync and e2[1] == buchi_start) or (data["sync"] == pi_sync and e2[1] == final_buchi and e1[1] != e2[1]))):
                subgraph.add_edge(e1, e2, key=data['sync'], weight=data['sync'])

    subgraph.edge_subgraph(list(subgraph.edges(keys=True)))

    # for final in final_states:
    #     # if nx.has_path(aut, init_state , final):
    #         # if there's KeyError, means there is no path to the end state
    #         # t = time.time()

    #         # check if path can be found to this accepting state
    #         if final in subgraph.nodes():
    #             if nx.has_path(subgraph, init_state , final):
    #                 dist, path = nx.single_source_dijkstra(subgraph, init_state , final, weight='weight')
    #                 # print('path found', final, dist)
    #                 # print('time for one out of ', test, ' iteration of djikstra: ', time.time()-t)
    #                 if dist < shortest_dist:
    #                     shortest_path = path
    #                     shortest_dist = dist
            


    def shortest_path_to_targets(G, start, targets):
        """
        Finds the shortest path in a MultiDiGraph given a list of target nodes.
        
        :param G: The MultiDiGraph to search for the path in.
        :param start: The starting node of the path.
        :param targets: The list of target nodes.
        :return: A tuple of list of nodes representing the shortest path and the target node.
        """
        targets = set(targets)
        dist, pred = nx.single_source_dijkstra(G, start, weight='weight')
        closest_target = None
        closest_dist = float('inf')
        for node, node_dist in dist.items():
            if node in targets and node_dist < closest_dist:
                closest_target = node
                closest_dist = node_dist
        if closest_target is None:
            return None, None
        path = [closest_target]
        while path[-1] != start:
            node = path[-1]
            for pred_node in pred[node]:
                path.append(pred_node)
                break
        return path[::-1], closest_dist

    shortest_path, shortest_dist = shortest_path_to_targets(subgraph, init_state, final_states)

    # add pi_sync into path to make it easier to read
    for p in range(len(shortest_path)):
        if p == len(shortest_path)-1:
            shortest_path[p] += (pi_sync,)
        else:
            shortest_path[p] += (self_sync,)

    return shortest_path, shortest_dist

from queue import PriorityQueue
def find_shortest_path_sync_binding(prod_aut, buchi_sync_og, agent_start, buchi_start, init_sync, pi_sync, final_buchi, self_transitions, binding_list, init = False):
    ''' find shortest path through prod_aut given the sync path and the bindings required
    '''

    if '*' in buchi_start:
        self_sync = 'pi_1'
    else:
        self_sync = self_transitions[buchi_start]
    final_states = [n for n,v in prod_aut.nodes(data=True) if n[1] == final_buchi or n[1] == pi_sync] 


    # if the very initial state in path
    if init: 
        init_state = (agent_start, buchi_start)
        # final_states.remove(init_state)
    else:
        init_state = (agent_start, buchi_start)
    shortest_dist = float('inf')
    shortest_path  = []

    G = nx.MultiDiGraph()

    binding_list_set = set(binding_list)

    prod_formula_dict =nx.get_edge_attributes(prod_aut,'formula')
    prod_edge_dict = nx.get_edge_attributes(prod_aut,'buchi_transition')
    prod_bindings_sync = nx.get_edge_attributes(prod_aut,'sync')
    prod_bindings_dict = nx.get_edge_attributes(prod_aut,'bindings')

    # buchi_bindings_dict = nx.get_edge_attributes(buchi_sync_og,'bindings')

    # condition 1
    # ((set(binding_list).issubset(data["bindings"]) or set(binding_list).isdisjoint(buchi_bindings))
    keys1 =  set(k for k, v in prod_bindings_dict.items() if (binding_list_set.issubset(v) or binding_list_set.isdisjoint(find_buchi_formulas(buchi_sync_og, k[0][1], k[1][1], prod_bindings_sync[k]))))

    # AND condition 2
    # ((data["sync"] == self_sync and e2[1] == buchi_start) or (data["sync"] == pi_sync and e2[1] == final_buchi and e1[1] != e2[1])))
    keys2 =  set(k for k, v in prod_bindings_sync.items() if (v == self_sync and k[1][1] == buchi_start and k[0][1]==buchi_start) or (v == pi_sync and k[1][1] == final_buchi and k[0][1]==buchi_start))
    edges2check = list(keys1 & keys2)

    # find_buchi_formulas(buchi_sync_og, k[0][1], k[1][1], prod_bindings_sync[k])
    for e in edges2check:
        e1,e2,k = e
        G.add_edge(e1,e2,key=prod_bindings_sync[e])

    # aut = prod_aut.copy()
    # for edge in aut.edges(data=True):
    #     e1, e2, data = edge

    #     buchi_bindings = find_buchi_formulas(buchi_sync_og, e1[1], e2[1], data['sync'])
    #     # if (e1 == init_state and ((set(binding_list).issubset(data["bindings"]) or set(binding_list).isdisjoint(buchi_bindings))) and ((data["sync"] == self_sync and e2[1] == buchi_start) or (data["sync"] == pi_sync and e2[1] == final_buchi and e1[1] != e2[1]))) or \
    #     if    ((set(binding_list).issubset(data["bindings"]) or set(binding_list).isdisjoint(buchi_bindings)) and \
    #                     ((data["sync"] == self_sync and e2[1] == buchi_start) or (data["sync"] == pi_sync and e2[1] == final_buchi and e1[1] != e2[1]))):

    #         # [(('!room1 & !room2 & room3 & !room4 & !room5', '!camera', '!beep'), '0', 'pi_d'), (('!room1 & !room2 & room3 & !room4 & !room5', '!camera', 'beep'), '0', 'pi_d')]
    #         # if e1 == (('!room1 & !room2 & room3 & !room4 & !room5', '!camera', '!beep', '!scan'), '0') and e2 == (('!room1 & !room2 & room3 & !room4 & !room5', '!camera', '!beep'), '0') and data['sync'] == 'pi_d':

    #         #     print('added')
            
    #         G.add_edge(e1,e2,key=data['sync'])


    subgraph = prod_aut.edge_subgraph(list(G.edges(keys=True)))
    
    # subgraph.edge_subgraph(list(G.edges(keys=True)))

    # final_states = [s for s in final_states if s in subgraph.nodes()]

    # try:
    # paths = nx.single_source_dijkstra_path(subgraph, init_state, weight='weight')
    # # print([(path[final], final) for final in final_states if final in path])
    # # all_paths = [(final, path[final]) for final in final_states if final in path]
    # # all_paths.sort(key=lambda x: x[1])
    # # shortest_path, min_final = min(all_paths)
    # # min_state = all_paths.index(shortest_path)
    # shortest_path, min_final = min([(paths[final], nx.path_weight(subgraph, paths[final], weight="weight")) for final in final_states if final in paths], key=lambda x: x[1])

    # print('len', len(subgraph.edges()), init_state)
    if init_state not in subgraph:
        return [], float('inf')
    lengths, paths= nx.single_source_dijkstra(subgraph, init_state, weight='weight')

    # if state is z_int1 and there's no path, then the path is just [init_state]
    if lengths == {init_state:0} and len(re.findall('[^0-9]', init_state[1])):
        shortest_path = [init_state]
        shortest_dist = 0
    else:
        all_paths = [(paths[final], lengths[final]) for final in final_states if final in paths]
        if not all_paths:
            return [], float('inf')
        else:
            shortest_path, shortest_dist = min(all_paths, key=lambda x: x[1])


    # shortest_dist = nx.shortest_path_length(subgraph, init_state, shortest_path, weight='weight')
    
    # return shortest_dist, shortest_path
    # except Exception as e:
    #     print(e)
    #     return [], float('inf')

    # # add pi_sync into path to make it easier to read
    # for p in range(len(shortest_path)):
    #     if p == len(shortest_path)-1:
    #         shortest_path[p] += (pi_sync,)
    #     else:
    #         shortest_path[p] += (self_sync,)

    # return shortest_path, shortest_dist


    # for final in final_states:
    #     # if nx.has_path(aut, init_state , final):
    #         # if there's KeyError, means there is no path to the end state
    #         # t = time.time()

    #         # check if path can be found to this accepting state
    #         try:
    #             dist, path = nx.single_source_dijkstra(subgraph, init_state , final, weight='weight')
    #             # print('path found', final, dist)
    #             # print('time for one out of ', test, ' iteration of djikstra: ', time.time()-t)
    #             if dist < shortest_dist:
    #                 shortest_path = path
    #                 shortest_dist = dist
    #         except Exception as e:
    #             # try the next accepting state
    #             # print(final, e)
    #             pass

    # def shortest_path_to_targets(graph, start, targets):
    #     distances = {node: float('inf') for node in graph.nodes}
    #     distances[start] = 0
    #     pq = PriorityQueue()
    #     pq.put((0, start))
    #     parent = {start: None}
    #     target_reached = False
        
    #     while not pq.empty() and not target_reached:
    #         current_distance, current_node = pq.get()
            
    #         if current_node in targets:
    #             target_reached = True
    #             break
            
    #         for neighbor, edge_data in graph[current_node].items():
    #             for edge in edge_data.values():
    #                 print(edge)
    #                 weight = edge['weight']
    #                 new_distance = current_distance + weight
    #                 if new_distance < distances[neighbor]:
    #                     distances[neighbor] = new_distance
    #                     parent[neighbor] = current_node
    #                     pq.put((new_distance, neighbor))
        
    #     if target_reached:
    #         path = []
    #         node = current_node
    #         while node is not None:
    #             path.append(node)
    #             node = parent[node]
    #         path.reverse()
    #         return path, distances[current_node]
    #     else:
    #         return None, float('inf')

    # shortest_path, shortest_dist = shortest_path_to_targets(subgraph, init_state, final_states)
    # add pi_sync into path to make it easier to read
    for p in range(len(shortest_path)):
        if p == len(shortest_path)-1:
            shortest_path[p] += (pi_sync,)
        else:
            shortest_path[p] += (self_sync,)

    return shortest_path, shortest_dist


def find_shortest_path_robot(prod_aut, buchi_sync_og, agent_start, sync_buchi_path, self_transitions, binding_list):
    ''' given path through sync buchi ([[prefix],[suffix]]) and binding assignments, find a minimum cost path to each buchi node and corresponding waiting state/cycle
    '''
    total_dist = 0
    prefix_buchi = sync_buchi_path
    # print('buchi path', sync_buchi_path)
    # print(prefix_buchi)
    # print('prefix buchi', prefix_buchi)
    # print('prefix', prod_aut.nodes())

    path_agent =[]
    counter = 0

    pi_sync = " "
    for buchi_start, buchi_next, pi_sync_next in prefix_buchi:
        if counter == 0:
            init = True
        else:
            init = False
        # print('start','a', agent_start,'b', buchi_start, 'c', buchi_next, 'd', pi_sync)
        # next_path_prefix, dist_prefix = find_path_attribute(prod_aut, agent_start, buchi_start, pi_sync, "sync", [pi_sync] , final_state = buchi_next)
        next_path_prefix, dist_prefix = find_shortest_path_sync_binding(prod_aut, buchi_sync_og, agent_start, buchi_start, pi_sync, pi_sync_next, buchi_next, self_transitions, binding_list, init=init)
        # not a valid path
        if dist_prefix == float('inf'):
            return [], float('inf')
        # print('next_path_prefix', next_path_prefix, binding_list, '\n')
        # print('dist_prefix', dist_prefix)


        # 8/16 - if i want the suffix to work, i need pi_sync to increment to the next one
        # next_path_suffix, dist_suffix= find_path_attribute(prod_aut, agent_start, buchi_next,  pi_sync, "sync", [pi_sync] , final_state = buchi_next)
        # next_path_suffix, dist_suffix = find_shortest_path_sync_binding(prod_aut, agent_start, buchi_next, pi_sync_next, pi_sync_next, buchi_next, self_transitions, binding_list)
        # print('next_path_suffix', next_path_suffix, '\n')

        # path_agent.append((next_path_prefix, next_path_suffix))
        path_agent += [next_path_prefix[1:]]
        total_dist += dist_prefix
        if len(next_path_prefix) != 0:
            agent_start = next_path_prefix[-1][0]

        # print('agent_start', agent_start)
        pi_sync = pi_sync_next

        counter += 1

    
    return path_agent, total_dist


def lcm(denominators):
    val=1
    for i in denominators:
        val = val*i//math.gcd(val, i)
    return val





def find_path_buchi(sync_buchi, init_buchi, show_bindings = False):
    '''
    find possible path in sync buchi automaton to accepting state/accepting cycle

    THIS DOESNT CHECK FOR CONSISTENCY WITH ROBOTS

    output: [[prefix], [suffix]] where each element within prefix/suffix is (state1, state2, pi_sync)
    '''

    final_states = nx.get_node_attributes(sync_buchi, 'accepting')
    edge_labels = nx.get_edge_attributes(sync_buchi, 'sync')

    buchi_data = sync_buchi.edges(data=True)

    prefix = []
    suffix = []

    for final in final_states:
        prefix_path = []
            # if nx.has_path(aut, init_state , final):
                # if there's KeyError, means there is no path to the end state
                # t = time.time()

                # check if path can be found to this accepting state
        try:
            prefix_path = nx.shortest_path(sync_buchi, init_buchi, final)
            suffix_path = nx.shortest_path(sync_buchi, final, final)

            if len(suffix_path) == 1:
                suffix_path = [suffix_path[0], suffix_path[0]]

            # print('time for one out of ', test, ' iteration of djikstra: ', time.time()-t)
            break

        except Exception as e:
            # try the next accepting state
            print(e)
            pass

    if len(prefix_path) == 0:
        return prefix_path, {}

    prefixGraph = nx.path_graph(prefix_path)

    suffixGraph = nx.path_graph(suffix_path)


    # Read attributes from each edge
    for e in prefixGraph.edges():
        edge_list = [item for item in buchi_data if int(item[0]) == int(e[0]) and int(item[1]) == int(e[1])]
        
        pi_sync = edge_list[0][2]['sync']                       # the first element (0) means we're picking a specific edge with label 0
        # pi_sync = sync_buchi.edges[e[0], e[1]]['sync']     # the third element (0) means we're picking a specific edge with label 0

        # if show_bindings:
        #     buchi_formula = edge_list[0][2]['label'][0]
        #     buchi_bindings = method2.extract_bindings(buchi_formula) 
        #     buchi_bindings = [int(binding) for binding in buchi_bindings]
        #     prefix.append((e[0],e[1],pi_sync, buchi_bindings))
        # else:
        prefix.append((e[0],e[1],pi_sync))

    # for e in suffixGraph.edges():
    #     edge_list = [item for item in buchi_data if int(item[0]) == int(e[0]) and int(item[1]) == int(e[1])]
    #     pi_sync = edge_list[0][2]['sync']                   # the first element (0) means we're picking a specific edge with label 0
    #     # pi_sync = sync_buchi.edges[e[0], e[1]]['sync']
    #     suffix.append((e[0],e[1],pi_sync))


    # pick a self-transition edge in sync_buchi
    self_transitions = {}
    for step in prefix+suffix:
        state = step[0]
        edge_list = [item for item in buchi_data if int(item[0]) == int(state) and int(item[1]) == int(state)]
        self_transitions[state] = edge_list[0][2]['sync']



    return prefix, self_transitions

def find_teaming_assignment2(agent_list, binding_list, buchi_sync):
    ''' given a sync buchi with a valid path (and given pi_sync edges) through it, return a subset of robots and their respective assignments

    token-based approach

    we already know that all agents in agent_list contain these edges, so no need to check

    '''
    count = 0
    assign_dict = {}    # {agent: [bindings]}
    token_dict = {}     # {binding: agent}
    token_vec = [0]*len(binding_list)
    for b in binding_list:
        token_dict[b] = 0

    for agent in agent_list:
        agent.cost_dict = {}
        
    while 0 in token_dict.values() and count < len(agent_list):
        for agent in agent_list:
            count += 1

            # binding combinations that the robot can do
            possible_combos = agent.possible_bindings

            # bindings that have already been assigned
            assigned_bindings = [i for i in token_dict if token_dict[i] != 0]

            # bindings that have NOT been assigned
            unassigned_bindings = [i for i in token_dict if token_dict[i] == 0]

            # store any binding combo the agent can do that contains unassigned bindings
            # binding_combos_compare = [i for i in possible_combos if not set(i).isdisjoint(set(unassigned_bindings))]
            
            # FOR NOW, only find combos where there's no overlap (will need to think about how to make comparisons with overlap)
            binding_combos_compare = sorted([i for i in possible_combos if set(i).issubset(set(unassigned_bindings))], key = len, reverse=True)

            # if all combos the robot can do has already been assigned
            if len(binding_combos_compare) == 0:
                continue
            combo2assign = binding_combos_compare[0]

            for b in combo2assign:
                token_dict[b] = agent.id

                if agent in assign_dict.keys():
                    assign_dict[agent].append(b)
                else:
                    assign_dict[agent] = [b]

    print('token_dict', token_dict)
    print('assign_dict', assign_dict)
    return assign_dict


def update_buchi_team(agent_bindings, edge, buchi, buchi_path, buchi_init, binding_constraints = []):
    '''
    add (or remove) edge from team buchi and update team of robots that can do it
    agent_bindings = {agent: set(binding_combos)}
    binding_constraints = [tuple] binding combo that if all be must assigned to the same agent

    edge = (n1, n2, data)
    '''
    agent_bindings_upd = agent_bindings
    # binding_constraints = {tuple(binding_constraints)}
    dict_items = agent_bindings.items()

    e1, e2, pi_sync = edge

    # pi_sync = attributes['sync']
    for agent, bindings in dict_items:
        

        # if agent cannot synthesize behavior from init to e2
        possible_binding_combos = pa.remove_unreachable_bindings(agent.prod_aut, buchi, (agent.init_state, buchi_init),
                                                                 buchi_path, list(bindings),
                                                                 final_state = buchi_path[-1], combos=True)
        # possible_binding_combos = pa.remove_unreachable_bindings(agent.prod_aut, (agent.init_state, buchi_init),
        #                                                          buchi, buchi_path, list(agent_bindings[agent]),
        #                                                          final_state=None, combos=True)

        # if binding_constraints, then remove all binding combos that are a subset of it
        # do not need to do if possible_binding_combos == binding_constraints?
        if binding_constraints and binding_constraints != possible_binding_combos and binding_constraints != {()}:
            combos2remove = pa.powerset(binding_constraints) - binding_constraints
            possible_binding_combos = possible_binding_combos - combos2remove

        if not possible_binding_combos:
            # if not agent.buchi_possible.has_edge(e1, e2, key = pi_sync):
            agent_bindings_upd.pop(agent)
        else:
            # attributes = agent.buchi_possible[e1][e2][pi_sync]
            # keep the agent, but update the possible binding combos
            # binding_combos_new = bindings.intersection(set(attributes['binding_combos']))

            binding_combos_new = bindings.intersection(possible_binding_combos)

            # if no possible binding_combos, remove agent
            if not binding_combos_new:
                agent_bindings_upd.pop(agent)
            else:
                agent_bindings_upd[agent] = binding_combos_new
    return agent_bindings_upd


def add_to_team(all_agent_bindings, current_team, binding_list, edge, visited_teams):
    '''
    add (or remove) edge from team buchi and update team of robots that can do it
    agent_bindings = {agent: set(binding_combos)}

    edge = (n1, n2, data)
    '''
    possible_teams = []
    agent_bindings_upd = current_team.copy()

    e1, e2, pi_sync = edge

    counter = 0
    # all_combos = set()
    for a, bindings in current_team:
        all_combos.union(bindings)

    found = (set(itertools.chain.from_iterable(all_combos)) == set(binding_list))

    # keep adding agents to team until all bindings have been assigned
    while counter < len(all_agent_bindings):
        all_combos_copy = all_combos.copy()
        
        agent = all_agent_bindings[counter]

        if agent not in current_team and agent.buchi_possible.has_edge(e1, e2, key = pi_sync):
            
            attributes = agent.buchi_possible[e1][e2][pi_sync]
            combos_a = attributes['binding_combos']

            current_bindings = set(itertools.chain.from_iterable(all_combos))
            # pick the combo with the most number of bindings that doesn't exist in the current team
            binding_combo = sorted(combos_a, key=lambda x: len(set(x)-set(x).intersection(current_bindings)), reverse=True)[0]

            # if binding_combo does not have any new bindings to contribute, don't add it to the team
            if not set(binding_combo).issubset(current_bindings):
                agent_bindings_upd[agent] = binding_combo

            all_combos.union(binding_combo)

        # check if all bindings have been assigned
        found = (set(itertools.chain.from_iterable(all_combos)) == set(binding_list))

        counter += 1

    if found:
        return

    return possible_teams


                          
def find_teaming_assignment_small(agent_list, binding_list, buchi_team, buchi_sync_og, init_buchi):
    ''' DFS while keeping track of each robots path to through the edges

    Assuming we start with no robots and add robots
    '''
    buchi_path, self_transitions = find_path_buchi(buchi_team, init_buchi)              
    final_states = nx.get_node_attributes(buchi_team, 'accepting')

    all_agent_bindings = {}         # {agent: set(binding_combos)}
    for agent in agent_list:
        all_agent_bindings[agent] = agent.possible_bindings

    # print(list(buchi_sync_og.out_edges(keys=True)))
    init_edge = list(buchi_sync_og.out_edges(init_buchi, keys=True))[0] # pick a random edge   (n1, n2, pi_sync)
    init_edge = (init_buchi, init_buchi, 'pi_a')
    # print(init_edge)
    # sdff
    init_agent = agent_list[0]
    agent_bindings = {init_agent: init_agent.possible_bindings }

    stack = [(init_buchi, init_edge, agent_bindings, [init_edge])]
    valid_nodes = set()
    visited_edges = set()
    visited_teams = set()

    while len(stack)!= 0:
        # pick the smallest team
        (vertex, edge, agent_bindings, path) = sorted(stack, key=lambda x: len(x[2]), reverse=True)[0]
        stack.remove((vertex, edge, agent_bindings, path))        
        
        if edge not in visited_edges:
            visited_edges.add(edge)

            # given current team, find all possible teams 
            possible_teams = add_to_team(all_agent_bindings, agent_bindings, binding_list, edge)
            
            # check if any robots in current team cannot do this edge. If yes, 

            # remove robots from team that cannot do this edge 
            agent_bindings_upd = update_buchi_team(agent_bindings, edge, True)

            # eliminate agents in path that cannot do current path

            # get all bindings the agent can do for the current PATH
            team_bindings= set()
            combos_upd_dict = {}
            all_combos = set()

            for agent in agent_bindings_upd.keys():
                combos_upd = agent_bindings_upd[agent]

                # check new edge has valid binding combos
                e1, e2, pi_sync = edge
                try:
                    combos_i = agent.buchi_possible[e1][e2][pi_sync]['binding_combos']

                except Exception as e:
                        # print(e)

                        combos_upd_dict.pop(agent, None)
                combos_upd = combos_upd.intersection(combos_i)


                # for e in path: 
                #     e1, e2, pi_sync = e
                #     try:
                #         combos_i = agent.buchi_possible[e1][e2][pi_sync]['binding_combos']
                #         # if edge == ('0', '2', 'pi_b'):
                #         #     print('i', combos_i)
                #     except Exception as e:
                #         # print(e)

                #         combos_upd_dict.pop(agent, None)

                #     combos_upd = combos_upd.intersection(combos_i)

                # if agent can't do any bindings
                if len(combos_upd) == 0:
                    combos_upd_dict.pop(agent, None)
                else:
                    all_combos = all_combos.union(combos_upd)
                    # if edge == ('0', '2', 'pi_b'):
                    #     print('k', all_combos, combos_upd)
                    combos_upd_dict[agent] = combos_upd

            # if every binding has an assignment to at least one robot
            if set(itertools.chain.from_iterable(all_combos)) == set(binding_list) and edge[1] in final_states.keys() and edge[0]==edge[1]:
                print('done')
                agent_bindings = combos_upd_dict.copy()
                # agent_bindings = combos_upd_dict
                for a, assignment in combos_upd_dict.items():
                    bindings_a = set(itertools.chain.from_iterable(assignment))
                    agent_bindings[a] = bindings_a

                return path , agent_bindings

            # if edge == ('0', '2', 'pi_b'):
            #     temp = {}
            #     for agent in combos_upd_dict:
            #         temp[agent.id] = combos_upd_dict[agent]

            #     print('combos', temp, path, edge)
            #     print(all_combos)
            #     print(set(itertools.chain.from_iterable(all_combos)), set(binding_list))
            #     sdf  

            # check_newnode = True
            # if every binding has an assignment to at least one robot
            
            if set(itertools.chain.from_iterable(all_combos)) == set(binding_list):
                agent_bindings = combos_upd_dict

                # if valid edge in path, then indicate whether or not the edge is a self-transition or not
                if edge[0] == edge[1]:
                    self_trans_found = True
                else:
                    self_trans_found = False
                # if check_newnode:
                    # print('here2')
                    
                # check next set of edges 
                next_vertex = edge[1]
                for neighbor in buchi_sync_og.neighbors(next_vertex):
                    edges_list = list(buchi_sync_og.out_edges(next_vertex, keys=True))
                    for next_edge in edges_list:
                        # if next_edge not in visited_edges:
                            # make sure we have a self-transition
                            # print('next', edge, next_edge, self_trans_found)
                            if (self_trans_found and next_edge[0] != next_edge[1]) or (not self_trans_found and next_edge[0] == next_edge[1]):
                                # print('adding', (neighbor, next_edge, agent_bindings, path + [next_edge]))
                                # print(next_edge in visited_edges)
                                stack.append((neighbor, next_edge, agent_bindings, path + [next_edge]))
                            # elif not self_trans_found and next_edge[0] == next_edge[1]:


            else:
                # since current edge is not valid in the path, we want to keep finding a self-transition if current edge was also a self-transition
                if edge[0] == edge[1]:
                    self_trans_found = False
                else:
                    self_trans_found = True


                # check other edges
                edges_list = list(buchi_sync_og.out_edges(vertex, keys=True))
                # print('here1', edges_list)
                for next_edge in edges_list:
                    # (n1, n2, pi_sync)
                    # print('next', next_edge, visited_edges)
                    # if next_edge not in visited_edges: 
                        if (self_trans_found and next_edge[0] != next_edge[1]) or (not self_trans_found and next_edge[0] == next_edge[1]): 
                            stack.append((vertex, next_edge, agent_bindings, path[:-1] + [next_edge]))
                            # check_newnode = False
                        







            # print('check', path, edge, len(stack), all_combos)
            # print(set(itertools.chain.from_iterable(all_combos)) == set(binding_list), vertex, edge[1])


def find_teaming_assignment(agent_list, binding_list, buchi_sync_og, init_buchi):
    ''' DFS while keeping track of each robots path to through the edges

    assuming we start with all robots and remove
    '''
    # buchi_path, self_transitions = find_path_buchi(buchi_team, init_buchi)              
    final_states = nx.get_node_attributes(buchi_sync_og, 'accepting')

    agent_bindings = {}         # {agent: set(binding_combos)}
    for agent in agent_list:
        agent_bindings[agent] = agent.possible_bindings

    # print(list(buchi_sync_og.out_edges(keys=True)))
    init_edge = list(buchi_sync_og.out_edges(init_buchi, keys=True))[0] # pick a random edge   (n1, n2, pi_sync)
    init_edge = (init_buchi, init_buchi, 'pi_1')
    # print(init_edge)
    # sdff

    stack = [(init_buchi, init_edge, agent_bindings, [init_edge])]
    valid_nodes = set()
    visited_edges = set()

    while len(stack)!= 0:
        (vertex, edge, agent_bindings, path) = stack.pop()
        # print('path', path, edge)

        
        if edge not in visited_edges:
            visited_edges.add(edge)
            # print('path', path, edge)
            # for a in agent_bindings:
            #     print(a.id, agent_bindings[a])
            
            # edge_data = buchi_sync_og[edge[0]][edge[1]]
            # edge_data = buchi_sync_og.get_edge_data(edge[0], edge[1], key= edge[2])
            # remove robots from team that cannot do this edge 

            # keep track of if the edge is both valid and a self-transition
            if edge[0] == edge[1]:
                self_trans_found = False
            else:
                self_trans_found = True


            agent_bindings_upd = update_buchi_team(agent_bindings, edge, True)

            # eliminate agents in path that cannot do current path

            # get all bindings the agent can do for the current PATH
            team_bindings= set()
            combos_upd_dict = {}
            all_combos = set()

            for agent in agent_bindings_upd.keys():

                combos_upd = agent_bindings_upd[agent]

                # check new edge has valid binding combos
                e1, e2, pi_sync = edge
                try:
                    combos_i = agent.buchi_possible[e1][e2][pi_sync]['binding_combos']

                except Exception as e:
                        # print(e)

                        combos_upd_dict.pop(agent, None)
                combos_upd = combos_upd.intersection(combos_i)


                # for e in path: 
                #     e1, e2, pi_sync = e
                #     try:
                #         combos_i = agent.buchi_possible[e1][e2][pi_sync]['binding_combos']
                #         # if edge == ('0', '2', 'pi_b'):
                #         #     print('i', combos_i)
                #     except Exception as e:
                #         # print(e)

                #         combos_upd_dict.pop(agent, None)

                #     combos_upd = combos_upd.intersection(combos_i)

                    # if e == ('2', '1', 'pi_a') and agent.id == 2:
                    #     print(agent.buchi_possible[e1][e2][pi_sync]['binding_combos'])
                    # # if edge == ('0', '2', 'pi_b'):
                    #     print('j', agent.id, agent_bindings_upd[agent], combos_upd)

                # if agent can't do any bindings
                if len(combos_upd) == 0:
                    combos_upd_dict.pop(agent, None)
                else:
                    all_combos = all_combos.union(combos_upd)
                    # if edge == ('0', '2', 'pi_b'):
                    #     print('k', all_combos, combos_upd)
                    combos_upd_dict[agent] = combos_upd

            # if every binding has an assignment to at least one robot
            if set(itertools.chain.from_iterable(all_combos)) == set(binding_list) and edge[1] in final_states.keys() and edge[0]==edge[1]:
                agent_bindings = combos_upd_dict.copy()
                # agent_bindings = combos_upd_dict
                for a, assignment in combos_upd_dict.items():
                    bindings_a = set(itertools.chain.from_iterable(assignment))
                    agent_bindings[a] = bindings_a
                    # agent_bindings[a] = assignment
                return path , combos_upd_dict

            # if edge == ('0', '2', 'pi_b'):
            #     temp = {}
            #     for agent in combos_upd_dict:
            #         temp[agent.id] = combos_upd_dict[agent]

            #     print('combos', temp, path, edge)
            #     print(all_combos)
            #     print(set(itertools.chain.from_iterable(all_combos)), set(binding_list))
            #     sdf  

            # check_newnode = True
            # if every binding has an assignment to at least one robot
            
            if set(itertools.chain.from_iterable(all_combos)) == set(binding_list):
                agent_bindings = combos_upd_dict.copy()

                # if valid edge in path, then indicate whether or not the edge is a self-transition or not
                if edge[0] == edge[1]:
                    self_trans_found = True
                else:
                    self_trans_found = False
                # if check_newnode:
                    # print('here2')
                # print('good', vertex, path, self_trans_found, [a.id for a in agent_bindings])

                # check next set of edges 
                next_vertex = edge[1]
                for neighbor in buchi_sync_og.neighbors(next_vertex):
                    edges_list = list(buchi_sync_og.out_edges(next_vertex, keys=True))
                    for next_edge in edges_list:

                        # if next_edge not in visited_edges:
                            # make sure we have a self-transition
                            # print('next', edge, next_edge, self_trans_found)
                            if (self_trans_found and next_edge[0] != next_edge[1]) or (not self_trans_found and next_edge[0] == next_edge[1]):
                                # print('adding', (neighbor, next_edge, agent_bindings, path + [next_edge]))
                                # print(next_edge in visited_edges)
                                stack.append((neighbor, next_edge, agent_bindings, path + [next_edge]))
                            # elif not self_trans_found and next_edge[0] == next_edge[1]:

    return [],{}

            # else:
            #     # since current edge is not valid in the path, we want to keep finding a self-transition if current edge was also a self-transition
            #     # if edge[0] == edge[1]:
            #     #     self_trans_found = False
            #     # else:
            #     #     self_trans_found = True

            #     if path[-1][0] == path[-1][1]:
            #         self_trans_found = True
            #     else:
            #         self_trans_found = False


            #     # check other edges
            #     edges_list = list(buchi_sync_og.out_edges(vertex, keys=True))
            #     path_nosync = [x[:2] for x in path ]
            #     # print('here1', edges_list)
            #     for next_edge in edges_list:
            #         # (n1, n2, pi_sync)
            #         print('next', edge, next_edge, vertex, path,self_trans_found)
            #         for a in agent_bindings:
            #             print(a.id, agent_bindings[a])
            #         if next_edge[:2] not in path_nosync: 
            #             if (self_trans_found and next_edge[0] != next_edge[1]) or (not self_trans_found and next_edge[0] == next_edge[1]): 
            #                 stack.append((vertex, next_edge, combos_upd_dict, path + [next_edge]))
                        # check_newnode = False

                        





                        
            # print(stack)
            # print('check', path)
            # print('check', path, edge, len(stack), all_combos)
            # print(set(itertools.chain.from_iterable(all_combos)) == set(binding_list), vertex, edge[1])


  
                
def find_reachable_nodes(buchi_sync_og, buchi_sync):
    '''
    find nodes where edge doesn't exist in the current buchi sync but it can have 
    '''
    found = False

    for buchi_state, data in buchi_sync_og.nodes(data=True):
        buchi_neighbors = list(buchi_sync_og.successors(buchi_state))
        for neighbor_b in buchi_neighbors:
            
            if not buchi_sync.get_edge_data(buchi_state,neighbor_b):
                print('edge data', buchi_state,neighbor_b, buchi_sync.get_edge_data(buchi_state,neighbor_b))
                # add a random edge back in
                edge_idx = list(buchi_sync_og.get_edge_data(buchi_state,neighbor_b).keys())[0]

                buchi_formula = buchi_sync_og.get_edge_data(buchi_state,neighbor_b)[edge_idx]['label']
                pi_sync = buchi_sync_og.get_edge_data(buchi_state,neighbor_b)[edge_idx]['sync']

                buchi_bindings = method2.extract_bindings(buchi_formula)
                buchi_bindings = [int(binding) for binding in buchi_bindings]

                buchi_sync.add_edge(buchi_state,neighbor_b, label=(buchi_formula, pi_sync), key=pi_sync, sync = pi_sync)

                found = True
                return found, buchi_sync
 

    return found, buchi_sync


def is_eligible_agent(agent_buchi, buchi_path, self_transitions):
    for s1, s2, pi_sync in buchi_path:
        if not agent_buchi.get_edge_data(s1,s2, key=pi_sync):
            return False

    # check self transitions
    for s, pi_sync in self_transitions.items():
        if not agent_buchi.get_edge_data(s,s, key=pi_sync):
            return False

    return True


def find_buchi_formulas(buchi, current_state, next_state, pi_sync):
    try:
        buchi_data = buchi[current_state][next_state][pi_sync]
    except:
        return []
    buchi_formula = buchi_data['label']
    # formula_bindings = method2.extract_bindings(buchi_formula)
    formula_bindings = set(re.findall('\[([0-9])\]+',buchi_formula))

    return [int(i) for i in formula_bindings]

    # buchi_data = buchi.edges(data=True)
    # edge_data = [item for item in buchi_data if item[0] == current_state and item[1] == next_state and item[2]['sync'] == pi_sync][0]
    # buchi_formula = edge_data[2]['label']


    # formula_bindings = method2.extract_bindings(buchi_formula)    # all the bindings necessary for the buchi transition
    # return [int(i) for i in formula_bindings]

def check_intermediate(non_inst_props, formula1, formula2):
    '''

    given two buchi transitions e1 and e2, Check if multiple non-inst propositions
    1) change from true to false or from false to true (that are NOT non_inst starting ap, e.g. room1 or push)
        - or true_ex to not true/false, or false_ex to true/false
    2) is true/false on formula2 and not in formula1 (that are NOT non_inst starting ap, e.g. room1 or push)

    3) if (self-trans = False_ex -> non-self = True) OR (self-trans = False -> non-self = True_ex),
            OR (self-trans = True_ex -> non-self = False) OR (self-trans = True -> non-self = False_ex),
        must have binding constraint that set of bindings must be assigned to same agent
        (but doesn't have to only be one agent)
            - this is because we need the intermediate state to have a self-transition for sync purposes,
                but the outputted behavior z_wait = pi (as opposed to pi_c), which the agent cannot do

    INPUTS
    non_inst_actions: list of non_inst propositions
    formula1, formula2: strings of formulas along buchi transitions

    OUTPUTS
    int_props_all: dict of {(ap,binding): True/False} that fit condition 1.
    int_props_ex: dict of {(ap,binding): True_ex/False_ex} that fit condition 1.
    int_props_add: dict of {(ap,binding): True/False/True_ex/False_ex} that that fit condition 2.
    agent_constraints: set of bindings that can only be assigned to one agent
                        (fit condition1 but only with all true to all false/vice-versa)
    '''

    int_props_dict = {}
    int_props_ex = {}
    int_props_add = {}
    agent_constraints = set()
    dict1 = pa.assign_buchi_values(formula1)  # {binding: {ap: True/False/True_ex/False_ex}}
    dict2 = pa.assign_buchi_values(formula2)  # {binding: {ap: True/False/True_ex/False_ex}}

    # flatten to become {(ap,binding int): True/False/True_ex/False_ex}
    ap_binding_dict1 = {}
    ap_binding_dict2 = {}
    for b in dict1:
        for ap in dict1[b]:
            ap_binding_dict1[(ap, b)] = dict1[b][ap]
    for b in dict2:
        for ap in dict2[b]:
            ap_binding_dict2[(ap, b)] = dict2[b][ap]

    bindings_ex = set()
    for (ap, binding), value2 in ap_binding_dict2.items():
        if ap in non_inst_props and ap[-1] == 'c':
            if (ap, binding) in ap_binding_dict1:
                value1 = ap_binding_dict1[(ap, binding)]
            else:
                value1 = None
            # value1 = true/true_ex, value2 = false/false_ex; value1 = false/false_ex, value2 = true/true_ex;
            if (value1 is not None and value2 != value1):
                if '_ex' in value1 or '_ex' in value2:
                    int_props_ex[(ap, binding)] = value2
                else:
                    int_props_dict[(ap, binding)] = value2
            elif value1 is None and (value2 in ("True", "True_ex", "False", "False_ex")):
                # value1 = none, value2 = true
                int_props_add[(ap, binding)] = value2

            # value1 = true, value2 = false; value1 = false, value2 = true;
            if (value1 == "True" and value2 == "False") or (value1 == "False" and value2 == "True"):
                agent_constraints.add(binding)

            # need to check if multiple bindings go from true -> true_ex or false -> false_ex, or vice-versa
            if value2 in ('True_ex', 'False_ex') and value1 != value2 and value1 is not None:
                bindings_ex.add(binding)

    if len(bindings_ex) == 1:
        bindings_ex = set()
    return int_props_dict, int_props_ex, int_props_add, agent_constraints.union(bindings_ex)

def update_intermediate_buchi(non_inst_props, buchi, e1, e2):
    '''
    generate all possible intermediate transitions

    e1: (z1, z2, pi_sync)
    e2: (z2, z3, pi_sync)

    if z1 == z2 and z2 != z3:

    if z1 != z2 and z2 == z3:

    OUTPUT
    updated buchi graph
    new_edges: list of new (z1, z2, pi_sync)
    agent_constraints: tuple of bindings
    '''
    def generate_formula(ap_dict, conj=True):
        '''
        given {(ap,binding): True/False/True_ex/False_ex}, output string that represents the formula

        if conj = True, then conjunct each ap. else, disjunct over each !(ap)
        '''
        if conj:
            formula = ""
        else:
            formula = "("
        for (ap,binding), value in ap_dict.items():
            binding = str(binding)
            if conj:
                if value == "True":
                    prop = ap + ".[" + binding + "]"
                elif value == "False":
                    prop = "!" + ap + ".[" + binding + "]"
                elif value == "True_ex":
                    prop = "!(!" + ap + ".[" + binding + "]" + ")"
                elif value == "False_ex":
                    prop = "!(" + ap + ".[" + binding + "]" + ")"

                formula += prop + " & "

            else:
                if value == "True":
                    # negate to become !(ap^rho)
                    prop = "!(" + ap + ".[" + binding + "]" + ")"
                elif value == "False":
                    # negate to become !(!ap^rho)
                    prop = "!(!" + ap + ".[" + binding + "]" + ")"
                elif value == "True_ex":
                    # negate to become !ap^rho
                    prop = "!" + ap + ".[" + binding + "]"
                elif value == "False_ex":
                    # negate to become ap^rho
                    prop = ap + ".[" + binding + "]"

                formula += prop + " | "

        if conj:
            return formula[:-3]
        else:
            return formula[:-3] + ")"

    binding_constraints = []
    new_edges =[]
    buchi_temp = buchi.copy()
    buchi_temp.remove_edges_from(list(buchi_temp.edges()))

    buchi_formula_dict = nx.get_edge_attributes(buchi, 'label')
    formula1 = buchi_formula_dict[e1]
    formula2 = buchi_formula_dict[e2]

    dict1 = pa.assign_buchi_values(formula1)
    # flatten to become {(ap,binding int): True/False/True_ex/False_ex}
    ap_binding_dict1 = {}
    for b in dict1:
        for ap in dict1[b]:
            ap_binding_dict1[(ap, b)] = dict1[b][ap]

    int_props_dict, int_props_ex, int_props_add, agent_constraints = check_intermediate(non_inst_props, formula1, formula2)

    # no need to generate intermediate states:
    # condition for self to non-self transition
    # if (e2[0] != e2[1]) and (len(int_props_dict) == 0 and len(int_props_ex) <= 1)
    if (e2[0] != e2[1]) and len(int_props_add) == 0:
        return buchi, new_edges, agent_constraints

    # condition for non-self to self transition
    # if (e2[0] == e2[1]) and (len(int_props_dict) + len(int_props_add) <= 1):
    if (e2[0] == e2[1]) and (len(int_props_add) == 0):
        return buchi, new_edges, agent_constraints





    # APPROACH 1: for n number of int_props, generate state with transition
    # sequences = list(itertools.permutations(int_props.keys()))

    # # if truth value of intermediate violates formula1, then intermediate traces are all not valid
    # inters_dict = {x: int_props_add[x] for x in int_props_add if
    #                x not in ap_binding_dict1 or int_props_add[x] == ap_binding_dict1[x]}
    #
    # # if violation, transition is invalid. remove transition z2 to z3
    # # ACTUALLY, no need to check because it will never be invalid
    # # if inters_dict != int_props_dict:
    # #     buchi.remove_edge(e2[0], e2[1], key=formula2)
    #
    # for seq in itertools.permutations(int_props_add):
    #     seq_int = []
    #     for state in seq:
    #     # generate dictionary to flip only one ap at a time based on seq
    #     #     int_dict_ordered =
    #
    #         # check if truth value of intermediate violates formula1
    #         # ACTUALLY, no need to check because it will never be invalid
    #         # inters_dict = {x: int_props_add[x] for x in int_props_add if x not in ap_binding_dict1 or int_props_add[x] == ap_binding_dict1[x]}
    #         #
    #         # # if no violation, transition is valid
    #         # if inters_dict == int_props_dict:
    #         formula_int = formula1 + " & " + generate_formula(inters_dict)
    #         seq_int.append(formula_int)

    # APPROACH 2: only ever need to generate two intermediate buchi states
    # z1 to z_int1: formula1 & [OR over all props in int_props_add]
    # self-trans on z_int1: formula1 & [OR over all props in int_props_add]
    # z_int1 to z_int2: formula1 & [AND over all props in int_props_add]
    # self-trans on z_int2: formula1 & [AND over all props in int_props_add]
    # z_int2 to z2: formula2

    # formula_int1 = formula1 + " & " + generate_formula(int_props_add, conj=False)
    # formula_int2 = formula1 + " & " + generate_formula(int_props_add, conj=True)

    # UPDATE: it's actually like this:
    # z1 to z_int1: formula1 & [OR over negation of all props in int_props_add] ( !(pi1 & pi2) = !(pi1) | !(pi2)
    # self-trans on z_int1: formula1 & [OR over negation of all props in int_props_add] ( !(pi1 & pi2) = !(pi1) | !(pi2)
    # z_int1 to z_int2: formula1 & [AND over all props in int_props_add]
    # self-trans on z_int2: formula1 & [AND over all props in int_props_add]
    # z_int2 to z2: formula2

    # formula_int1 = formula1 + " & " + generate_formula(int_props_add, conj=True) #TODO: CHANGE THIS BACK TO FALSE

    if int_props_add:
        formula_int2 = formula1 + " & " + generate_formula(int_props_add, conj=True)
    else:
        formula_int2 = formula1

    # if formula_int2 requires a starting non-inst prop to be true (e.g. room1, pickup as opposed to room1c, pickupc)
    # then the edge (z1, z2) is not valid and we need to find a new in edge in DFS
    non_inst_props_true = [prop for (prop,b), val in int_props_add.items() if 'True' in val and prop[-1] != 'c']
    if any(i in set(non_inst_props_true) for i in non_inst_props):
        # return buchi as empty graph
        return nx.MultiDiGraph(), [], tuple()



    # TODO: if formula1 and formula for int_props_add conflicts: don't update buchi and edge z1 to z2 is not valid in path

    # add edges in buchi

    if e2[0] != e2[1]:    # if second transition is not a self-transition
        # add intermediate node
        buchi_formula_dict.pop(e2)

        # # z1 to z_int1
        # buchi_formula_dict[(e2[0], e2[0]+"*", "pi_1")] = formula_int1
        # # z_int1 to z_int1
        # buchi_formula_dict[(e2[0] + "*", e2[0] + "*", "pi_1")] = formula_int1
        # # z_int1 to z_int2
        # buchi_formula_dict[(e2[0] + "*", e2[0] + "**", "pi_1")] = formula_int2
        # # z_int2 to z_int2
        # buchi_formula_dict[(e2[0] + "**", e2[0] + "**", "pi_1")] = formula_int2
        # # z_int2 to z2
        # buchi_formula_dict[(e2[0] + "**", e2[1], "pi_1")] = formula2

        # z1 to z_int2
        buchi_formula_dict[(e2[0], e2[0] + "*", "pi_1")] = formula_int2
        # z_int2 to z_int2
        buchi_formula_dict[(e2[0] + "*", e2[0] + "*", "pi_1")] = formula_int2
        # z_int2 to z2
        buchi_formula_dict[(e2[0] + "*", e2[1], "pi_1")] = formula2

        for d, v in buchi_formula_dict.items():
            buchi_temp.add_edge(d[0], d[1], label=v, sync=d[2], key=d[2])
        buchi = buchi_temp

        # buchi.add_edge(e2[0], e2[0]+"*", label=formula_int, sync="pi_1")
        # buchi.add_edge(e2[0] + "*", e2[1], label=formula2, sync="pi_1")
        # buchi.add_edge(e2[0] + "*", e2[0] + "*", label=formula_int, sync="pi_1")
        # buchi.remove_edge(e2[0], e2[1], key = formula2)

        new_edges = [(e2[0], e2[0]+"*", "pi_1"), (e2[0] + "*", e2[1], "pi_1")]
        # new_edges = [(e2[0], e2[0]+"*", "pi_1"), (e2[0] + "*", e2[0] + "**", "pi_1"), (e2[0] + "**", e2[1], "pi_1")]
    else:
        # modify first transition
        buchi_formula_dict[e1] = formula_int2
        nx.set_edge_attributes(buchi, buchi_formula_dict, name='label')
        new_edges = [e1]  # to keep track of formula change

    # write_dot(buchi, 'test2.dot')
    # self.buchi_to_dot(buchi, 'test2.dot')


    # if one explicitly goes all true to all false/vice-versa, then only one agent can be assigned those bindings
    # agent_constraints = set of bindings that can only be assigned to one agent
    # binding_constraints = tuple of binding combos that must be assigned together

    # if more than one is explicitly true to false/vice-versa, then will need to add constraints on binding assignments(??)
    # if len(int_props_dict) > 1:
    #     bindings = set(x[1] for x in int_props_dict)
    #     if len(bindings) > 1:
    #         binding_constraints = tuple(sorted(bindings))

    return buchi, new_edges, {tuple(sorted(agent_constraints))}


def synthesize_team_behavior(team_assignment, sync_buchi, sync_buchi_path):
    '''
    each element in agent_list is agent class object

    assumes agent.path_nosync does not contain initial state


    '''
    agent_list = list(team_assignment.keys())

    buchi_data = sync_buchi.edges(data=True)

    prefix_buchi = sync_buchi_path
    # print('here',prefix_buchi)
    for agent in agent_list:
        agent.path_sync = []

    for buchi_idx in range(len(prefix_buchi)):
        collab_subpath = {}

        current_buchi, next_buchi, pi_sync = prefix_buchi[buchi_idx]
        edge_data = [item for item in buchi_data if item[0] == current_buchi and item[1] == next_buchi and item[2]['sync'] == pi_sync][0]
        buchi_formula = edge_data[2]['label']


        formula_bindings = method2.extract_bindings(buchi_formula)    # all the bindings necessary for the buchi transition
        formula_bindings = [int(i) for i in formula_bindings]
        for agent in agent_list:
            # if pi_sync == ' ':
            agent_prefix = agent.path_nosync[buchi_idx]
            # else:
            #     agent_prefix = agent.path_nosync[buchi_idx][1:]     # don't include initial state to avoid double counting
            if pi_sync == ' ':
                agent.path_sync += agent_prefix
            # don't include initial state to avoid double counting
            else:
                agent.path_sync += agent_prefix[1:]

            # check if agent requires collaboration (assigned bindings overlap w the buchi formula and one robot can't satisfy all the formula bindings 
            # OR one robot can satisfy all the formula bindings and other robots assigned to relevant bindings
            if not agent.assigned_bindings.isdisjoint(set(formula_bindings)) and agent.assigned_bindings != set(formula_bindings):
                collab_subpath[agent] = agent_prefix
            elif agent.assigned_bindings==(set(formula_bindings)) and (any(set(team_assignment[a]).intersection(formula_bindings) and a != agent.id for a in team_assignment)):
                collab_subpath[agent] = agent_prefix
        # for agents that need to collab - add waiting path until the path lengths of all agents are the same
        # assuming the waiting path length is one (every node in prod_aut has a self transition?)
        if len(list(collab_subpath.keys())) > 1:
            max_length = max([len(a.path_sync) for a in collab_subpath])



            # print('buchi', prefix_buchi[buchi_idx], max_length)
            # print(collab_subpath)

            for ag, path in collab_subpath.items():
                wait_length = max_length - len(ag.path_sync)
                # print(ag.id, path)
                if len(path) == 1:
                    wait_state = [path[-1]]

                else:
                    wait_state = [path[-2]]
                # print('wait',ag.id, path, wait_length, wait_state)
                if wait_length > 0:
                    ag.path_sync = ag.path_sync[:-1] + wait_length*wait_state + [path[-1]]
                    # ag.path_sync += wait_length*path[-1]



# if  agent_prefix[-2] is the initial state ' ', then add 
# should i add 0,0 transition at the beginning?



        # # assuming the waiting path length can vary
        # all_path_lengths = [len(p) for p in collab_subpath.values()]

        # if set(all_path_lengths) = 1:     # if all paths are the same length, no need to do anything more
        #     pass
        # else:

        #     path_length = lcm(all_path_lengths)
        #     for ag in collab_subpath.items():

    # for agent in agent_list:
    #     print(agent.id, agent.path_sync, len(agent.path_sync))
        
    return



def assign_bindings2robots(agent_list, binding_list, init_buchi, buchi_og):
    ''' agent_list is a list of Agent objects
    '''

    # assign all robots to all bindings
    dict4buchi = {}

    for ag in agent_list:
        dict4buchi[ag] = binding_list


    # start with all robots as part of the team
    # buchi_team = construct_team_buchi(agent_list, buchi_og)

    # pa.buchi_to_dot(buchi_team, 'buchi_sync_team_og.dot')


    # try to find a path to accepting state in buchi


    # METHOD 2: DFS
    path, team_assignment = find_teaming_assignment(agent_list, binding_list, buchi_og, init_buchi)

    # for ag in team_assignment:
    #     print('dfs assignemnt', ag.id, team_assignment[ag])

    # if no possible team
    if not team_assignment:
        return path, [], team_assignment

    # if agent has binding assignment that isn't possible, pick the max set of bindings it can actually do (e.g. possible = [(1,),(2)] and team_assigment = {1,2})
    # for agent, bindings in team_assignment.items():
    #     if tuple(bindings) not in agent.possible_bindings:
    #         subset_bindings = [b for b in agent.possible_bindings if set(b).issubset(bindings)]
    #         team_assignment[agent] = set(max(subset_bindings, key=len))

    already_assigned_bindings = set()
    for agent, bindings in team_assignment.items():
        possible_bindings = set(tuple(binding) for binding in agent.possible_bindings)
        intersection = possible_bindings & set(bindings)
        print(agent.id, possible_bindings, bindings, intersection)
        possible_bindings_sorted = sorted(list(bindings), key=len, reverse=True)

        # If there is no intersection, find the largest subset. still make sure all bindings are assigned 
        # if not intersection:
        #     subset_bindings = [b for b in possible_bindings if set(b).issubset(bindings)]
        #     # team_assignment[agent] = set(max(subset_bindings, key=len))

        #     temp_list = max(subset_bindings, key=len)
            
        #     # find all subset_bindings with max length
        #     max_bindings = [set(x) for x in subset_bindings if len(x) == len(temp_list)]
        #     for b in max_bindings:
        #         if not b.issubset(already_assigned_bindings):
        #             team_assignment[agent] = b
        #             break
        #         # if all bindings are already assigned, randomly choose the assignment
        #         team_assignment[agent] = set(random.choice(max_bindings))
        # else:
        temp_list = max(bindings, key=len)

        # subset_bindings = [b for b in possible_bindings if set(b).issubset(bindings)]
        # team_assignment[agent] = set(max(subset_bindings, key=len))

        # temp_list = max(subset_bindings, key=len)

        # find all subset_bindings with max length
        max_bindings = [set(x) for x in bindings if len(x) == len(temp_list)]
        for b in possible_bindings_sorted:
            b = set(b)
            if not b.issubset(already_assigned_bindings):
                team_assignment[agent] = b
                break
            # if all bindings are already assigned, randomly choose the assignment
            team_assignment[agent] = set(random.choice(max_bindings))

        # team_assignment[agent] = set(max(intersection, key=len))
            
        already_assigned_bindings.update(team_assignment[agent])





    agent_test = {}
    # for agent in team_assignment:
    #     agent_test[agent.id] = team_assignment[agent]
    # print('dfs: ', path, agent_test)

    # hard code a different solution
    # for agent in team_assignment:
    #     if agent.id == 1:
    #         team_assignment[agent] = {1}
    #     elif agent.id == 3:
    #         team_assignment[agent] = {2}




    # {1: {1, 2}, 2: {3}, 3: {1, 2}}

    self_transitions = {}

    path_copy = path.copy()

    for n1, n2, pi_sync in path_copy:
        if n1 == n2:
            path.remove((n1,n2,pi_sync))
            self_transitions[n1] = pi_sync

    # add 0,0 transition at beginning of path
    # path.insert(0, ('0','0', self_transitions['0']))

    # include loop in accepting state
    acc = path[-1][1]
    path.append((acc, acc, self_transitions[acc]))




    return path, self_transitions, team_assignment

















