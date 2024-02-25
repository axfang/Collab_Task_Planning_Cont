#!/usr/bin/env python

# generate intermediate trace to satisfy continuous controllers

# importing sys
import sys

# adding Folder_2 to the system path
sys.path.insert(0, '../../Mission_Switching/scripts')
sys.path.insert(0, '/Users/amy/usr/lib/python3.7/site-packages/')

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
import create_agents as af
from agent_module import Agent
import find_optimal_run as fr
import find_optimal_run as fir
import bool_parsing
import buchi_functions as bf
from buchi_bindings_module import BuchiBindings
import math
import itertools
import synthesize_team_behavior as stb
import generate_figures as gf
import time
import product_automata as pa
import execute_sync as es
import random
import copy
import testing_methods2 as method2


def sync_discrete(agent_team, team_paths, required_bindings, step, init_buchi, init_pi):
    '''
    given a team of robots and their traces, make it so that all the traces are the same length

    agent_paths = {agent: path}, where path is agent.path_nosync[step]
    '''
    agent_paths = {}

    relevant_agents = []
    for agent in agent_team:
        agent_bindings = set(itertools.chain.from_iterable(agent_team[agent]))
        if agent_bindings.intersection(required_bindings):
            # agent_paths[agent] = team_paths[agent][step]
            relevant_agents.append(agent)
        agent_paths[agent] = team_paths[agent][step]

        # else:
        #     # only include agents that have required binding
        #     team_paths.pop(agent)

    max_length = len(max(agent_paths.values(), key=len))
    if max_length == 0:
        return agent_team, team_paths, [], 0

    # if all paths are the same length
    if all(len(v) == max_length for v in agent_paths.values()):
        return agent_team, team_paths, list(agent_paths.keys()), max_length

    for agent, path in agent_paths.items():
        if len(path) == 1 and step == 0 and max_length > 1 and len(agent_paths[agent]) <= 1:
            q_wait = (agent.init_state, init_buchi, init_pi)
            q_sync = path[0]
            path_upd = (max_length - 1) * [q_wait] + [q_sync]

        elif len(path) == 1 and max_length > 1:
            # q_wait = team_paths[agent][step-1][-1]
            q_wait = (agent.init_state, init_buchi, init_pi)
            q_sync = path[0]
            path_upd = (max_length - 1) * [q_wait] + [q_sync]

        else:
            # if path[-2] is a non-inst starting pi, want to wait in the state before that (which should be a non-inst pi_c)
            # RIGHT NOW IS ONLY FOR ROOMS, WILL NEED TO UPDATE FOR OTHER NON-INST ACTIONS
            # current method: if state does not have self-transition in robot model, take the previous state
            state2check = path[-2][0]
            if agent.graph.has_edge(state2check, state2check):
                q_wait = path[-2]
                q_wait_int = []
                path_length = len(path)
                if path_length == max_length:
                    path_upd = path
                else:
                    if not path[:-2]:
                        path_upd = [path[0]] + [q_wait] * (max_length - path_length) + [path[-1]]
                    else:
                        path_upd = path[:-2] + [q_wait] * (max_length - path_length + 1) + q_wait_int + [path[-1]]
            else:
                q_wait = path[-3]
                q_wait_int = [path[-2]]
                path_length = len(path) - 1
                if path_length == max_length:
                    path_upd = path
                else:
                    if not path[:-3]:
                        path_upd = [path[0]] + [q_wait] * (max_length - path_length) + [path[-1]]
                    else:
                        path_upd = path[:-3] + [q_wait] * (max_length - path_length) + q_wait_int + [path[-1]]
            if len(path_upd) != max_length:
                print('uh oh')

        # agent.path_nosync[step] = path_upd
        team_paths[agent][step] = path_upd

        # agent_team_new[agent] = path_upd
        # print('new',agent.id, path_upd)

    return agent_team, team_paths, relevant_agents, max_length


def check_team_transition2(agent_team, transition, buchi_formula_dict, path_length, path_step):
    ''' check that the TEAM trace does not violate formulas over self transition and edge
    '''

    # for each binding's APs, the relevant robots should all have the same truth values
    buchi_formula = buchi_formula_dict[transition]

    buchi_binding_dict = pa.assign_buchi_values(buchi_formula)

    required_bindings = set(re.findall('\[([0-9])\]+', buchi_formula))
    required_bindings = {int(item) for item in required_bindings}

    binding_ap_dict = {}

    print('required b', path_step, path_length, required_bindings, transition)
    for binding in required_bindings:
        # look only at APs associated with those bindings
        binding_ap_dict = {**binding_ap_dict, **buchi_binding_dict[binding]}

        # find all agents with binding
        agents2check = [k for k, v in agent_team.items() if
                        binding in itertools.chain.from_iterable(v)]  # not required_bindings.isdisjoint(set(v))]

        # for a in agents2check:
        #     print(a.id, agent_team[a], agents2check, path_length)

        if len(agents2check) > 1:

            for step in range(path_length):
                # compare all traces with the first agent
                print('state2check', agents2check[0].path_nosync_int[step])
                agent_state_dict0 = pa.assign_agent_values(agents2check[0].path_nosync_int[step])
                inters_dict0 = {x: binding_ap_dict[x] for x in binding_ap_dict if
                                (x in agent_state_dict0 and agent_state_dict0[x] == agent_state_dict0[x]) or (
                                            x not in agent_state_dict0 and not binding_ap_dict[x])}

                for ag in agents2check[1:]:
                    agent_state_dict = pa.assign_agent_values(ag.path_nosync_int[step])

                    inters_dict = {x: binding_ap_dict[x] for x in binding_ap_dict if
                                   (x in agent_state_dict and agent_state_dict[x] == binding_ap_dict[x]) or (
                                               x not in agent_state_dict and not binding_ap_dict[x])}
                    if inters_dict != inters_dict0:
                        print('violation team!', ag.id, buchi_formula, agent_state_dict)
                        violated = True
                        return violated

    return False


def check_team_transition(agent_team, transition, self_transition, buchi_formula_dict, all_agent_state_dict,
                          path_length, path_step):
    ''' check that the TEAM trace does not violate formulas over self transition and edge
    '''

    # for each binding's APs, the relevant robots should all have the same truth values
    buchi_formula = buchi_formula_dict[transition]
    buchi_formula_self = buchi_formula_dict[self_transition]

    buchi_binding_dict = pa.assign_buchi_values(buchi_formula)
    buchi_binding_dict_self = pa.assign_buchi_values(buchi_formula_self)

    required_bindings = set(re.findall('\[([0-9])\]+', buchi_formula))
    required_bindings = {int(item) for item in required_bindings}

    required_bindings_self = set(re.findall('\[([0-9])\]+', buchi_formula_self))
    required_bindings_self = {int(item) for item in required_bindings_self}

    # agents = list(agent_team.keys())
    agents = list(all_agent_state_dict.keys())

    # assumes all agents have same int_trace length

    # if self-transition, use agent.path_nosync_int, otherwise use q_sync
    # if transition[0] == transition[1]:
    path_idx = range(len(all_agent_state_dict[agents[0]]))
    # else:
    #     path_idx = [-1]

    print('path_idx', path_idx, transition)

    for step in path_idx:
        # if last step in path and will make transition to next buchi node
        # if step == path_idx[-1] and agents[0].path_nosync_int[step][1] == transition[1]:

        if step == path_idx[-1] and all_agent_state_dict[agents[0]][step][1] == transition[1]:
            buchi2check = buchi_binding_dict
            bindings2check = required_bindings
        else:
            buchi2check = buchi_binding_dict_self
            bindings2check = required_bindings_self
        agent_state_dict_all = {}

        for agent in agents:
            # agent_state = agent.path_nosync_int[step][0]
            agent_state = all_agent_state_dict[agent][step][0]
            print(agent.id, all_agent_state_dict[agent], agent_state)
            agent_state_dict = pa.assign_agent_values(agent_state)
            agent_state_dict_all[agent] = agent_state_dict

        for binding in bindings2check:
            # get all agent with required bindings
            agent_list = [agent for agent in agents if binding in set(itertools.chain.from_iterable(agent_team[agent]))]
            binding_ap_dict = buchi2check[binding]

            binding_ap_exist = {ap: binding_ap_dict[ap] for ap in binding_ap_dict if
                                binding_ap_dict[ap] == "False_ex" or binding_ap_dict[ap] == "True_ex"}
            binding_ap_dict_all = {ap: binding_ap_dict[ap] for ap in binding_ap_dict if
                                   binding_ap_dict[ap] == "False" or binding_ap_dict[ap] == "True"}
            print('binding dict', binding, binding_ap_dict_all, binding_ap_exist)

            # first, check all  ap in bindg_ap_dict that is a not_exist
            for ap in binding_ap_exist.keys():
                # check if at least one agent is True/False for this ap
                if binding_ap_exist[ap] == "True_ex":
                    check_agents = [agent for agent in agent_list if (
                                ap not in agent_state_dict_all[agent] or agent_state_dict_all[agent][ap] == "True")]
                    print('True ex', check_agents)
                    if len(check_agents) == 0:
                        print('violation for exist!')
                        return violated

                else:
                    print('agents', step)
                    print(agent_state_dict_all)
                    print(agent_list)
                    check_agents = [agent for agent in agent_list if (
                                ap not in agent_state_dict_all[agent] or agent_state_dict_all[agent][ap] == "False")]
                    print('False ex', check_agents)
                    if len(check_agents) == 0:
                        print('violation for exist!')
                        violated = True
                        return violated

            # then check rest of aps
            if binding_ap_dict_all:
                for agent in agent_list:
                    print('agent_state', agent_state)
                    agent_state_dict = agent_state_dict_all[agent]
                    print('buchi aps', agent.id, binding_ap_dict_all, agent_state_dict)
                    inters_dict = {x: binding_ap_dict_all[x] for x in binding_ap_dict_all if
                                   (x in agent_state_dict and agent_state_dict[x] == binding_ap_dict_all[x]) or (
                                               x not in agent_state_dict and not binding_ap_dict_all[x])}
                    print('intersect', inters_dict)
                    if inters_dict != binding_ap_dict_all:
                        print('violation team all!', agent.id, buchi_formula, agent_state_dict)
                        violated = True
                        return violated

        # for agent in agents:
        #     for binding in set(itertools.chain.from_iterable(agent_team[agent])):
        #         # only care about bindings relevant to buchi formula
        #         if binding in buchi_binding_dict:
        #             binding_ap_dict = buchi_binding_dict[binding]

        #             agent_state = agent.path_nosync_int[step][0]
        #             print('agent_state', agent_state)
        #             agent_state_dict = pa.assign_agent_values(agent_state)
        #             print('buchi aps', agent.id, binding_ap_dict, agent_state_dict )
        #             inters_dict = {x: binding_ap_dict[x] for x in binding_ap_dict if (x in agent_state_dict and agent_state_dict[x] == binding_ap_dict[x]) or (x not in agent_state_dict and not binding_ap_dict[x])}
        #             print('intersect', inters_dict)
        #             if inters_dict != binding_ap_dict:
        #                 print('violation team!', agent.id, buchi_formula, agent_state_dict)
        #                 violated = True
        #                 return violated

    print('no violation for exist!')
    return False


def check_team_behavior(agent_team, buchi_edge, self_transition, buchi_formula_dict, path_length, path_step):
    ''' check that the TEAM trace does not violate formulas over self transition and edge

    first check path over self transition, then check path over actual transition
    '''

    # first, check self transition
    violated = check_team_transition(agent_team, buchi_edge, self_transition, buchi_formula_dict, path_length,
                                     path_step)

    print('self check', violated)

    # if not violated:
    #     violated = check_team_transition(agent_team, buchi_edge, self_transition, buchi_formula_dict, path_length, path_step)
    #     print('non-self check', violated)
    return violated


def generate_intermediate_trace(agent_team, team_paths, buchi, init_buchi, sync_buchi_path, self_transitions):
    ''' given behavior for a team of robots, add an intermediate trace that does not violate original spec.

    INPUTS:
    agent_team              {agent:{(bindings)}}
    buchi                   buchi automaton
    sync_buchi_path         path in buchi
    self_transitions        self transitions in buchi
    '''

    buchi_formula_dict = nx.get_edge_attributes(buchi, 'label')

    init_pi = self_transitions[init_buchi]

    agent_team_new = agent_team.copy()
    # team_paths_og = team_paths.copy()
    # team_paths_og = copy.deepcopy(team_paths)

    for ag in agent_team:
        ag_new = ag
        ag_new.path_nosync_cont = ag_new.path_nosync
        agent_team_new[ag_new] = agent_team_new.pop(ag)
        # ag_new.path_nosync_int = []
        # team_paths_og[ag] = ag.path_nosync

    for i in range(len(sync_buchi_path)):
        i = -1

        transition = sync_buchi_path[i]
        buchi_state = transition[1]
        pi_sync = transition[2]

        # get self-transition formula
        buchi_formula = buchi_formula_dict[(transition[0], transition[0], self_transitions[transition[0]])]
        buchi_binding_dict = pa.assign_buchi_values(buchi_formula)

        # get non self-transition formula
        buchi_formula_next = buchi_formula_dict[transition]
        required_bindings = set(re.findall('\[([0-9])\]+', buchi_formula)).union(
            set(re.findall('\[([0-9])\]+', buchi_formula_next)))
        required_bindings = {int(item) for item in required_bindings}
        print('required', required_bindings, buchi_formula, buchi_formula_next, sync_buchi_path)

        agent_team, team_paths, relevant_agents, path_length = sync_discrete(agent_team, team_paths, required_bindings,
                                                                             -1, init_buchi, init_pi)
        relevant_team = {}

        team_paths_og = copy.deepcopy(team_paths)
        # for a in team_paths:
        #     if len(team_paths[a]) != path_length:
        #         print('why')

        # if multiple robots are moving to different rooms, add intermediate trace
        if len(relevant_agents) <= 1:
            print('no need to check intermediate')
            return agent_team, team_paths

        # # if path length = 1, that means the only state in the path is the sync state. so get the prior state and treat it as the wait state
        # if path_length == 1:
        #     for a in agent_team:
        #         team_paths[a][-1] = team_paths[a][-2] + team_paths[a][-1]

        # following code is for if we just want to check sync state and state right before it
        '''
        # get (waiting state, sync state) for each robot
        for ag in relevant_agents:

            if i == 0:    # initial step
                path = [(ag.init_state, init_buchi)] + ag.path_nosync[-1]
            else:
                path = ag.path_nosync[-1]

            print('bw', ag.id, path)


            if len(path) >= 2:
                q_wait = path[-2]
                q_sync = path[-1]
            else:
            # elif i > 0 and len(path) <2:
                q_wait = (path[-1][0], transition[0], self_transitions[transition[0]])
                q_sync = path[-1]

            print('wat', q_wait,q_sync)
            # print(path, i)

            # find room state (assumes first state in cap model is location)
            # loc_wait = re.findall(r'\s([a-z1-9]*)\s', q_wait[0][0])
            # loc_sync = re.findall(r'\s([a-z1-9]*)\s', q_sync[0][0])
            loc_wait = q_wait[0][0]
            loc_sync = q_sync[0][0]

            # if loc_wait != loc_sync:
            states_dict[ag] = (q_wait,q_sync)
        '''

        # if len(states_dict.keys())>1:
        print(transition, list(relevant_agents))

        # check all possible traces
        # all_int_traces = pa.powerset(states_dict.keys())    # each element is a combination of agents to generate traces
        all_int_traces = list(
            itertools.permutations(relevant_agents))  # e.g. a1 first, then a2, then a3/ a2 first, then a1, then a3/ ...
        # int_states_dict = {}
        # for agent in states_dict:
        #     q_wait, q_sync = states_dict[agent]
        #     int_states_dict[agent] = (((q_sync[0][0],) + q_wait[0][1:]), q_wait[1], pi_sync)
        # print('int', int_states_dict)

        violation_num = 0

        # TO DO:::: check the ordering for EVERY step in path
        step_idx = 0
        for step in range(path_length - 1):
            # get (waiting state, sync state) for each robot
            states_dict = {}

            for ag in relevant_agents:
                relevant_team[ag] = agent_team[ag]

                if i == 0:  # initial step
                    path = [(ag.init_state, init_buchi)] + team_paths_og[ag][i]
                else:
                    path = team_paths[ag][i]

                print('bw', ag.id, path)
                print(path_length, len(path))

                # if path[-2] is a non-inst starting pi, want to wait in the state before that (which should be a non-inst pi_c)
                # RIGHT NOW IS ONLY FOR ROOMS, WILL NEED TO UPDATE FOR OTHER NON-INST ACTIONS
                # current method: if state does not have self-transition in robot model, take the previous state
                state2check = path[-2][0]
                if ag.graph.has_edge(state2check, state2check):
                    if len(path) >= 2:

                        q_wait = path[step]
                        q_wait_int = []
                        q_sync = path[step + 1]
                    else:
                        # elif i > 0 and len(path) <2:
                        q_wait = (path[step][0], transition[0], self_transitions[transition[0]])
                        q_wait_int = []
                        q_sync = path[step + 1]
                else:
                    if len(path) >= 2:
                        if step == 0:
                            # take previous state in previous buchi transition (aka "current state"
                            q_wait = team_paths[ag][-2][-1]
                        else:
                            q_wait = path[step - 1]
                        q_wait_int = path[step]
                        q_sync = path[step + 1]
                    else:
                        if step == 0:
                            prev_state = team_paths[ag][-2][-1]
                            q_wait = (prev_state[0], transition[0], self_transitions[transition[0]])
                        else:
                            # elif i > 0 and len(path) <2:
                            q_wait = (path[step][0], transition[0], self_transitions[transition[0]])
                        q_wait_int = path[step]
                        q_sync = path[step + 1]

                print('wat', q_wait, q_wait_int, q_sync)
                # print(path, i)

                # if loc_wait != loc_sync:
                states_dict[ag] = (q_wait, q_wait_int, q_sync)

            all_valid_trace = True
            violated_before = False
            violated = False
            one_valid_trace = False

            # for a trace
            for trace in all_int_traces:
                int_states_dict = {}
                # for each agent
                for agent_order in range(1, len(trace) + 1):

                    agent = trace[agent_order - 1]
                    q_wait, q_wait_int, q_sync = states_dict[agent]
                    # int_states_dict[agent] = [q_wait, q_sync]
                    # agent.path_nosync_int = int_states_dict[agent]

                    if not q_wait_int:
                        # the intermediate states with q_sync should have the same buchi state and pi_sync as q_wait
                        q_sync_int = (q_sync[0], q_wait[1], q_wait[2])
                        int_states_dict[agent] = [q_wait] * agent_order + [q_sync_int] * (
                                    len(trace) - agent_order + 1) + [
                                                     q_sync]  # just the states that need to be inserted PLUS q_sync
                    else:
                        q_sync_int = (q_sync[0], q_wait[1], q_wait[2])
                        int_states_dict[agent] = [q_wait] * agent_order + [q_wait_int] + [q_sync_int] * (
                                    len(trace) - agent_order) + [
                                                     q_sync]

                    agent.path_nosync_int = int_states_dict[agent]

                    # following code is for if we jus want to check sync state and state right before it
                    '''
                    q_wait, q_sync = states_dict[agent]
                    print(agent.id, q_wait, q_sync)

                    # the intermediate states with q_sync should have the same buchi state and pi_sync as q_wait
                    q_sync_int = (q_sync[0], q_wait[1], q_wait[2])
                    int_states_dict[agent] = [q_wait]*agent_order + [q_sync_int]*(len(trace)-1 - agent_order) + [q_sync]  # just the states that need to be inserted PLUS q_sync
                    
                    # agent.path_nosync_int = agent.path_nosync_int[:-1] + int_states_dict[agent] + [agent.path_nosync_int[-1]]
                    agent.path_nosync_int = int_states_dict[agent]
                    '''

                print('int', int_states_dict)
                # debugging
                c = 0
                for x in int_states_dict:
                    if c == 0:
                        checklen = len(int_states_dict[x])
                    else:
                        if checklen != len(int_states_dict[x]):
                            print('something is wrong')
                    c += 1

                # if all possible intermediate traces do not violate spec (doesn't matter which robot gets to a room first) - no need to update discrete path
                # else, will need to update
                violated = False
                print('trace', trace)

                for agent in trace:

                    binding_ap_dict = {}
                    # check if states of all robots violates current transition

                    for binding in set(itertools.chain.from_iterable(agent_team[agent])):
                        # if binding is not in buchi_binding_dict, that means that there aren't required APs for that binding so any robot can do this binding at this state
                        if binding in buchi_binding_dict:
                            # look only at APs associated with that binding
                            binding_ap_dict = buchi_binding_dict[binding]

                    # print('bindings', ag.assigned_bindings, binding_ap_dict)

                    # check that all agents in this int trace satisfies the formula

                    # agent_state = states_dict[agent][0][0]
                    # print('remove', states_dict[agent])
                    # print('int state', agent_state, int_states_dict[agent])
                    # agent_state_dict = pa.assign_agent_values(agent_state)
                    # print(binding_ap_dict, agent_state_dict)

                    # agent_state_dict = pa.assign_agent_values(int_states_dict[ag][0])

                    # inters_dict = {x: binding_ap_dict[x] for x in binding_ap_dict if (x in agent_state_dict and agent_state_dict[x] == binding_ap_dict[x]) or (x not in agent_state_dict and not binding_ap_dict[x])}
                    # if inters_dict != binding_ap_dict:
                    #     print('violation!')#, ag.id, buchi_formula, agent_state_dict)
                    #     violated = True
                    #     break
                    # else:

                    # check no violations across traces within team
                    print('cehcking teams', transition, self_transitions[transition[0]])
                    # if violation
                    self_transition = (transition[0], transition[0], self_transitions[transition[0]])

                    # if check_team_behavior(relevant_team, transition, self_transition, buchi_formula_dict, path_length, i):
                    if check_team_transition(agent_team, transition, self_transition, buchi_formula_dict,
                                             int_states_dict, path_length, i):
                        violated = True
                        violated_before = True
                        violation_num += 1
                        print('violation for this trace!')
                        break

                # if found a valid trace, store it
                if not violated and not one_valid_trace:

                    one_valid_trace = True
                    print('no violation for at least one trace')
                    for rel_ag in relevant_agents:
                        if rel_ag in trace:
                            # else:
                            print('ok!', trace, rel_ag.id, buchi_formula)
                            q_wait, q_wait_int, q_sync = states_dict[rel_ag]
                            # print(q_wait[0], q_sync[0])
                            # ag.path_nosync[i] = ag.path_nosync[i][:-1] + [int_states_dict[ag], q_sync]
                            print('old path', rel_ag.id, team_paths[rel_ag][i])
                            # team_paths[rel_ag][i] = team_paths[rel_ag][i][:-1] + int_states_dict[rel_ag]
                            print('insert path', int_states_dict[rel_ag])

                            # nested list to keep track of which states are getting updated
                            team_paths[rel_ag][i][step:step + 1] = [int_states_dict[rel_ag][:-1]]
                            print('new path', team_paths[rel_ag][i])

                            agent_team_new[rel_ag] = agent_team_new.pop(rel_ag)
            if one_valid_trace and not violated_before:
                print('no violates for all traces, no need to update team path')
                for a1 in team_paths:
                    for a2 in team_paths_og:
                        if a1.id == a2.id:
                            team_paths[a1][i][step] = team_paths_og[a2][i][step]
            elif one_valid_trace and violated_before:
                print('found valid trace, but not all traces are valid')
                # flatten list
                team_paths[rel_ag][i][step:step + 1] = team_paths[rel_ag][i][step]

                # return agent_team_og, team_paths_og

                # break

                # else:
                #     print('not ok!')
                #     q_wait, q_sync = states_dict[rel_ag]
                #     # ag.path_nosync[i] = ag.path_nosync[i][:-1] + [q_wait, q_sync]
                #     # team_paths[ag][i] = team_paths[ag][i][:-1] + [q_wait, q_sync]
                #
                #     agent_team_new[rel_ag] = agent_team_new.pop(rel_ag)

        # if all possible int traces don't violate, output original paths (remove this if we want to be efficient and just output the first non violating int trace)
        # if one_valid_trace and not violated_before:
        #     print('no violates, no need to update team path', sync_buchi_path)
        #     return agent_team, team_paths_og
        # # if found a trace in which the team trace still satisfies the task
        # if one_valid_trace and violated_before:
        #     print('no violation but not all paths are valid')
        if one_valid_trace:
            print('no violation')

            # for ag in relevant_agents:
            #     if ag in trace:
            #         # else:
            #         print('ok!', trace, ag.id,buchi_formula)
            #         q_wait, q_sync = states_dict[ag]
            #         # print(q_wait[0], q_sync[0])
            #         # ag.path_nosync[i] = ag.path_nosync[i][:-1] + [int_states_dict[ag], q_sync]  
            #         print('old path', team_paths[ag][i])
            #         team_paths[ag][i] = team_paths[ag][i][:-1] + int_states_dict[ag]
            #         print('new path', team_paths[ag][i])

            #         agent_team_new[ag] = agent_team_new.pop(ag)
            #     else:
            #         q_wait, q_sync = states_dict[ag]
            #         # ag.path_nosync[i] = ag.path_nosync[i][:-1] + [q_wait, q_sync] 
            #         # team_paths[ag][i] = team_paths[ag][i][:-1] + [q_wait, q_sync]  

            #         agent_team_new[ag] = agent_team_new.pop(ag)

            return agent_team_new, team_paths

    print('violation across team!')
    # else, team does not work 
    for agent in agent_team_new:
        agent_team_new[agent] = set()
    return agent_team_new, team_paths


def get_possible_team_assignment(team_assignment):
    already_assigned_bindings = set()
    for agent, bindings in team_assignment.items():
        # possible_bindings = set(tuple(binding) for binding in agent.possible_bindings)
        intersection = set(bindings)
        possible_bindings_sorted = sorted(list(intersection), key=len, reverse=True)

        temp_list = max(intersection, key=len)

        # subset_bindings = [b for b in possible_bindings if set(b).issubset(bindings)]
        # team_assignment[agent] = set(max(subset_bindings, key=len))

        # temp_list = max(subset_bindings, key=len)

        # find all subset_bindings with max length
        max_bindings = [set(x) for x in intersection if len(x) == len(temp_list)]
        for b in possible_bindings_sorted:
            b = set(b)
            if not b.issubset(already_assigned_bindings):
                team_assignment[agent] = {tuple(sorted(list(b)))}
                break
            # if all bindings are already assigned, randomly choose the assignment
            # team_assignment[agent] = set(random.choice(max_bindings))
            assignment = set(random.choice(max_bindings))
            team_assignment[agent] = {tuple(sorted(list(assignment)))}

        # team_assignment[agent] = set(max(intersection, key=len))

        already_assigned_bindings.update(team_assignment[agent])

    return team_assignment


def update_buchi_team_intermediate(agent_bindings, sync_buchi_path, buchi, binding_list, team_paths):
    '''
    add (or remove) edge from team buchi and update team of robots that can do it
    agent_bindings = {agent: set(binding_combos)}

    sync_buchi_path includes one self transition, one non self transition

    edge = (n1, n2, data)

    output updated agent_bindings and agent_paths
    '''

    # first, update possible bindings the robot can do 
    # for each combo of binding assignments in which all are assigned across robots
    # check intermediate traces
    # if none of them work, try a different combo

    # in dfs, when backtracking, backtrack to a different binding assignment. If none work, backtrack to different edge

    self_e, edge = sync_buchi_path
    # agent_bindings_upd = stb.update_buchi_team(agent_bindings, edge, True)
    self_transitions = {self_e[0]: self_e[-1]}

    agent_bindings_upd = get_possible_team_assignment(agent_bindings)
    print('edges', [edge], self_transitions, agent_bindings_upd)

    # pick a combo out of bindings assignments

    for agent in agent_bindings_upd:
        # if init:
        # agent_start = agent.path_nosync[-1][0]
        # agent_start = agent.path_nosync[0][-1][0]

        print(team_paths[agent])
        agent_start = team_paths[agent][-1][-1][0]
        buchi_start = team_paths[agent][-1][-1][1]
        print('start', agent_start, buchi_start, edge[0])
        # else:
        #     # agent_start = agent.init_state
        #     # init is equal to init_buchi
        #     buchi_start = init
        # print(agent_start)
        # print('start', agent.id, agent_bindings_upd[agent])
        agent_binding_set = set(itertools.chain.from_iterable(agent_bindings_upd[agent]))

        edge_path, agent.cost = stb.find_shortest_path_robot(agent.prod_aut, buchi, agent_start, [edge],
                                                             self_transitions, agent_binding_set)
        print('edge_path', edge_path, agent_binding_set)

        # if agent doesn't work
        if agent.cost == float('inf'):
            agent_bindings_upd[agent] = set()
        else:

            print('before', agent.id, agent_bindings_upd[agent], edge_path, edge)
            print(team_paths[agent])
            # agent.path_nosync.append(edge_path[0])
            agent.path_nosync += edge_path
            team_paths[agent] = team_paths[agent] + edge_path
            print('updated', team_paths[agent])

            # print('apath',agent.path_nosync)
            # print('aedge', edge_path)

    print('a', agent_bindings_upd)
    return generate_intermediate_trace(agent_bindings_upd, team_paths, buchi, buchi_start, [edge], self_transitions)


def possible_binding_assignments(bindings, all_bindings, robot_dict, current_assignment={}, robot_ids=None):
    '''
    return a generator of dictionaries with all the possible binding assignments that satisfy all bindings. If none are possible, outputs empty

    initialize bindings = all_bindings

    robot_dict = {agent: [binding combos]}
    '''
    if robot_ids is None:
        robot_ids = list(robot_dict.keys())
    if not bindings:
        yield current_assignment
        return

    for binding in robot_dict[robot_ids[0]]:
        if set(binding).issubset(all_bindings):
            new_assignment = {**current_assignment, robot_ids[0]: {binding}}
            if len(binding) == 0:
                new_assignment.pop(robot_ids[0])

            remaining_bindings = list(set(bindings) - set(binding))
            if len(robot_ids) > 1:
                yield from possible_binding_assignments(remaining_bindings, all_bindings, robot_dict, new_assignment,
                                                        robot_ids[1:])
            else:
                if not remaining_bindings:
                    yield new_assignment


def find_teaming_assignment(agent_list, binding_list, buchi_sync_og, init_buchi, non_inst_props):
    ''' DFS while keeping track of each robots path to through the edges
    AND generate intermediate buchi nodes when necessary

    assuming we start with all robots and remove
    '''
    # buchi_path, self_transitions = find_path_buchi(buchi_team, init_buchi)
    final_states = nx.get_node_attributes(buchi_sync_og, 'accepting')

    agent_bindings = {}  # {agent: set(binding_combos)}
    for agent in agent_list:
        agent_bindings[agent] = pa.powerset(binding_list)

    # print(list(buchi_sync_og.out_edges(keys=True)))
    init_edge = list(buchi_sync_og.out_edges(init_buchi, keys=True))[0]  # pick a random edge   (n1, n2, pi_sync)
    init_edge = (init_buchi, init_buchi, 'pi_1')
    # print(init_edge)
    # sdff
    binding_constraints_upd = set()

    stack = [(buchi_sync_og, init_buchi, init_edge, agent_bindings, [init_edge], binding_constraints_upd)]
    valid_nodes = set()
    visited_edges = set()

    while len(stack) != 0:
        (buchi, vertex, edge, agent_bindings, path, binding_constraints_upd) = stack.pop()
        # print('path', path, edge)

        if edge not in visited_edges:
            visited_edges.add(edge)
            valid_buchi = True
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

            if len(path) > 1:
                e1 = path[-2]
                e2 = edge
                buchi_upd, new_edges, binding_constraints = stb.update_intermediate_buchi(non_inst_props, buchi, e1, e2)
                if len(buchi_upd.nodes()) == 0:
                    valid_buchi = False
                    # remove immediate neighbors from visited edges
                    neighbors = set(buchi.out_edges(e2[0], keys=True))
                    neighbors.remove(e1)
                    visited_edges = visited_edges - neighbors
                    stack.pop()

                elif len(new_edges) == 0:
                    edges_sub_buchi = []
                elif len(new_edges) == 1:
                    # non-self to self transition
                    self_e = (e1[0], e1[0], 'pi_1')
                    edges_sub_buchi = [self_e] + new_edges
                else:
                    # self transition to non-self
                    edges_sub_buchi = [e1] + new_edges
            else:
                buchi_upd, new_edges, binding_constraints = buchi, [], set()
                edges_sub_buchi = []

            if valid_buchi:
                # if non-self to self transition: new_edges = [(z1, z_int1), (z_int1, z_int2), (z_int2, z2)]
                # otherwise, new_edges = [(z1, z2)] and we need to update the formula in prod_aut
                agent_bindings_upd = agent_bindings.copy()
                for agent in agent_bindings_upd:
                    # agent.prod_aut = agent.prod_aut_prev.copy()
                    agent.prod_aut = update_prod_aut2(agent, buchi_upd, edge, edges_sub_buchi, agent_bindings_upd[agent], binding_list)
                    # agent_bindings2[agent] = agent_bindings[agent]
                # agent_bindings_upd = stb.update_buchi_team(agent_bindings, edge, init_buchi,
                #                                            binding_constraints=binding_constraints)
                if len(new_edges) == 2:
                    path2check = path[:-1] + new_edges
                else:
                    path2check = path
                agent_bindings_upd = stb.update_buchi_team(agent_bindings_upd, edge, buchi_upd, path2check, init_buchi, binding_constraints=binding_constraints)

                # eliminate agents in path that cannot do current path

                # get all bindings the agent can do for the current PATH
                team_bindings = set()
                combos_upd_dict = {}
                all_combos = set()

                for agent in agent_bindings_upd.keys():

                    combos_upd = agent_bindings_upd[agent]

                    # if agent can't do any bindings
                    if len(combos_upd) == 0:
                        combos_upd_dict.pop(agent, None)
                    else:
                        all_combos = all_combos.union(combos_upd)
                        # if edge == ('0', '2', 'pi_b'):
                        #     print('k', all_combos, combos_upd)
                        combos_upd_dict[agent] = combos_upd

                # if every binding has an assignment to at least one robot
                if set(itertools.chain.from_iterable(all_combos)) == set(binding_list) and edge[1] \
                        in final_states.keys() and edge[0] == edge[1]:
                    agent_bindings_upd = combos_upd_dict.copy()
                    # agent_bindings = combos_upd_dict
                    for a, assignment in combos_upd_dict.items():
                        bindings_a = set(itertools.chain.from_iterable(assignment))
                        agent_bindings_upd[a] = bindings_a
                        # agent_bindings[a] = assignment
                    return path, combos_upd_dict, buchi_upd, binding_constraints_upd

                # if every binding has an assignment to at least one robot
                if set(itertools.chain.from_iterable(all_combos)) == set(binding_list):
                    agent_bindings_upd = combos_upd_dict.copy()

                    # if valid edge in path, then indicate whether or not the edge is a self-transition or not
                    if edge[0] == edge[1]:
                        self_trans_found = True
                    else:
                        self_trans_found = False
                    # if check_newnode:
                    # print('here2')
                    # print('good', vertex, path, self_trans_found, [a.id for a in agent_bindings])

                    # check next set of edges
                    if not new_edges:
                        next_vertex = edge[1]
                    else:
                        next_vertex = new_edges[-1][1]
                        path = path[:-1]
                    for neighbor in buchi_upd.neighbors(next_vertex):
                        edges_list = list(buchi_upd.out_edges(next_vertex, keys=True))
                        for next_edge in edges_list:

                            # if next_edge not in visited_edges:
                            # make sure we have a self-transition
                            # print('next', edge, next_edge, self_trans_found)
                            if (self_trans_found and next_edge[0] != next_edge[1]) or (
                                    not self_trans_found and next_edge[0] == next_edge[1]):
                                # print('adding', (neighbor, next_edge, agent_bindings, path + [next_edge]))
                                # print(next_edge in visited_edges)
                                # for a in agent_bindings:
                                #     a.prod_aut_prev = a.prod_aut.copy()
                                binding_constraints_upd = binding_constraints_upd.union(binding_constraints)
                                stack.append((
                                             buchi_upd, neighbor, next_edge, agent_bindings_upd, path + new_edges + [next_edge],
                                             binding_constraints_upd))
                            # elif not self_trans_found and next_edge[0] == next_edge[1]:

    return [], {}, buchi_upd, []


def assign_bindings2robots(agent_list, binding_list, init_buchi, buchi_og, non_inst_props):
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
    path, team_assignment, buchi_upd, binding_constraints = find_teaming_assignment(agent_list, binding_list, buchi_og,
                                                                                    init_buchi, non_inst_props)

    # for ag in team_assignment:
    #     print('dfs assignemnt', ag.id, team_assignment[ag])

    # if no possible team
    if not team_assignment:
        return path, [], team_assignment, buchi_og, []

    # if agent has binding assignment that isn't possible, pick the max set of bindings it can actually do (e.g. possible = [(1,),(2)] and team_assigment = {1,2})

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

    self_transitions = {}

    path_copy = path.copy()

    for n1, n2, pi_sync in path_copy:
        if n1 == n2:
            path.remove((n1, n2, pi_sync))
            self_transitions[n1] = pi_sync
        if n2[-1] == '*':
            self_transitions[n2] = 'pi_1'

    # add 0,0 transition at beginning of path
    # path.insert(0, ('0','0', self_transitions['0']))

    # include loop in accepting state
    acc = path[-1][1]
    path.append((acc, acc, self_transitions[acc]))

    return path, self_transitions, team_assignment, buchi_upd, binding_constraints


def update_prod_aut2(agent, buchi, edge_og, edges, agent_bindings, all_bindings):
    '''
    Update an agent's original product automaton given intermediate nodes in the buchi

    edges:
        if non-self to self trans: [(z1, z*, pi_sync), (z*, z*, pi_sync), (z**,z2,pi_sync)] e.g. ('1', '1*', 'pi_1')
        if self to non-self: [(z1, z2, pi_sync)]

    Returns
    -------

    '''

    prod_aut = agent.prod_aut
    # agent_bindings = agent.assigned_bindings

    if len(edges) == 0:
        return prod_aut

    # self transition to non-self transition
    if len(edges) > 2:
        # (z1, z_int1, pi_sync1) = edges[0]
        # (z_int1, z_int2, pi_sync2) = edges[1]
        # (z_int2, z2, pi_sync3) = edges[2]

        (z1, z_int1, pi_sync1) = edges[1]
        (z_int1, z2, pi_sync2) = edges[2]

        pi_sync_og = edge_og[2]

        e0, e1, e2 = edges

        sub_buchi = buchi.edge_subgraph(edges + [(z_int1, z_int1, 'pi_1')]) #, (z_int2, z_int2, 'pi_1')])
        nx.set_node_attributes(sub_buchi, {z1: 'I'}, 'init_temp')
        nx.set_node_attributes(sub_buchi, {z2: 'F'}, 'acc_temp')

        # get subgraph with the relevant edges
        agent_edges = [(from_node[0], to_node[0]) for from_node, to_node in prod_aut.edges()
                       if (from_node[1], to_node[1]) in [(e1[0], e2[1]), (e1[0], e1[0])]]

        sub_agent = agent.graph.edge_subgraph(agent_edges)

        sub_prod, b = pa.construct_prod_aut(sub_agent, sub_buchi, all_bindings, sub_buchi=True)

        # remove edges from z1 to z2

        edges2remove = [(from_node, to_node) for from_node, to_node in prod_aut.edges()
                        if (from_node[1], to_node[1]) == (e1[0], e2[1])]
        prod_aut.remove_edges_from(edges2remove)

        prod_aut_upd = nx.compose(prod_aut, sub_prod)


    else:
        # non-self to self transition. update edges from z1 to z2 based on new formula
        prod_aut_upd = prod_aut.copy()
        z1, z2, pi_sync = edges[1]
        buchi_formula = nx.get_edge_attributes(buchi, 'label')[edges[1]]
        buchi_binding_dict = pa.assign_buchi_values(buchi_formula)

        # get subgraph with the relevant edges
        prod_edges = [(from_node[0], to_node[0], data['sync']) for from_node, to_node, data in prod_aut.edges(data=True)
                      if (from_node[1], to_node[1], data['sync']) == (z1, z2, pi_sync)]

        edges2remove = [(from_node, to_node, data['sync']) for from_node, to_node, data in prod_aut.edges(data=True)
                        if (from_node[1], to_node[1]) == (z1, z2) and data['sync'] != pi_sync]

        # remove any edges that are no longer valid
        for edge in prod_edges:
            s1, s2, pi_sync = edge
            # agent_state_dict1 = pa.assign_agent_values(s1)
            agent_state_dict = pa.assign_agent_values(s2)
            if not pa.check_state_with_buchi(buchi_formula, buchi_binding_dict, agent_state_dict, agent_bindings):
                # prod_aut_upd.remove_edge(edge, key=pi_sync)
                prod_aut_upd.remove_edge((s1, z1),(s2,z2), key=pi_sync)


        # remove all other non-self transitions
        prod_aut_upd.remove_edges_from(edges2remove)

    return prod_aut_upd


if __name__ == '__main__':
    # spec = 'F((room9 & beep).[2 & 3] & (room1 & pickup).[1]) & (!pickup.[1] U (room1 & ((scan | camera) & !(scan & camera))).[2])'

    # spec = 'F(room1.[1] & room2.[2]) & G(room1.[1] -> room2.[2]) & G(room2.[2] -> room1.[1])'
    # spec = 'F((room1 & pickup).[1] & F((dropoff & room4).[1] & F(scan.[2] & (camera.[3]) & (room4.[(2 & 3)])))) & G(room4.[(1 | (2 | 3))] -> room4.[4])'

    # spec = 'F(room1.[(1 & 2)]) & G(room1.[(1 | 2)] -> room1.[3]) & G((!room1.[1 & 2 & 3])  -> X(!(room1.[1] & room1.[2] & room1.[3])))'
    spec = 'F((room2c).[(1 & 2)]) & G(!(!(room2c).[(1 | 2)]) -> (room3c & camera).[3])'
    spec2 = 'F((room1c).[1]) & G(!((!room1c).[1]) -> (room2c & camera).[2 & 3])'
    spec_test = 'G(!(!room1c.[1]) -> (room2c & camera).[(2 & 3)])'

    # spec_test = 'F(!room9.[2] & room7.[1]) & ((room7.[1] & room9.[2] & scan.[3]) U (!room9.[2] & room7.[1]))'
    # spec = spec_test

    spec_cont = 'F((room9c & beep).[2 & 3] & (room1c & pickup).[1]) & ((!pickup).[1] U (room1c & ((scan | camera) & !(scan & camera))).[2])'

    spec_cont2 = "(F(pickup & room2c).[1] & F(push & room9c).[2])" \
                 " & ((!room3c).[1 & 2 & 3] U !(!pushc.[2]))"
                # " & G(!(!room2c).[1 | 2] -> (room2c & camera).[3])"
                 # " & G(!(!room2c).[1 | 2] -> (room2c & camera).[3])"

    spec3 = "F((beep & room1c).[1] & F(pickup & room9c).[2])" \
            " & ((!room2c).[1 & 2 & 3] U ((pushc.[3]) & room6c.[3]))" \
        # " & G(!((!room2c).[(1 & 2)]) -> (room5c & camera).[3])"
            # " & G(!(!room9c).[2] -> (room9c & camera).[3])"
    # room1 = dock, room2 = loading area, room9 = storage, room6 = hallway

    spec4 = "F((beep & room1c).[1] & F(pickup & room7c).[2])" \
            " & ((!room2c).[1 & 2 & 3] U ((pushc.[3]) & room6c.[3]))" \

    filename = "buchi_intermediate"
    load_file = filename

    t0 = time.time()

    buchi_class = BuchiBindings(filename, spec4)
    buchi, init_buchi, accepting_states, all_bindings = buchi_class.construct_buchi(individual_buchi=False,
                                                                                    sync_file=filename + '.dot',
                                                                                    load_file = filename + '.dot')

    tf = time.time()
    buchi_time = tf - t0

    # gf.buchi_pretty('buchi_intermediate_s.dot', 'pretty')
    gf.buchi_pretty(filename + '.dot', 'pretty')
    # ssss

    agent_list, ap_list = pa.generate_agents_c(7)
    non_inst_props = [ap for ap in ap_list if ('room' in ap and ap[-1] == 'c') or ap in ['pickupc', 'dropoffc', 'pushc']]
    print('non inst', non_inst_props)

    # agent_list2, ap_list2 = pa.generate_agents_c(3)
    # a2 = agent_list2[2]
    # a2.id = 2
    # agent_list = [a2] + agent_list[-2:]
    # agent_list = agent_list[:4]

    # buchi_formula_dict = nx.get_edge_attributes(buchi, 'label')
    # buchi_formula_dict[('0','0','pi_2')] = buchi_formula_dict[('0','0','pi_2')] + ' & room2c.[4] & room3c.[5]'
    # nx.set_edge_attributes(buchi, buchi_formula_dict, name='label')
    # e1 = ('1','0','pi_1')
    # e2 = ('0','0','pi_2')

    # e1 = ('1', '1', 'pi_1')
    # e2 = ('1','0','pi_1')
    # buchi_upd = buchi_class.update_intermediate_buchi(non_inst_props, buchi, e1, e2)
    #
    # print('bindings', all_bindings)

    # ----- from pa.main -----
    bindings_list = all_bindings
    for ag in agent_list:
        ag.time = 0
        print(ag.id)
        t0_1 = time.time()
        ag.prod_aut, ag.buchi_sync = pa.construct_prod_aut(ag, buchi_class, bindings_list)
        # ag.prod_aut_prev = ag.prod_aut.copy()
        tf_1 = time.time()

        t0_2 = time.time()
        # prune prod aut based on which bindings the agent can do
        ag.possible_bindings = pa.powerset(bindings_list)
        # ag.possible_bindings = pa.remove_unreachable_bindings(ag.prod_aut, (ag.init_state, buchi_class.init_state),
        #                                                       list(buchi.nodes()), bindings_list)

        # print(len(ag.prod_aut.nodes()),len(ag.prod_aut.edges()))
        tf_2 = time.time()

        # ag.possible_bindings = {(1,),(2,),(3,),(1,2),(2,3),(1,3),(1,2,3)}

        # # ag.buchi_possible = ag.buchi_sync
        # t0_5 = time.time()
        # ag.buchi_possible = pa.update_sync_buchi_with_possible(ag.prod_aut, ag.buchi_sync, ag.possible_bindings)
        # tf_5 = time.time()
        #
        # print('possible: ', ag.possible_bindings)
        # print('time to construct prod aut for agent', ag.id, tf_1 - t0_1)
        # print('time to prune prod aut for agent', ag.id, tf_2 - t0_2)
        # print('time to update buchi for agent', ag.id, tf_5 - t0_5)
        #
        # ag.time += tf_1 - t0_1
        # ag.time += tf_2 - t0_2
        # ag.time += tf_5 - t0_5

    # synthesize behavior
    # sync_buchi_path, self_transitions, robot_team = pa.main(spec, agent_list, all_bindings, ap_list, buchi_class, save_files = True, sims = False)

    # for ag in agent_list:
    #     ag.time = 0
    #     print(ag.id)
    #     t0_1 = time.time()
    #     ag.prod_aut, ag.buchi_sync = pa.construct_prod_aut(ag, buchi_class, all_bindings)
    #     tf_1 = time.time()

    print('================= intermediate ===============')
    for agent in agent_list:
        agent.path_nosync = []

    # buchi_path, team_assignment, team_paths = find_teaming_assignment(agent_list, all_bindings, buchi, init_buchi)

    t0_dfs = time.time()
    buchi_path, self_transitions, team_assignment, buchi_upd, binding_constraints = assign_bindings2robots(agent_list,
                                                                                                           all_bindings,
                                                                                                           init_buchi,
                                                                                                           buchi, non_inst_props)
    tf_dfs = time.time()
    # save updated buchi as png
    pa.buchi_to_dot(buchi_upd, 'buchi_final.dot')
    gf.buchi_pretty('buchi_final.dot', 'pretty', path=(buchi_path, self_transitions))
    print('checking', list(buchi_upd.edges()))

    print('time for dfs: ', tf_dfs - t0_dfs)

    print('final ======= ')
    print(buchi_path, self_transitions)
    print('constraints:', binding_constraints)

    # keep track of intermediate buchi edges; each element in int_edges = [(z1, z_int1), (z_int1, z_int2), (z_int2, z2)]
    int_edges = []
    idx = 0
    for i in range(len(buchi_path) - 1):
        if '**' in buchi_path[i][1]:
            int_edges.append([buchi_path[i - 1], buchi_path[i], buchi_path[i + 1]])
    print('int', int_edges)

    for ag in team_assignment:
        print(ag.id, team_assignment[ag])

    team_assignment_id = {ag.id: team_assignment[ag] for ag in team_assignment}
    # team_path = {ag.id: team_assignment[ag] for ag in team_assignment}
    team_paths = {}

    for ag in team_assignment:
        ag.assigned_bindings = team_assignment[ag]

        # update prod aut with intermediate buchi edges
        # for edges in int_edges:
        #     # ag.prod_aut = update_prod_aut(ag, buchi_upd, edges)
        #     ag.prod_aut = update_prod_aut2(ag, buchi_upd, edges, all_bindings)

        print('\n ROBOT ', ag.id, '------------------------------------')
        path_nosync, cost = stb.find_shortest_path_robot(ag.prod_aut, buchi_upd, ag.init_state, buchi_path,
                                                         self_transitions, ag.assigned_bindings)
        team_paths[ag.id] = path_nosync

        print(ag.id, path_nosync)

    print('updated plans')

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
    reg_pretty_abc = {
        'room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'roomA', \
        '!room1 & room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'roomB', \
        '!room1 & !room2 & room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'roomC', \
        '!room1 & !room2 & !room3 & room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'roomD', \
        '!room1 & !room2 & !room3 & !room4 & room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'roomE', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'roomF', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'roomG', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'roomH', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'roomI', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'roomAc', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'roomBc', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'roomCc', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'roomDc', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & room5c & !room6c & !room7c & !room8c & !room9c': 'roomEc', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & room6c & !room7c & !room8c & !room9c': 'roomFc', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & room7c & !room8c & !room9c': 'roomGc', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & room8c & !room9c': 'roomHc', \
        '!room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & room9c': 'roomIc', \
        }
    reg_simple = {'room1': 'roomA', 'room1c': 'roomAc',
                  'room2': 'roomB', 'room2c': 'roomBc',
                  'room3': 'roomC', 'room3c': 'roomCc',
                  'room4': 'roomD', 'room4c': 'roomDc',
                  'room5': 'roomE', 'room5c': 'roomEc',
                  'room6': 'roomF', 'room6c': 'roomFc',
                  'room7': 'roomG', 'room7c': 'roomGc',
                  'room8': 'roomH', 'room8c': 'roomHc',
                  'room9': 'roomI', 'room9c': 'roomIc',
                }
    arm_pretty = {'pickup & !dropoff & !push & !pickupc & !dropoffc & !pushc': 'pickup', \
                  '!pickup & dropoff & !push & !pickupc & !dropoffc & !pushc': 'dropoff', \
                  '!pickup & !dropoff & !push & !pickupc & dropoffc & !pushc': 'dropoff_c', \
                  '!pickup & !dropoff & !push & pickupc & !dropoffc & !pushc': 'pickup_c', \
                  '!pickup & !dropoff & push & !pickupc & !dropoffc & !pushc': 'push', \
                  '!pickup & !dropoff & !push & !pickupc & !dropoffc & pushc': 'push_c', \
                  '!pickup & !dropoff & !push & !pickupc & !dropoffc & !pushc': 'arm_idle', \
                  '!camera': '!camera', \
                  'camera': 'camera', \
                  '!beep': '!beep', \
                  'beep': 'beep', \
                  '!scan': '!scan', \
                  'scan': 'scan', \
                  }

    reg_pretty2 = {'room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room1c', 'room2c & !room1c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room2c', 'room3c & !room1c & !room2c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room3c', 'room4c & !room1c & !room2c & !room3c & !room5c & !room6c & !room7c & !room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room4c', 'room5c & !room1c & !room2c & !room3c & !room4c & !room6c & !room7c & !room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room5c', 'room6c & !room1c & !room2c & !room3c & !room4c & !room5c & !room7c & !room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room6c', 'room7c & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room7c', 'room8c & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room8c', 'room9c & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room9c', 'room1c & room2 & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room1c & room2', '!room1c & room2c & room3 & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room2c & room3', '!room1c & room2c & !room3c & !room4c & !room5c & room6 & !room7c & !room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room2c & room6', '!room1c & !room2c & room3c & room4 & !room5c & !room6c & !room7c & !room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room3c & room4', '!room1c & !room2c & !room3c & room4c & room5 & !room6c & !room7c & !room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room4c & room5', '!room1c & !room2c & !room3c & room4c & !room5c & room6 & !room7c & !room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room4c & room6', '!room1c & !room2c & !room3c & !room4c & !room5c & room6c & !room7c & room8 & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room6c & room8', '!room1c & !room2c & !room3c & !room4c & !room5c & !room6c & room7c & room8 & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room7c & room8', '!room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & room8c & room9 & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room8c & room9', 'room1 & room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room1 & room2c', '!room1c & room2 & room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room2 & room3c', '!room1c & room2 & !room3c & !room4c & !room5c & room6c & !room7c & !room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room2 & room6c', '!room1c & !room2c & room3 & room4c & !room5c & !room6c & !room7c & !room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room3 & room4c', '!room1c & !room2c & !room3c & room4 & room5c & !room6c & !room7c & !room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room4 & room5c', '!room1c & !room2c & !room3c & room4 & !room5c & room6c & !room7c & !room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room4 & room6c', '!room1c & !room2c & !room3c & !room4c & !room5c & !room6c & room7 & room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room7 & room8c', '!room1c & !room2c & !room3c & !room4c & !room5c & room6 & !room7c & room8c & !room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room6 & room8c', '!room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & room8 & room9c & !room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9': 'room8 & room9c'}



    # pretty_dict = {**reg_pretty, **reg_simple, **arm_pretty}
    pretty_dict = {**reg_pretty2, **arm_pretty}


    # for p in agent.path_sync:
    #     a_state, b_state, pi_sync = p
    #     a_list = []
    #     for a in a_state:
    #         a_list.append(pretty_dict[a])
    #     print(", ".join('"'+x+'"' for x in a_list) + ';')

    # flatten nested lists
    def flatten(items, seqtypes=(list)):
        try:
            for i, x in enumerate(items):
                while isinstance(x, seqtypes):
                    items[i:i + 1] = x
                    x = items[i]
        except IndexError:
            pass
        return items


    print('non sync behavior ------- ')
    team_path_id = {}
    for na in team_assignment:
        path = team_paths[na.id]
        print('team', na.id)
        # include initial state
        path = [(na.init_state, init_buchi, self_transitions[init_buchi])] + flatten(path)
        pretty_path = []
        for z in range(len(path)):
            # e1, e2, pi_sync = sync_buchi_path[z]
            # buchi_formula = buchi.get_edge_data(e1,e2)[pi_sync]['label']
            # print('buchi path')
            # for p in path[z]:
            p = path[z]
            a_state, b_state, pi_sync = p
            a_list = []
            for a in a_state:
                if 'room' in a:
                    a_new = a.replace('room1', 'dock')
                    a_new = a_new.replace('room2', 'roomB')
                    a_new = a_new.replace('room3', 'roomC')
                    a_new = a_new.replace('room4', 'roomD')
                    a_new = a_new.replace('room5', 'roomE')
                    a_new = a_new.replace('room6', 'hall')
                    a_new = a_new.replace('room7', 'storage')
                    a_new = ' & '.join(prop for prop in a_new.split(' & ') if prop[0] != '!')
                    a_list.append(a_new)
                elif a in pretty_dict:
                    a_list.append(pretty_dict[a])
                else:
                    a_list.append(a)
            # pretty_state = ", ".join('"'+x+'"' for x in a_list) + ';', b_state, pi_sync
            pretty_state = (tuple(a_list), b_state, pi_sync)
            print(pretty_state)
            pretty_path.append(pretty_state)

        team_path_id[na.id] = pretty_path

        # store necessary bindings for each transition in path
        buchi_bindings = {}
        for z1, z2, pi_sync in buchi_path:
            formula = buchi_upd[z1][z2][pi_sync]['label']
            buchi_bindings[(z1, z2, pi_sync)] = set(method2.extract_bindings(formula))

    # # save variables as pickle
    import pickle

    # team_assignment_id2 = team_assignment_id.copy()
    #
    # # only save agents 1,2,4
    # for ag in team_assignment_id2:
    #     if ag == 3:
    #         team_assignment_id.pop(ag)
    #         team_path_id.pop(ag)
    #
    # # Saving the objects:
    # with open('inputs_ex1.pkl', 'wb') as f:
    #     pickle.dump([team_assignment_id, team_path_id, buchi_path, buchi_bindings], f)

    # Getting back the objects:
    with open('inputs_ex1.pkl', 'rb') as f:
        team_assignment, team_path, buchi_path, buchi_bindings = pickle.load(f)

    print(team_assignment)
    print(team_path)
    print(buchi_path)
    print(buchi_bindings)