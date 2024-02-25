#!/usr/bin/env python

# construct synchronization buchi structure

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
import time

def load_buchi(filename,remove_init=True):
    # convert buchi to nx
    f_nx = (nx.drawing.nx_pydot.read_dot(filename))

    with open(filename, 'r') as file:
        f = file.read()
    
    # label accepting states
    accepting_states = bf.find_accepting_states(f)
    accepting_labels = {k:'F' for k in accepting_states}
    nx.set_node_attributes(f_nx, accepting_labels, 'accepting')

    # find initial state
    init_buchi = list(f_nx.successors('I'))[0]

    f_temp = f_nx.copy()
    # init_buchi = list(f_temp.successors('I'))[0]        # buchi should only have one init state
    # print(accepting_states, init_buchi)
    if remove_init:
        f_temp.remove_node('I')

    return f_temp, accepting_states, init_buchi


def create_sync_struct(formula_w_binding, buchi):
    S = nx.DiGraph()
    S.add_node('I')
    binding_set = extract_bindings(formula_w_binding)

    '''
    for each transition in the buchi
        for each OR, branch out in graph
            for each AP, expand binding. Branch out for each required binding, and make a note of which need to be synced.
        
    '''

    return

def output_ap_list(spec):
    ap_list = list(set(re.findall('[^a-z]([a-z].*?)[^a-z0-9_]', spec)))
    ap_list.sort(key=len, reverse=True)
    return ap_list

def expand_spec_ap(spec):
    '''
    expand spec so all binding formulas are on APs rather than on LTL formulas

    from <LTL formula>.[buchi formula] -> <LTL formula>.[binding formula] &| <LTL formula>.[binding formula] .... 
    '''
    formula_list = re.findall('[A-Z]+(.*?\])', spec)
    formula_list.sort(key=len, reverse=True)

    formula_replaced = spec
    for formula in formula_list:
        formula_spec = formula.split('.')[0]
        binding = formula.split('.')[1]
        ap_list = output_ap_list(formula_spec)

        if ap_list == []:
            continue
        
        spec_replaced = formula_spec
        for ap in ap_list:
            print('ap',ap, ap+'.'+binding)
            spec_replaced = spec_replaced.replace(ap, ap+'.'+binding)
        
        formula_replaced = formula_replaced.replace(formula, spec_replaced)

    return formula_replaced

def expand_spec_binding(spec, dict_form_id):
    '''
    from ap.[buchi formula] -> ap.[one binding] &| ap.[one binding] .... 
    '''

    # store an id number for each ap with binding and its tree, e.g. {id: [room.[3 | 4], Tree(3 | 4)]}
    spec_replaced = spec
    dict_num_form = {}
    counter = 0
    for form in dict_form_id.keys():
        dict_num_form[counter] = [form]     # ap
        dict_num_form[counter].append(psi_parser.Tree.build(re.findall('\.\[(.*?)\]', form)[0]))
        spec_replaced = spec_replaced.replace(form, str(counter))
        counter += 1

    # construct expansion
    spec_exp = spec
    for k, v in dict_num_form.items():
        form = v[0]
        tree = v[1]
        ap = form.split('.')[0]

        # find aps with negation
        for idx in [m.start() for m in re.finditer(ap, spec_exp)]:
            if spec_exp[idx-1] == '!':
                spec_exp = spec_exp.replace('!' + form, tree.evaluate('!'+ap, tree.root))
                break

        spec_exp = spec_exp.replace(form, tree.evaluate(ap, tree.root))
    return spec_exp


def construct_buchi_edges(buchi):
    '''
    separate all OR parts of each edge formula in Buchi into separate edges
    '''
    buchi_new = buchi.copy()
    for edge in buchi.edges():
        formula = buchi.get_edge_data(edge[0], edge[1])[0]['label'].strip('"')
        formula_or = formula.split(' | ')
        if len(formula_or) > 1:
            buchi_new.remove_edge(edge[0],edge[1])
            for subformula in formula_or:
                buchi_new.add_edge(edge[0],edge[1], label=subformula)

    return buchi_new

def construct_binding_buchis(spec, buchi):
    '''
    given centralized buchi (without edge formulas being separated by ORs), construct a buchi for each binding with sync APs
    '''
    binding_list_all = method2.extract_bindings(spec)
    buchi_dict = {}     # {binding #: buchi}

    for binding in binding_list_all:
        buchi_dict[int(binding)] = buchi.copy()

    for edge in buchi.edges():
        pi_sync = ord('a')

        # if buchi edge is 1, assign 1 to all corresponding binding buchi edges
        if buchi.get_edge_data(edge[0], edge[1])[0]['label'].strip('"') == '1':
            for binding, b in buchi_dict.items():
                b.remove_edge(edge[0],edge[1])
                b.add_edge(edge[0],edge[1], label='1 & ' + '\u03C0_' + chr(pi_sync))
                pi_sync = pi_sync+1
            continue

        # else, split edges by OR formula
        for binding, b in buchi_dict.items():         
            b.remove_edge(edge[0],edge[1])

        # split edges by OR formula
        edge_formula = buchi.get_edge_data(edge[0], edge[1])[0]['label'].strip('"')
        formula_or = edge_formula.split(' | ')
        
        for formula in formula_or:
            # find aps and !aps associated with each binding. store as string 'ap & ap & !ap ...'
            binding_ap_dict = construct_binding_ap_dict(formula, buchi_dict.keys())

            for binding, ap_formula in binding_ap_dict.items():
                b = buchi_dict[binding]
                
                b.add_edge(edge[0],edge[1], label=ap_formula + ' & ' + '\u03C0_' + chr(pi_sync))

            pi_sync = pi_sync+1

    return buchi_dict


def construct_binding_ap_dict(spec, binding_list):
    '''
    given formula structured as ap.[binding] & ap.[binding] & ...
    output {binding: 'ap & !ap & ...']'}
    '''
    binding_ap_dict = {}
    for b in binding_list:
        binding_ap_dict[b] = '1' 

    spec_copy = re.sub('[())]', '', spec)
    pi_psi_list = spec_copy.split(' & ')
    for p in pi_psi_list:    # p = 'room1.[2]'
        ap = p.split('.')[0]
        binding = int(p.split('.')[1][1:-1])    # remove brackets for binding

        if binding_ap_dict[binding] != '1':
            binding_ap_dict[binding] += ' & ' + ap
        else:
            binding_ap_dict[binding] = ap

    return binding_ap_dict

def buchi_to_dot(buchi, filename):
    write_dot(buchi, filename) # nx built-in function

    f = open(filename,'r')
    filedata = f.read()
    f.close()

    newdata = filedata.replace(r'\"circle\"', 'circle')

    f = open(filename,'w')
    f.write(newdata)
    f.close()

def synthesize_path(agents, buchi_dict):
    '''
    find A path (not necessarily optimal)

    generate a tree of possible paths until accepting state is reached (or an accepting cycle)

    for each edge - generate possible paths
    
    how to know if binding isn't possible? if no possible next states and accepting state hasn't yet been reached

    if transition is 1 - no need to sync with other robots, can move on to next transition


    each robot initializes a tree
    for each buchi node:
        for each of its neighbors:
            for each edge:
                for each robot:
                    for each initial state (leaf from robot tree):
                        synthesize path to neighbor. include path for accepting cycle
                        if robot cannot find path (no possible way to get to neighbor):
                            remove edge for all buchis
                            remove part of tree leading up to this leaf that doesn't contain any other leaves
                        else:
                            add path to robot's tree
            if neighbor == accepting state:
                break

    '''

def construct_buchi(filename, spec, individual_buchi=False):
    # first, move the binding formulas to each AP
    ap_list = output_ap_list(spec)

    spec = expand_spec_ap(spec)

    # then expand binding formulas  
    # dict_form_id, dict_form_aps = method2.create_ap_ids(spec, ap_list)
    method2.spec_to_dot(filename, spec, dict_form_id)

    buchi, accepting_states, init_buchi = load_buchi(filename)


    dict_form_id, dict_form_aps = method2.create_ap_ids(spec_exp, ap_list)
    spec_exp = expand_spec_binding(spec, dict_form_id)

    dict_form_id, dict_form_aps = method2.create_ap_ids(spec_exp, ap_list)

    method2.spec_to_dot(filename, spec_exp, dict_form_id)

    # separate out the edges for each OR part of the transition formulas
    buchi, accepting_states, init_buchi = load_buchi(filename)
    buchi_new = construct_buchi_edges(buchi)
    write_dot(buchi_new, filename)
    

    # if we want the buchis of each binding
    if individual_buchi:
        buchi_dict = construct_binding_buchis(spec_exp, buchi)

        # save buchi files
        for binding, b in buchi_dict.items():
            buchi_to_dot(b,'buchi' + str(binding) + '.dot')

        return buchi_new, init_buchi, buchi_dict

    return buchi_new, init_buchi


 
if __name__ == '__main__':
    filename = 'synth_buchi.dot'
    spec = "(!m.[1] & !room.[1]) | (a.[3 | 4] & p.[3 & 2])"    # something's wrong when m replaces a
    spec = "GF(!p.[1] & room1.[1 & 2]) & (!p.[1] U (a.[1 | 3]))"    # something's wrong when m replaces a
    spec = "GF(scan.[1] & room1.[1 & 2]) & (!scan.[1] U (camera.[1 | 3]))"
    spec = "GF(not_scan.[1 & 3] & room1.[1 & 2]) & (not_scan.[1] U (camera.[1 | 3]))"
    spec = "GF(not_scan.[(1 & 3)] & room1.[(1 & 2)]) & (not_scan.[1] U camera.[(1 | 3)])"
    spec = "F(scan.[(1 & 3)])"
    spec = "GF(room1 & scan).[(1 & 3)] & (!scan.[1] U camera.[(2 | 3)])"


    construct_buchi('buchi_all.dot', spec, individual_buchi=False)
   


    # to create robot's product automaton
        # pi_sync becomes true only when rest of formula is true


    # maybe for now, let's just find A path (not necessarily the optimal one)

    # generate a tree of possible paths until accepting state is reached (or an accepting cycle)

    # for each edge - generate possible paths

    # how to know if binding isn't possible? if no possible next states and accepting state hasn't yet been reached




    # -----------------

    # brute force, exhaustive method to find optimal path (ASSUMING every node has self-transition)

    # for each neighboring buchi node (neighbor):
        # for each edge (node, neighbor):
            # find shortest paht from node to neighbor

                # if neighbor is an accepting state
                    # compare path length to other paths that have reached an accepting state. If yes, replace the optimal path with this one

    # find shortest path from buchi node 1 to buchi node 2




