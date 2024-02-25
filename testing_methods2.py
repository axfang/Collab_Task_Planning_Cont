#!/usr/bin/env python

# Synthesizing path using method 2: treat pi^psi as its own proposition

# syntax: pi**(psi) ex: F a**(1) & G b**(1 | 2)

# given: APs for both pi and psi
# make dict1 where for each Ap_pi in formula, pi**(psi): [pi, psi]
# make dict_form_id where for each Ap_pi in formula, pi**(psi): pi#

# replace formula with dict_form_id values
# create buchi

# importing sys
import sys
  
# adding Folder_2 to the system path
sys.path.insert(0, '../../Mission_Switching/scripts')
sys.path.insert(0, '/Users/amy/usr/lib/python3.7/site-packages/')


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





# method 1: AP.[3 & (2 | !(1 & 4)] becomes AP.[3] & (AP.[2] | (!AP.[1] | !AP.[4]))

# for each AP
#     while AP's [...] is not just one number
#         take up to the first & or | and split it ( this doesnt work)



def extract_bindings(formula):
    # find total number of bindings in a formula
    binding_formulas = ''.join(set(re.findall('\[([0-9].*?)\]', formula)))
    b = set(re.findall(r'\d', binding_formulas))
    return list(b)

# print(list(fir.powerset(extract_bindings(test))))

def assign_binding_values(possible_bindings, binding_combo):
    ''' generate dictionary of truth values for a combination of bindings. 
    possible_bindings is the set of all the bindings in the formula (from extract_bindings)
    '''
    binding_dict = {}
    # generate every combination
    for b in possible_bindings:
        if b in binding_combo:
            binding_dict[b] = 'True'
        # else:
        #     binding_dict[b] = 'False'
    return binding_dict


def create_phipsi_ids(spec):
    '''
    make dict {phi.[binding_formula]: ap<id num>}
    ex: {'(reg2 & ((thermal | visual) & !(thermal & visual))).[1 | 2]': 'a_1', '(weed & reg1 & reg2).[1]': 'b_2', '!weed.[1]': 'c_3', '(reg2 & soil & light).[2 & 3]': 'd_4'}

    '''

    def parenthetic_contents(string):
        """Generate parenthesized contents in string as pairs (level, contents)."""
        stack = []
        for i, c in enumerate(string):
            if c == '(':
                stack.append(i)
            elif c == ')' and stack:
                start = stack.pop()
                yield (len(stack), string[start + 1: i])

    def binding_subformula(string):
        """Generate parenthesized contents in string as pairs (level, contents)."""
        stack = []
        for i, c in enumerate(string):
            if c == '(' and not stack:
                stack.append(i)
            elif c == ']' and stack:
                start = stack.pop()
                yield string[start: i + 2]

    dict_form_aps = {}
    dict_form_id = {}

    z = list(parenthetic_contents(spec))

    formula_list_ap, formula_list_binding = [], []
    subformulas = set()
    for i, s in z:
        if '.' not in s:
            # print(i,s)
            sub = [tuple for tuple in z if (int(tuple[0]) < int(i)) and (s in tuple[1]) and ('.' in tuple[1])]
            # print(sub)
            binding_form = list(binding_subformula(sub[0][1]))
            for b in range(len(binding_form)):
                if binding_form[b][0] == '(' and binding_form[b][-1] == ')':
                    binding_form[b] = binding_form[b][1:-1]
            subformulas = subformulas.union(set(binding_form))
    for form in subformulas:
        form = form.strip()
        phi, psi = form.split('.')
        formula_list_ap.append(phi[1:-1])
        formula_list_binding.append(psi[1:-1])

    temp_ap = ord('a')
    temp_psi = 1
    for pi_idx in range(len(formula_list_ap)):
        # pi, psi = subformula.split(".")
        pi = formula_list_ap[pi_idx]
        # if '&' in pi or '|' in pi:
        pi = '(' + pi + ')'
        psi = formula_list_binding[pi_idx]
        subformula = pi + '.[' + psi + ']'

        dict_form_id[subformula] = chr(temp_ap) + '_' + str(temp_psi)

        temp_ap += 1
        temp_psi += 1

    return dict_form_id

def create_ap_ids(spec, neg=False):
    '''
    make dict {ap.[binding_formula]: ap<id num>}
    ex: {'m.[1]': 'm_1', 'room.[1 | 2]': 'room_1'}

    dict_form_aps = {'1 & 2': ['room1', 'carry', 'room2'], '!(1 | 2)': ['scan'], '': ['carry', 'room4']}
    '''
    # ap_pi = ['m','room','c','d']   # not a letter before it
    # ap_pi = re.findall('[^a-z]([a-z].*?)\.', spec)

    dict_form_aps = {}
    dict_form_id = {}
    spec_temp = spec

    formula_list_ap = []

    formula_list_binding_set = set(re.findall('\.\[(.*?)\]', spec))
    formula_list_binding = []
    # print('bindings', formula_list_binding_set)

    # formula_list.sort(key=len, reverse=True)
    count = 0
    for i in formula_list_binding_set:
        # print(i, spec_temp)
        splitting = spec_temp.split('['+i+']')[0:-1]
        # print('split',i, splitting)

        
        # print('before', spec_temp.split('['+i+']'))
        for s in splitting:
            if any(x.isalpha() for x in s):
                # phi = re.findall('\((.*?)[\)\.]', s)
                # phi = re.findall('[^\[\(\s][a-z0-9]+', s)
                phi = re.findall('([\!a-z][\!a-z\&\|\s]+.*?[\.\)])', s)
                # print('s',s, phi)
                # phi = re.findall('[a-z]+',s)
                # print('phi', phi)
                if len(phi) == 0:
                    phi = re.findall('[\!a-z0-9]+', s)
                phi = phi[-1][:-1]
                
                # phi = re.findall('\((.*?)[\)\.]', s)[-1]
                # if phi[0] == '!' or phi[0] == '(':
                if phi[0] == '(':
                    formula_list_ap.append(phi[1:])
                else:
                    formula_list_ap.append(phi)
                formula_list_binding.append(i)


    temp_ap = ord('a')
    temp_psi = 1
    for pi_idx in range(len(formula_list_ap)):
        # pi, psi = subformula.split(".")
        pi = formula_list_ap[pi_idx]
        if '&' in pi or '|' in pi:
            pi = '(' + pi + ')'
        psi = formula_list_binding[pi_idx]
        subformula = pi + '.[' + psi + ']'

        if neg:
            dict_form_id['('+subformula+')'] = chr(temp_ap) + '_' + str(temp_psi)
        else:
            dict_form_id[subformula] = chr(temp_ap) + '_' + str(temp_psi)
        temp_ap += 1
        temp_psi +=1

    return dict_form_id


def replace_spec_ids(spec, dict_form_id):
    ''' replace all APs with binding formulas to be APs with their ids
    '''
    spec_replaced = spec
    keys = list(dict_form_id.keys())
    keys.sort(key=len, reverse=True)
    for k in keys:
        # remove any parentheses
        replaced = re.sub("\(|\)", "",k)
        spec_replaced = spec_replaced.replace(replaced, dict_form_id[k])
    return spec_replaced

def spec_to_dot(filename, spec, dict_form_id):
    # method 2: replace to generate dot file using spot
    print('saving dot file...')

    # flip the keys and values => pi#: formula
    dict_id_form = dict((v,k) for k,v in dict_form_id.items())

    spec_replaced = replace_spec_ids(spec, dict_form_id)
    print('replaced', spec_replaced)

    # create buchi, then replace
    import inspect
    print('file',inspect.getfile(spot))

    f = spot.translate(spec_replaced, 'BA', 'sbacc', 'unambig')

    file = open(filename, "w")
    # file.write(f.to_str('dot'))

    file_str = f.to_str('dot')

    for line in file_str.splitlines():
        if ' -> ' in line and 'label=' in line and 'label="1"' not in line:
            line_formula = line.split('label=')[1][1:-2]
            aps = line_formula.split(' ')
            aps2 = [re.sub('[()!&|]', '', string) for string in aps]
            # aps2 = [''.join(filter(str.isalnum, string)) for string in aps]   # remove special characters
            # print(aps2)
            aps2 = list(filter(('').__ne__, aps2))
            aps2 = [value[1:] if value[0] == '!' else value for value in aps2]
            for a in aps2:
                line = line.replace(a, dict_id_form[a])

        file.write(line+'\n')
    file.close()

# create_dot_file(specification, a)

def plot_agent(caps_list):
    P = caps_list[0]
    for i in range(len(caps_list)-1):
        P = nx.tensor_product(P,caps_list[i+1])
    # P.add_node(((not_agent,),))
    # P.add_weighted_edges_from((a,b,0) for a,b in itertools.product(P.nodes(), ((not_agent,),)))
    agent = fr.flatten_nodes(P)
    fr.flatten_edges(agent)
    print('size: ', agent.number_of_nodes(), agent.number_of_edges())

    # labels = fr.make_pretty_labels(aut, ap_list, {**reg_pretty, **arm_pretty})

    pos = nx.nx_pydot.graphviz_layout(agent, prog='neato')
    nx.draw(agent, pos=pos,with_labels=True)
    # nx.draw(agent, pos=pos,labels={**reg_pretty, **arm_pretty})
    plt.show()
# agent_list = [ag1, ag2, ag3]
# --------------------------------------------------------------------------------------------

# dict_form_id = create_ap_ids(test)
# create_dot_file(spec, dict_form_id)
# print(re.sub('[3 & 2]', '[3 & True]', '(!m.[1] & !room.[1]) | (!m2.[1] & p.[3 & 2])'))
# print('(!m.[1] & !room.[1]) | (!m2.[1] & p.[3 & 2])'.replace('[3 & 2]', '[3 & True]'))

def find_binding_formula_value(formula, binding_dict):
    '''
    given a formula (with bindings) and which individual bindings are true, determine which binding formulas are true
    INPUT: 
    binding_dict = {'1': 'True', '3': 'False', '2': 'False'}
    formula = "(!m.[1] & !room.[1]) | (!m.[1] & p.[3 & 2])"
    OUTPUT:
    replace_binding_formula_dict = {'[1]': True, '[3 & 2]': False}
    '''
    replace_binding_formula_dict = {}
    all_binding_formulas = find_all_binding_formulas(formula)
    # all_binding_replaced = all_binding_formulas.copy()
    # for b in binding_dict:         # for each binding AP
    #     for i in range(len(all_binding_replaced)):      # for each binding formula
    #         all_binding_replaced[i] = re.sub(b, binding_dict[b], all_binding_replaced[i])    # [3 & 2] => [True & False]
    #         binding_values = re.sub('[\[\]]', '', all_binding_replaced[i]) # remove brackets
    #         truth_value = bool_parsing.nested_bool_eval(binding_values,False)
    #         # assign each binding formula with a true or false depending on binding assignment
    #         replace_binding_formula_dict[all_binding_formulas[i]] = truth_value

    for formula in all_binding_formulas:      # for each binding formula
        formula_tab = formula.maketrans(binding_dict)
        formula_truth = formula.translate(formula_tab)     # replace formula with truth values of eaching binding AP
        print('dict', binding_dict)
        truth_value = bool_parsing.nested_bool_eval(formula_truth[1:-1],True)     # evaluate formula of Ts and Fs
        print('formula', formula, formula_truth, truth_value)
        replace_binding_formula_dict[formula] = truth_value
    return replace_binding_formula_dict

def find_all_binding_formulas(formula_w_binding):
    ''' outputs list of all the binding formulas in the specification
    '''
    return ['[' + string + ']' for string in set(re.findall('\.\[(.*?)\]', formula_w_binding))]

def is_binding_combo_valid(binding_combo, binding_set, formula_w_binding, ap_w_bindings, ap_dict_agent):

    '''
    OUTPUTS
    True/False              if binding formula is True or False
    formula_no_binding      remove all bindings [...] from formula and replace any "AP.[False]" with "False"
                            Ex: (!m & !room) | (!m & False)
    '''
    binding_dict = assign_binding_values(binding_set, binding_combo)      # assign truth values to bindings
    replace_binding_formula_dict = find_binding_formula_value(formula_w_binding, binding_dict)
            
    print(replace_binding_formula_dict)
    # for each valid binding formula, get associated APs
    # remove bindings from formula_w_binding and APs that conflict
    # dict_form_id, dict_form_aps = create_ap_ids(formula_w_binding)
    # ap_w_bindings = list(dict_form_id.keys())
    ap_w_bindings.sort(key=lambda s: -len(re.findall('([a-z].*?)\.', s)[0]))
    print(ap_w_bindings)

    formula_no_binding = formula_w_binding
    for ap in ap_w_bindings:
        binding = '['+re.findall('\.\[(.*?)\]', ap)[0]+']'
        re_ap = re.sub("([^a-z1-9])", r"\\\1", ap)
        if not replace_binding_formula_dict[binding]:   # binding formula conficts
            if ap in formula_no_binding:     # if ap in formula_no_binding
                formula_no_binding= re.sub('[\(\!\s)]' + re_ap, 'False', formula_no_binding)
            # if '!'+ap in formula_no_binding:
            #     formula_no_binding=formula_no_binding.replace('!'+ap,'False')
            # if '(' + ap in formula_no_binding:
            #     formula_no_binding=formula_no_binding.replace(' ' + ap,'False')   
        else:
            # formula_no_binding=formula_no_binding.replace('.'+binding,'')
            ap_wo_bindings = ap.split('.')[0]
            
            formula_no_binding=re.sub('[\!]' + re_ap, ap_dict_agent[ap_wo_bindings], formula_no_binding)
            formula_no_binding=re.sub('[\s]' + re_ap, ' ' +ap_dict_agent[ap_wo_bindings], formula_no_binding)
            formula_no_binding=re.sub('[\()]' + re_ap, '(' + ap_dict_agent[ap_wo_bindings], formula_no_binding)

            # formula_no_binding=formula_no_binding.replace(ap, ap_dict_agent[ap_wo_bindings])
    
    # if not bool_parsing.nested_bool_eval(formula_no_binding,True):      # the case where all APs are False, don't add node
    # else: # check formula against agent APs
        # formula_no_binding_bool = re.sub('[\.\[](.*?)\]', '', formula_no_binding)
    return bool_parsing.nested_bool_eval(formula_no_binding,True), formula_no_binding

# --------- code that will go into generate prod


def all_valid_binding_combos(formula_w_binding, ap_dict_agent):
    ''' Output list of all binding combos that the agent can satisfy given the specification
    '''
    # ap_dict_agent = {'m': 'False', 'room': 'False'}
    all_binding_formulas = find_all_binding_formulas(formula_w_binding)

    binding_set = extract_bindings(formula_w_binding)
    binding_combos_all = fir.powerset(binding_set)
    binding_combos_valid=[]


    binding_combos_all = [('2', '3')]

    for combo in binding_combos_all:        # for each binding combo
        dict_form_id, dict_form_aps = create_ap_ids(formula_w_binding)
        is_valid, formula_no_binding = is_binding_combo_valid(combo, binding_set, formula_w_binding, list(dict_form_id.keys()), ap_dict_agent)
        print('here',combo, is_valid, formula_no_binding)
        if is_valid:       # the case where not all APs are False - agent can perform some part of formula
            formula_no_binding_valid = formula_no_binding.replace('False', 'True')     # whatever APs the agent can't do will be set to True
            
            # check if base APs conflict. if not, add to binding_combos_valid. change added_node to true and add node
            # for b in ap_dict_agent:
            #     # instead of subbing all APs, only sub APs with binding************
            #     # formula_no_binding_valid = re.sub(b + , ap_dict_agent[b], formula_no_binding_valid)
            #     formula_no_binding_valid = re.sub(b, ap_dict_agent[b], formula_no_binding_valid)
            not_conflict = bool_parsing.nested_bool_eval(formula_no_binding_valid,False)
            print('after replacing: ', formula_no_binding_valid, not_conflict)
            if not_conflict:
                binding_combos_valid.append(combo)
                # add_node = True
    return binding_combos_valid


if __name__ == '__main__':
   

    specification = 'F(m.[1]) | G(room.[1 | 2]) & G(!p.[1 & 2]) & F(m.[3 & !2])'
    specification = 'F(room1.[1 & 2] & scan.[!(1 | 2)] & carry.[1 & 2] U (room2.[1 & 2] & X !carry.[1 & 2])) & G(carry.[] -> !room4.[])'


    # agent 3
    loc3, ap_loc3 = af.create_mobility(1)
    arm3, ap_arm3 = af.create_carry(2)
    scan3, ap_scan3 = af.create_scan(1.2)
    caps3 = [loc3, arm3,scan3] #, downlink2, batt2, track2]
    ap3 = [ap_loc3, ap_arm3,ap_scan3]

    ag3 = Agent(ap3, caps3, 3)

    caps_name3 = ['loc', 'arm', 'scan']
    ag3.init_state = ('!room1 & !room2 & !room3 & !room4 & room5',  '!carry & !lever', '!scan')

    # ag3.graph, ag3.ap_list = ag3.create_agent_c()      # why doesn't this work??


    # --------- PLOTTING ---------------
    reg_pretty = {'room1 & !room2 & !room3 & !room4 & !room5': 'room1', \
                          '!room1 & room2 & !room3 & !room4 & !room5': 'room2', \
                          '!room1 & !room2 & room3 & !room4 & !room5': 'room3', \
                          '!room1 & !room2 & !room3 & room4 & !room5': 'room4', \
                          '!room1 & !room2 & !room3 & !room4 & room5': 'room5'
                          }
    arm_pretty = { 
                    'carry & !lever': 'carry', \
                    '!carry & lever': 'lever', \
                    '!carry & !lever': 'arm_idle' , \
                      }


    test = "(!m.[1] & !room.[1]) | (!m.[1] | p.[3 & 2])"   #  {'3': 'True', '1': 'False', '2': 'False'}

    # current issue: with this config, agent can still have 1 as a valid binding, as long as p.[3 & 2] is true from other robots
    # how to know what to sync with?
    ap_dict_agent = {'m': 'True', 'room': 'False'}

    # other issue:
    test = "(!m.[1] & !room.[1]) | (m.[1] & p.[3 & 2])"
    ap_dict_agent = {'m': 'False', 'room': 'True', 'p':'True'}
    # NEEDS TO BE FIXED: ------------
    # 2 is not out being outputted as a valid binding, since 3 & 2 is not true


    binding_combos_valid = all_valid_binding_combos(test, ap_dict_agent)
    print(binding_combos_valid)

    if binding_combos_valid != []: # if there is a valid combo, add node to buchi and list of valid combos
        print('adding node')
           
        # else: don't add to buchi


    asdf
        # replace_binding_formula_dict[binding_replaced] = 

            # print(b, binding_replaced, binding_dict, all_binding_formulas[i])
        # print('replaced', binding_replaced)
        # formula_w_binding2 = formula_w_binding.replace(all_binding_formulas[i], binding_replaced)
        # print(combo, all_binding_formulas[i], binding_replaced, formula_w_binding, '\n',formula_w_binding2)
        
    # for ap in ap_w_bindings:
    #     check_conflict_binding = bool_parsing.nested_bool_eval(binding_formula,False)

    # if check_conflict_binding:   # there are no conflicts in the bindings
    #     ap_dict[]

# after the code works, replace binding w psi



# ------------
'''
replace each binding formula with bool_parsing.nested_bool_eval
if all are true: no conflict
'''




# how to determine which bindings given binding formula

