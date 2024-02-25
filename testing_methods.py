#!/usr/bin/env python

# Synthesizing path using method 2: treat pi^psi as its own proposition

# syntax: pi**(psi) ex: F a**(1) & G b**(1 | 2)

# given: APs for both pi and psi
# make dict1 where for each Ap_pi in formula, pi**(psi): [pi, psi]
# make dict_form_id where for each Ap_pi in formula, pi**(psi): pi#

# replace formula with dict_form_id values
# create buchi

import spot
import graphviz
import networkx as nx
import matplotlib.pyplot as plt
from networkx.drawing.nx_agraph import write_dot
import numpy as np
import re
import psi_parser

# spec = 'F(m.[1]) & G(room.[1 | 2]) & G(!p.[1 & 2]) & F(m.[!(3 & 2)])'

# method 1: AP.[3 & (2 | !(1 & 4)] becomes AP.[3] & (AP.[2] | (!AP.[1] | !AP.[4]))

# spec = '((3|5)&(2|!((1|3)&!(2|4))))'
# spec = '((!1|3)&!(2|4))'   # needs () around the whole formula

def save_buchi(spec, filename):
    f = spot.translate(spec_exp, 'BA', 'sbacc', 'unambig')
    file = open(filename, "w")
    file.write(f.to_str('dot'))
    file.close()
    return

def expand(spec,pi):
    ''' !(a&b) -> (!a|!b)
        !(a|b) -> (!a&!b)
    '''
    
    tree=psi_parser.Tree.build(spec)
    gr = tree.evaluate(pi)
    # print('eval',gr)
    return gr

def findLeafs(spec,pi):
    ''' !(a&b) -> (!a|!b)
        !(a|b) -> (!a&!b)
    '''
    
    tree=psi_parser.Tree.build(spec)
    leaf_list = tree.outputLeafNodes()
    # print(gr)
    print(spec)

    print('leafs',leaf_list)

    print('parent', tree.findParent(leaf_list[0]))
    asf
    
    # print('eval',gr)
    return gr

 # METHOD 2
# make dict1 where for each Ap_pi in formula, pi**(psi): [pi, psi]
# ap_pi = ['m','room','c','d']   # not a letter before it


def generate_spec(spec):
    '''
    expand spec and convert into syntax for spot using method 1
    '''
    ap_pi = list(set(re.findall('[^a-z]([a-z].*?)\.', spec)))
    ap_pi.sort(key=len, reverse=True)

    dict1 = {}
    dict_form_id = {}
    spec_temp = spec.replace(" ", "")
    spec_exp = spec_temp

    for pi in ap_pi:
        # print('pi', pi, spec_temp)
        psi = re.findall('[^a-z]'+pi+ '\.\[(.*?)\]', spec_temp)[0]
        # print('psi',psi)
        if not str.isdecimal(psi):    # if psi needs expanding (not just a number)
            psi_new = psi
            if psi[0] != '!':
                psi_new = '('+psi+')'
            subformula = expand(psi_new, pi)
            spec_exp = spec_exp.replace(pi + '.[' + psi + ']',subformula)
     

        # dict1[subformula] = [pi, psi]

        # # start dict_form_id out as pi**(psi): [pi, num]
        # pi_val = [item[0] for item in dict_form_id.values()]
        # num_val = [item[1] for item in dict_form_id.values()]
        # if pi in pi_val:
        #     idx = pi_val.index(pi)
        #     dict_form_id[subformula]= [pi, num_val[idx]+1]
        # else:
        #     dict_form_id[subformula]= [pi, 1]
        spec_temp = spec_temp.replace(pi + '.[' + psi + ']','')

    # replace brackets with '_' to pass into spot
    # print('fina', spec_exp)
    spec_exp = spec_exp.replace('.[','_')
    spec_exp = spec_exp.replace(']','')
    return spec_exp

def generate_sync_aps(spec):
    '''
    expand spec and convert into syntax for spot using method 1
    '''
    ap_pi = list(set(re.findall('[^a-z]([a-z].*?)\.', spec)))
    ap_pi.sort(key=len, reverse=True)

    dict1 = {}
    dict_form_id = {}
    spec_temp = spec.replace(" ", "")
    spec_exp = spec_temp
    spec_list = []

    for pi in ap_pi:
        # print('pi', pi, spec_temp)
        psi = re.findall('[^a-z]'+pi+ '\.\[(.*?)\]', spec_temp)[0]
        # print('psi',psi)
        if not str.isdecimal(psi):    # if psi needs expanding (not just a number)
            psi_new = psi
            if psi[0] != '!':
                psi_new = '('+psi+')'
            subformula = findLeafs(psi_new, pi)
            print('sub',subformula)
            print(psi_new, pi)
            spec_exp = spec_exp.replace(pi + '.[' + psi + ']',subformula)
     

        # dict1[subformula] = [pi, psi]

        # # start dict_form_id out as pi**(psi): [pi, num]
        # pi_val = [item[0] for item in dict_form_id.values()]
        # num_val = [item[1] for item in dict_form_id.values()]
        # if pi in pi_val:
        #     idx = pi_val.index(pi)
        #     dict_form_id[subformula]= [pi, num_val[idx]+1]
        # else:
        #     dict_form_id[subformula]= [pi, 1]
        spec_temp = spec_temp.replace(pi + '.[' + psi + ']','')

    # replace brackets with '_' to pass into spot
    # print('fina', spec_exp)
    spec_exp = spec_exp.replace('.[','_')
    spec_exp = spec_exp.replace(']','')
    return spec_exp


if __name__ == '__main__':
    spec = 'F(m.[1]) & G(room.[1 | 2]) & G(!p.[1 & 2]) & F(m.[!(3 & 2)])'
    spec_exp = generate_spec(spec)
    print(spec_exp)

    save_buchi(spec_exp, '../dot_files/spec_method1.dot')

    spec_curr = 'G(!room) F a'

# add _# to every ap in buchi_curr




# # concantenate [pi, #] in dict_form_id to be "pi#"
# for k, v in dict_form_id.items():
#     dict_form_id[k] = ''.join(str(x) for x in v)
# print(dict_form_id)

# # flip the keys and values => pi#: formula
# dict_id_form = dict((v,k) for k,v in dict_form_id.items())
# print(dict_form_id)


# method 2: replace 
# spec_method2 = spec
# for k, v in dict_form_id.items():
#     spec_method2 = spec_method2.replace(k, v)
# print(spec_method2)

# create buchi, then replace
# f = spot.translate(spec_exp, 'BA', 'sbacc', 'unambig')
# file = open('spec_method2.dot', "w")
# file.write(f.to_str('dot'))

# file_str = f.to_str('dot')
# for line in file_str.splitlines():
#     if ' -> ' in line and 'label=' in line:
#         line_formula = line.split('label=')[1][1:-2]
#         print(line)
#         aps = line_formula.split(' ')
#         aps2 = [value for value in aps if value not in ['','&','|']]
#         aps2 = [value[1:] if value[0] == '!' else value for value in aps2]

#         for a in aps2:
#             line = line.replace(a, dict_id_form[a])

#     file.write(line+'\n')
# file.close()



# find path through buchi


