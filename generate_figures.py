#!/usr/bin/env python

# construct synchronization buchi structure for each robot using method of creating one product automaton with all bindings

# importing sys
import sys
import os
  
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
import testing_methods as method1
import testing_methods2 as method2
import buchi_functions as bf
import psi_parser
import centralized_synth
from buchi_bindings_module import BuchiBindings
import math
import itertools
import synthesize_team_behavior as stb
import product_automata as pa
import find_optimal_run as fr
import plot_agents 
import os

def create_arm(weighting_factor):
    K = nx.DiGraph()

    # box capability
    p = 'pickup'
    d = 'dropoff'
    c = 'carry'

    not_p = '!pickup'
    not_d = '!dropoff'
    not_c = '!carry'

    ap = [p, d, c]

    s_p = ' & '.join([p, not_d, not_c])
    s_d = ' & '.join([not_p, d, not_c])
    s_c = ' & '.join([not_p, not_d, c])
    s_idle = ' & '.join([not_p, not_d, not_c])
    s_l = ' & '.join([not_p, not_d, not_c])

    K.add_node(s_p); K.add_node(s_d); K.add_node(s_c)

    K.add_edge(s_p, s_d, weight = 0.1*weighting_factor); 
    K.add_edge(s_p, s_c, weight = 0.5*weighting_factor);  

    K.add_edge(s_d, s_idle, weight = 0.05*weighting_factor);
    K.add_edge(s_d, s_p, weight = 0.5*weighting_factor); 
    
    K.add_edge(s_idle, s_idle, weight = 0*weighting_factor); 
    K.add_edge(s_idle, s_p, weight = 0.5*weighting_factor); 

    K.add_edge(s_c, s_c, weight = 0.5*weighting_factor)
    K.add_edge(s_c, s_d, weight = 0.1*weighting_factor);

    # lever capability

    # K.add_edge(s_idle, s_l, weight = 0.25*weighting_factor)
    # K.add_edge(s_l, s_idle, weight = 0*weighting_factor)
    # K.add_edge(s_l, s_l, weight = 0.25*weighting_factor)

    # K.add_edge(s_d, s_l, weight = 0.25*weighting_factor)
    # K.add_edge(s_l, s_p, weight = 0.25*weighting_factor)


    return K, ap

def create_rooms(weighting_factor):
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


    ap = [l1, l2, l3, l4, l1c, l2c, l3c, l4c]  # , l10]

    s1 = ' & '.join([l1, not_l2, not_l3, not_l4,
                     not_l1c, not_l2c, not_l3c, not_l4c])
    s2 = ' & '.join([not_l1, l2, not_l3, not_l4,
                     not_l1c, not_l2c, not_l3c, not_l4c])
    s3 = ' & '.join([not_l1, not_l2, l3, not_l4,
                     not_l1c, not_l2c, not_l3c, not_l4c])
    s4 = ' & '.join([not_l1, not_l2, not_l3, l4,
                     not_l1c, not_l2c, not_l3c, not_l4c])

    s1c = ' & '.join([not_l1, not_l2, not_l3, not_l4,
                      l1c, not_l2c, not_l3c, not_l4c])
    s2c = ' & '.join([not_l1, not_l2, not_l3, not_l4,
                      not_l1c, l2c, not_l3c, not_l4c])
    s3c = ' & '.join([not_l1, not_l2, not_l3, not_l4,
                      not_l1c, not_l2c, l3c, not_l4c])
    s4c = ' & '.join([not_l1, not_l2, not_l3, not_l4,
                      not_l1c, not_l2c, not_l3c, l4c])

    K.add_node(s1)
    K.add_node(s2)
    K.add_node(s3)
    K.add_node(s4)

    K.add_node(s1c)
    K.add_node(s2c)
    K.add_node(s3c)
    K.add_node(s4c)

    K.add_edge(s1c, s2, weight=0.3 * weighting_factor);
    K.add_edge(s2c, s3, weight=0.3 * weighting_factor);
    K.add_edge(s3c, s4, weight=0.3 * weighting_factor);
    K.add_edge(s4c, s1, weight=0.3 * weighting_factor);

    K.add_edge(s2c, s1, weight=0.3 * weighting_factor);
    K.add_edge(s3c, s2, weight=0.3 * weighting_factor);
    K.add_edge(s4c, s3, weight=0.3 * weighting_factor);
    K.add_edge(s1c, s4, weight=0.3 * weighting_factor);

    K.add_edge(s1, s1c, weight=0.3 * weighting_factor);
    K.add_edge(s2, s2c, weight=0.3 * weighting_factor);
    K.add_edge(s3, s3c, weight=0.3 * weighting_factor);
    K.add_edge(s4, s4c, weight=0.3 * weighting_factor);


    # K.add_edge(s1, s1, weight = 0);
    # K.add_edge(s2, s2, weight = 0);
    # K.add_edge(s3, s3, weight = 0);
    # K.add_edge(s4, s4, weight = 0);
    # K.add_edge(s5, s5, weight = 0);
    # K.add_edge(s6, s6, weight = 0);
    # K.add_edge(s7, s7, weight = 0);
    # K.add_edge(s8, s8, weight = 0);
    # K.add_edge(s9, s9, weight = 0);

    K.add_edge(s1c, s1c, weight=0);
    K.add_edge(s2c, s2c, weight=0);
    K.add_edge(s3c, s3c, weight=0);
    K.add_edge(s4c, s4c, weight=0);

    return K, ap

def robot_model(pretty_dict):
    # agent 4
    loc4, ap_loc4 = pa.create_rooms_wh(1)
    # arm4, ap_arm4 = create_arm(1)
    caps4 = [loc4] #, downlink2, batt2, track2]
    ap4 = [ap_loc4]
    # pretty = [reg_pretty]
    cap_names = ['loc']

    ag4 = Agent(ap4, caps4, 4)

    # individual caps
    # for idx in range(len(caps4)):
    #     plot_agents.save_plot(caps4[idx], 'cap_' + cap_names[idx], pretty_dict=pretty_dict[cap_names[idx]], weight_dict = None)

    # nx.draw(loc4, labels = pretty_dict, with_labels=True)
    # plt.show()

    # overall robot model
    plot_agents.save_plot(loc4, 'agent_fig', pretty_dict=pretty_dict, weight_dict = None)

    # pa.buchi_to_dot(a2plot,'agent_fig')

def buchi_pretty(filename, example_ext, path=None, with_legend=False):

    # load buchi
    f_nx = (nx.drawing.nx_pydot.read_dot(filename))

    outfile_name= filename[:-4] + '_' + example_ext + '.dot'

    with open(filename, 'r') as infile, \
     open(outfile_name, 'w') as outfile:
        # data = infile.read()

        for line in infile:
            # if 'graph' in line:
            #     print('here')
            #     line = line.replace('graph [label="\"\n[Büchi]\""', 'graph [ratio=0.4, nodesep=0.22;')
            if '[Büchi]' in line.strip():
                print('here')
                line = line.replace('\\"\\n[Büchi]\\"', "")
                # line = line.replace('\"\n', "")

            elif 'label=' in line.strip() and "." in line.strip():
                # if not with_legend:
                check_line = re.findall('"([^"]*)"', line)
                
                if len(check_line) != 0:
                    formula = check_line[0]
                    formula_replace = formula
                    # if formula_replace[0] == '(' and formula_replace[-1] == ')':
                    #     formula_replace = formula_replace[1:-1]

                    formula_replace = formula_replace.replace("&", "&amp;")
                    formula_replace = formula_replace.replace("not_", "&not;")

                    formula_replace = formula_replace.replace("!", "&not;")
                    formula_replace = '<' + formula_replace + '>'

                    line = line.replace('"' + formula + '"', formula_replace)







                #     line = line.replace(')"', ">")
                #     line = line.replace("'(", "<")
                #     # line = line.replace('"(', "")
                    
                #     # line = line.replace(")'", "")

                #     line = line.replace("&", "&amp;")
                #     # pi_num = re.findall('pi_(.*?)[^0-9]', line)[0]
                #     # # line = line.replace(", 'pi_" + pi_num + "'", ' &amp; &pi;<SUP>s</SUP><SUB>' + pi_num + '</SUB>')
                #     # line = line.replace(", 'pi_" + pi_num + "'", '')

                #     line = line.replace("='", "=<(")
                #     line = line.replace("]'", "]")
                #     line = replace_str(line, replace_dict)
                #     line = line.replace("'", "")
                #     line = line.replace("(", "")
                # else:
                #     formula = re.findall('"(.*?)"', line)[0]
                #     pi_num = re.findall('pi_(.*?)[^0-9]', line)[0]
                #     line = line.replace('"' + formula + '"', '<&pi;<SUP>s</SUP><SUB>' + pi_num + '</SUB>>')





                #make bindings superscripts
                bindings = re.findall('\.(.*?)\]', line)
                for b in bindings:
                    b = b[1:]
                    line = line.replace('.['+b+']', '<SUP>' + b + '</SUP>')

                # highlight path
            elif 'key' in line.strip() and path:
                buchi_path, self_transitions = path
                if buchi_path:
                    highlight = ' penwidth = 5, color = "mediumpurple1", fontcolor = "mediumpurple1",'
                    edge = re.findall('([(0-9)\*\"]*( -> )[(0-9)\*\"]*)', line)[0][0]
                    s1, s2 = re.findall('[(0-9)\*]+', edge)
                    pi_sync = re.findall('((pi)[a-z0-9_]*)', line)[0][0]

                    if (s1 == s2 and self_transitions.get(s1) == pi_sync) \
                            or (s1, s2, pi_sync) in buchi_path:
                        # add highlighting
                        line += highlight

            else:
                line = line.replace('\\"', "")
                
            outfile.write(line)

    outfile.close()

    print(outfile_name)

    os.system("dot -Tpng -Gdpi=300 " + outfile_name + " > " + outfile_name[:-4] + ".png")


    return 

def replace_str(string, replace_dict):
    for d, v in replace_dict.items():
        string = string.replace(d, v)
    return string


if __name__ == '__main__':
    replace_dict = {'room1': 'room<SUB>A</SUB>', \
                    'room2': 'room<SUB>B</SUB>', \
                    'room3': 'room<SUB>C</SUB>', \
                    'room4': 'room<SUB>D</SUB>', \
                    'room5': 'room<SUB>E</SUB>', \
                    'room6': 'room<SUB>F</SUB>', \
                    'room7': 'room<SUB>G</SUB>', \
                    'room8': 'room<SUB>H</SUB>', \
                    'room9': 'room<SUB>J</SUB>',
                    }




    # buchi_pretty('prod_aut_partial.dot', 'aut_test', path=None)

    reg_pretty9 = {'room1 & !room2 & !room3 & !room4 & !room5 & !room6 & !room7 & !room8 & !room9 & !room1c & !room2c & !room3c & !room4c & !room5c & !room6c & !room7c & !room8c & !room9c': 'room1', \
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

    reg_pretty4 = {'room1 & !room2 & !room3 & !room4 & !room1c & !room2c & !room3c & !room4c': 'room1', \
        '!room1 & room2 & !room3 & !room4 & !room1c & !room2c & !room3c & !room4c': 'room2', \
        '!room1 & !room2 & room3 & !room4 & !room1c & !room2c & !room3c & !room4c': 'room3', \
        '!room1 & !room2 & !room3 & room4 & !room1c & !room2c & !room3c & !room4c': 'room4', \
        '!room1 & !room2 & !room3 & !room4 & room1c & !room2c & !room3c & !room4c': 'room1c', \
        '!room1 & !room2 & !room3 & !room4 & !room1c & room2c & !room3c & !room4c': 'room2c', \
        '!room1 & !room2 & !room3 & !room4 & !room1c & !room2c & room3c & !room4c': 'room3c', \
        '!room1 & !room2 & !room3 & !room4 & !room1c & !room2c & !room3c & room4c': 'room4c'
                   }

    arm_pretty = {  'pickup & !dropoff & !carry & !lever': 'pickup', \
                '!pickup & dropoff & !carry & !lever': 'dropoff', \
                '!pickup & !dropoff & carry & !lever': 'carry', \
                '!pickup & !dropoff & !carry & lever': 'lever', \
                '!pickup & !dropoff & !carry & !lever': 'arm_idle' , \
                # '!camera': '!camera', \
                # 'camera': 'camera'
                  }

    arm_pretty = {  'pickup & !dropoff & !carry': 'pickup', \
                '!pickup & dropoff & !carry': 'dropoff', \
                '!pickup & !dropoff & carry': 'carry', \
                '!pickup & !dropoff & !carry': 'arm_idle',\
                             # '!camera': '!camera', \
                # 'camera': 'camera'
                  }

    cam_pretty = {
                '!camera': '!camera', \
                'camera': 'camera'
                  }
    beep_pretty = {
                '!beep': '!beep', \
                'beep': 'beep'
                  }


    # pretty_dict = {}
    # for reg in reg_pretty:
    #     for arm in arm_pretty:
    #         pretty_dict[(reg, arm)] = (reg_pretty[reg],arm_pretty[arm])
    #     # for cam in cam_pretty:
    #     #     for beep in beep_pretty:
    #     #         pretty_dict[(reg, cam,beep)] = (reg_pretty[reg],cam_pretty[cam], beep_pretty[beep])

    pretty_dict = reg_pretty4
    robot_model(pretty_dict)


