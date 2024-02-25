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
import psi_parser2
import os
import itertools

class BuchiBindings:
    def __init__(self, filename, spec_og):
        self.filename = filename
        self.spec_og = spec_og
        self.spec = ''
        self.graph = []
        self.init_state = -1
        self.accepting_states = []

    def load_buchi(self,remove_init=True):
        # convert buchi to nx
        f_nx = (nx.drawing.nx_pydot.read_dot(self.filename))

        with open(self.filename, 'r') as file:
            f = file.read()
        
        # label accepting states
        accepting_states = bf.find_accepting_states(f)
        accepting_labels = {k:'F' for k in accepting_states}
        nx.set_node_attributes(f_nx, accepting_labels, 'accepting')

        # find initial state
        init_buchi = list(f_nx.successors('I'))[0]
        nx.set_node_attributes(f_nx, {init_buchi: 'I'}, 'init')

        f_temp = f_nx.copy()
        # init_buchi = list(f_temp.successors('I'))[0]        # buchi should only have one init state
        # print(accepting_states, init_buchi)
        if remove_init:
            f_temp.remove_node('I')

        return f_temp, accepting_states, init_buchi


    def output_ap_list(self, spec_str):
        ap_list = list(set(re.findall('([\!a-z].*?)[^a-z0-9_]', spec_str)))
        ap_list.sort(key=len, reverse=True)

        ap_list = list(filter(('!').__ne__, ap_list))
        return ap_list

    def expand_spec_ap(self):
        '''
        expand self.spec so all binding formulas are on APs rather than on LTL formulas

        from <LTL formula>.[buchi formula] -> <LTL formula>.[binding formula] &| <LTL formula>.[binding formula] .... 
        '''
        # formula_list = re.findall('[A-Z\(!]+(.*?\])', self.spec)
        # formula_list = re.findall('([a-z]+.*?\])', self.spec)
        formula_list = re.findall('([^(|&][\!a-z]+.*?\])', self.spec)

        formula_list.sort(key=len, reverse=True)
        # print(formula_list, self.spec, 'list')

        formula_replaced = self.spec
        for formula in formula_list:
            formula_spec = formula.split('.')[0]
            binding = formula.split('.')[1]
            ap_list = self.output_ap_list('('+formula_spec)

            if ap_list == []:
                continue
            
            spec_replaced = formula_spec
            for ap in ap_list:
                # print('ap',ap, ap+'.'+binding)
                spec_replaced = spec_replaced.replace(ap, ap+'.'+binding)
            
            formula_replaced = formula_replaced.replace(formula, spec_replaced)

        return formula_replaced

    def expand_spec_binding(self, dict_form_id):
        '''
        from ap.[buchi formula] -> ap.[one binding] &| ap.[one binding] .... 
        '''

        # store an id number for each ap with binding and its tree, e.g. {id: [room.[3 | 4], Tree(3 | 4)]}
        spec_replaced = self.spec
        dict_num_form = {}
        counter = 0
        for form in dict_form_id.keys():
            tree_form = re.findall('\.\[(.*?)\]', form)[0]
            tree_form = tree_form.replace(" ","")
            dict_num_form[counter] = [form]     # ap
            # print(form , tree_form)
            dict_num_form[counter].append(psi_parser2.Tree.build(tree_form))
            # dict_num_form[counter].append(psi_parser.Tree.build(re.findall('\.\[(.*?)\]', form)[0]))
            spec_replaced = spec_replaced.replace(form, str(counter))
            counter += 1

        # print('dict', dict_num_form)
        # construct expansion
        spec_exp = self.spec_og
        for k, v in dict_num_form.items():
            form = v[0]
            tree = v[1]
            phi = form.split('.')[0]
            # find aps with negation
            # for idx in [m.start() for m in re.finditer(phi, spec_exp)]:
            #     if spec_exp[idx-1] == '!':
            #         spec_exp = spec_exp.replace('!' + form, tree.evaluate('!'+ap, tree.root))
            #         break
            

            spec_exp = spec_exp.replace(form, tree.evaluate(phi, tree.root))

        # print('after expand_spec_binding', spec_exp)

        return spec_exp


    def construct_buchi_edges(self, buchi):
        '''
        separate all OR parts of each edge formula in Buchi into separate edges
        include a sync ap with each edge
        '''
        def assign_buchi_values(buchi_formula):
            ''' given a buchi transition, generate dictionary {(ap, binding): True/False} <- not this
            {bindg: {ap: True/False}}
            '''
            ap_dict = {}
            bf_og = buchi_formula
            # buchi_formula = buchi_formula.replace('(1)','')
            buchi_formula = buchi_formula.replace('"','')
            ap_values = buchi_formula.replace('(','').replace(')','').replace('&','').replace('|','')
            ap_values = ap_values.split('  ')

            buchi_formula_upd = buchi_formula

            if ap_values == ['1']:
                return {'a':'1'}, ''

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
                                
                    binding_num = int(re.findall(r'\d+', binding)[0])
                    if binding_num in ap_dict:
                        if ap in ap_dict[binding_num]:
                            val_og = ap_dict[binding_num][ap]
                            # if not_ap and ap both exist in formula
                            if value.split('_')[0] != val_og.split('_')[0]:
                                return {}, ''
                            # if both !(ap) and not_ap exists, remove !(ap)
                            # if both !(not_ap) and ap exists, remove !(not_ap)
                            if (len(value.split('_')) + len(val_og.split('_'))) == 3:
                                ap_dict[binding_num][ap] = value.split('_')[0]
                                # remove !(ap) or !(not_ap) from formula
                                if value.split('_')[0] == 'False':
                                    # remove !(ap)
                                    remove_str1 = '!' + ap + '.' + binding
                                    remove_str2 = '!(' + ap + ').' + binding
                                else:
                                    # remove !(not_ap)
                                    remove_str1 = '!not_' + ap + '.' + binding
                                    remove_str2 = '!(not_' + ap + ').' + binding
                                buchi_formula_upd = buchi_formula_upd.replace(' & ' + remove_str1, '')
                                buchi_formula_upd = buchi_formula_upd.replace(' & ' + remove_str2, '')
                                buchi_formula_upd = buchi_formula_upd.replace(remove_str1 + ' & ', '')
                                buchi_formula_upd = buchi_formula_upd.replace(remove_str2 + ' & ', '')
                        else:
                            ap_dict[binding_num][ap] = value
                    else:
                        ap_dict[binding_num] = {ap: value}
                    # ap_dict[(ap, int(binding[1:-1]))] = value
            return ap_dict, buchi_formula_upd



        buchi_new = buchi.copy()
        # pi_sync = ord('a')
        for edge in buchi.edges():
            # pi_sync = ord('a')
            pi_sync = 1
            formula = buchi.get_edge_data(edge[0], edge[1])[0]['label'].strip('"')
            formula_or = formula.split(' | ')




            if len(formula_or) > 1:
                buchi_new.remove_edge(edge[0],edge[1])
                for subformula in formula_or:
                    ap_dict, subformula_upd = assign_buchi_values(subformula)
                    # assign_buchi_values returns {} if edge needs to be removed (if ap and not_ap exist)
                    if ap_dict:
                        buchi_new.add_edge(edge[0],edge[1], key='pi_' + str(pi_sync), label=subformula_upd, sync= 'pi_' + str(pi_sync))
                        pi_sync += 1
            else:
                buchi_new.remove_edge(edge[0],edge[1])
                ap_dict, formula_upd = assign_buchi_values(formula)
                if ap_dict:
                    buchi_new.add_edge(edge[0],edge[1], key='pi_' + str(pi_sync), label=formula, sync= 'pi_' + str(pi_sync))
                    # buchi_new[edge[0]][edge[1]][0]['sync']='pi_' + chr(pi_sync)
                    pi_sync += 1
                

        buchi_copy = buchi_new.copy()
        removed_nodes = set()
        init_state = list(nx.get_node_attributes(buchi_copy, "init").keys())[0]
        # remove any unreachable nodes
        for node in buchi_copy.nodes():
            nodes = list(buchi_new.predecessors(node))

            if len(nodes) == 1 and nodes[0] == node and node != init_state:
                buchi_new.remove_node(node)
                removed_nodes.add(node)




        return buchi_new

    def construct_binding_buchis(self, buchi):
        '''
        given centralized buchi (without edge formulas being separated by ORs), construct a buchi for each binding with sync APs
        '''

        binding_list_all = method2.extract_bindings(self.spec)
        buchi_dict = {}     # {binding #: buchi}

        for binding in binding_list_all:
            buchi_dict[int(binding)] = buchi.copy()

        for edge in buchi.edges():
            pi_sync = ord('a')

            # if buchi edge is 1, assign 1 to all corresponding binding buchi edges
            if buchi.get_edge_data(edge[0], edge[1])[0]['label'].strip('"') == '1':
                for binding, b in buchi_dict.items():
                    b.remove_edge(edge[0],edge[1])
                    b.add_edge(edge[0],edge[1], label='1',key= '\u03C0_' + chr(pi_sync))
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
                binding_ap_dict = self.construct_binding_ap_dict(formula, buchi_dict.keys())

                for binding, ap_formula in binding_ap_dict.items():
                    b = buchi_dict[binding]
                    
                    b.add_edge(edge[0],edge[1], label=ap_formula, key='\u03C0_' + chr(pi_sync))

                pi_sync = pi_sync+1
            

        return buchi_dict


    def construct_binding_ap_dict(self, formula, binding_list):
        '''
        given formula structured as ap.[binding] & ap.[binding] & ...
        output {binding: 'ap & !ap & ...']'}
        '''
        buchi = self.graph
        binding_ap_dict = {}
        for b in binding_list:
            binding_ap_dict[b] = '1' 

        spec_copy = re.sub('[())]', '', formula)
        pi_psi_list = spec_copy.split(' & ')
        for p in pi_psi_list:    # p = 'room1.[2]'
            ap = p.split('.')[0]
            binding = int(p.split('.')[1][1:-1])    # remove brackets for binding

            if binding_ap_dict[binding] != '1':
                binding_ap_dict[binding] += ' & ' + ap
            else:
                binding_ap_dict[binding] = ap

        return binding_ap_dict

    def buchi_to_dot(self,buchi,name):
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






    def update_continuous(self,buchi,non_inst_actions):
        ''' 
        check any transition where more than one non-instantaneous ap must be true
        remove transition in buchi from A to B where the self-transitions on A does not allow for intermediate states to satisfy that transition
        remove transition in buchi from A to A if it does not allow for intermediate states to satisfy that transition
        
        action_dict         {ap: 0 for instantaneous, 1 for non-instantaneous}

        non_inst_actions     list of aps that represent non-instantaneous actions
        '''



        def check_continuous(ap_binding_dict, non_inst_actions):
            # see if we need to check transition (> 1 non-instantaneous action)
            check = 0
            for ap, binding in ap_binding_dict:
                if ap in non_inst_actions and ap_binding_dict[(ap, binding)] == True:
                    check += 1
                if check > 1:

                    return True 

            return False

        buchi_upd = buchi.copy()
        buchi_upd.remove_edges_from(list(buchi_upd.edges()))
        add_transitions = {}

        for edge in buchi.edges():
            for edge_idx in buchi.get_edge_data(edge[0],edge[1]).keys():
                edge_data = buchi.get_edge_data(edge[0],edge[1])[edge_idx]
                buchi_formula = edge_data['label']
                pi_sync = edge_data['sync']
                buchi_formula = buchi.get_edge_data(edge[0],edge[1])[edge_idx]['label']
                pi_sync = buchi.get_edge_data(edge[0],edge[1])[edge_idx]['sync']

                print(edge[0],edge[1], buchi_formula)

                if buchi_formula == "1":
                    add_transitions[(edge[0],edge[1], pi_sync)] = edge_data
                    continue
                ap_binding_dict = pa.assign_buchi_values(buchi_formula)   # {(ap, binding): True/False}

                # if <= 1 non-inst action
                print(check_continuous(ap_binding_dict, non_inst_actions))
                if not check_continuous(ap_binding_dict, non_inst_actions):
                    # add if self-transition
                    if edge[0] == edge[1]:
                        # buchi_upd.remove_edge(edge[0],edge[1],sync=pi_sync)

                        add_transitions[(edge[0],edge[1], pi_sync)] = edge_data
                    # else:


        for d,v in add_transitions.items():
            buchi_upd.add_edge(d[0], d[1], label=v['label'], sync=v['sync'])
        return buchi_upd


                    
   



    def construct_buchi(self, individual_buchi=False, sync_file="", load_file = None):

        # first, move the binding formulas to each AP
        # ap_list = self.output_ap_list(self.spec_og)
        # print('ap list', ap_list, self.spec_og)

        # then expand binding formulas  
        dict_form_id = method2.create_phipsi_ids(self.spec_og)
        print('dict',dict_form_id)

        # method2.spec_to_dot(self.filename, self.spec, dict_form_id)
        # method2.spec_to_dot('original.dot', self.spec, dict_form_id)
        print('spec og', self.spec_og)
        self.spec = self.expand_spec_binding(dict_form_id)

        print('spec2', self.spec)
        self.spec = self.expand_spec_ap()

        print('spec3', self.spec)

        all_bindings = list(set(re.findall('\[([0-9])\]+',self.spec)))
        all_bindings = sorted([int(x) for x in all_bindings])


        dict_form_id = method2.create_ap_ids(self.spec)

        # expand any !(subformula)
        self.spec = self.find_neg_subform(self.spec)
        # self.spec = self.spec.replace('!', 'not_')
        
        print('final spec ', self.spec, all_bindings)
        dict_form_id = method2.create_ap_ids(self.spec)
        print(dict_form_id)

        print('before', dict_form_id)
        # for aps in dict with ! - add parentheses around it
        dict_form_id_og = dict_form_id.copy()
        for ap in dict_form_id_og:
            if ap[0:3] == 'not_':
                
                dict_form_id['(' + ap + ')'] = dict_form_id[ap]
                dict_form_id.pop(ap)


        print('after', dict_form_id)

        

        # print(form , tree_form)

        # find any part of spec with !(phi), and expand into DNF
    


        
        
        method2.spec_to_dot(self.filename, self.spec, dict_form_id)
        method2.spec_to_dot('before_splitting.dot', self.spec, dict_form_id)

        # separate out the edges for each OR part of the transition formulas
        buchi, accepting_states, init_buchi = self.load_buchi(self.filename)
        buchi_new = self.construct_buchi_edges(buchi)

        write_dot(buchi_new, self.filename)

        # buchi_new = self.update_continuous(buchi_new,non_inst_actions)

        self.graph = buchi_new
        self.init_state = init_buchi
        self.accepting_states = accepting_states

        # os.system("dot -Tpng -Gdpi=300 " + self.filename + " > " + self.filename[:-4] + ".png")
        self.buchi_to_dot(self.graph, self.filename)

        
        # save buchi file
        if sync_file != "":
            buchi_temp = buchi_new.copy()
            buchi_temp.remove_edges_from(list(buchi_temp.edges()))

            sync_transitions = {}
            for buchi_state in buchi_new.nodes():
                neighbors = buchi_new.successors(buchi_state)
                for neighbor_b in neighbors:
                    for edge_idx in buchi_new.get_edge_data(buchi_state,neighbor_b).keys():
                        buchi_formula = buchi_new.get_edge_data(buchi_state,neighbor_b)[edge_idx]['label']
                        # make sure the ! generated by buchi has correct parentheses !(ap.[binding])

                        subforms = buchi_formula.split(' & ')
                        for f in subforms:
                            # remove outer ()
                            f = f.replace("(","")
                            if f[0] == '!':
                                f_new = "!(" + f[1:] + ")"
                                # if f[-1] != ")":
                                #     f_new = f_new + ")"
                                buchi_formula = buchi_formula.replace(f, f_new)
                        buchi_new.get_edge_data(buchi_state,neighbor_b)[edge_idx]['label'] = buchi_formula



                        pi_sync = buchi_new.get_edge_data(buchi_state,neighbor_b)[edge_idx]['sync'] 


                        sync_transitions[(buchi_state, neighbor_b, pi_sync)] = buchi_formula

            for d,v in sync_transitions.items():
                buchi_temp.add_edge(d[0], d[1], label=v, sync=d[2], key=d[2])

            self.buchi_to_dot(buchi_temp,sync_file)


        
        # if we want the buchis of each binding
        if individual_buchi:
            buchi_dict = self.construct_binding_buchis(buchi)

            # save buchi files
            for binding, b in buchi_dict.items():
                self.buchi_to_dot(b,'buchi' + str(binding) + '.dot')

            return buchi_new, init_buchi, accepting_states #, buchi_dict


        

        return buchi_new, init_buchi, accepting_states, all_bindings   


    def find_neg_subform(self, formula):
        def find_paren(text):
            istart = []  # stack of indices of opening parentheses
            d = {}
            
            for i, c in enumerate(text):
                if c == '(':
                     istart.append(i)
                if c == ')':
                    try:
                        d[istart.pop()] = i
                    except IndexError:
                        print('Too many closing parentheses')
            if istart:  # check if stack is empty afterwards
                print('Too many opening parentheses')
            return d
        
        def buchi_expansion():
            # f = spot.translate(text, 'BA', 'sbacc', 'unambig')

            # file = open('temp.dot', "w")
            # file.write(f.to_str('dot'))

            # # file_str = f.to_str('dot')

            # file.close()
            # load buchi
            f_nx = (nx.drawing.nx_pydot.read_dot('temp.dot'))

            with open('temp.dot', 'r') as infile:
                # data = infile.read()
                

                for line in infile:

                    # find the label with the formula
                    # for line in file_str.splitlines():
                    
                    # find non self-trans edge
                    if len(set(int(s) for s in line.split() if s.isdigit())) > 1:
                        # get formula
                        formula = re.findall('"([^"]*)"', line)[0]
                        return '(' + formula + ')'


        paren_dict = find_paren(formula)

        not_idxs = [i for i, x in enumerate(formula) if x == "!"]

        formula_upd = formula

        for idx in not_idxs:
            if formula[idx+1] == '(':
                # need to expand the ! expression
                # !(pi^1 & pi^2) -> !(pi^1) & !(pi^2)
                expr_end = paren_dict[idx+1]+1

                subformula = formula[idx:expr_end]
                dict_form_id = method2.create_ap_ids(subformula, neg=True)
                method2.spec_to_dot('temp.dot', subformula, dict_form_id)

                formula_dnf = buchi_expansion()
                formula_upd = formula_upd.replace(subformula, formula_dnf)

        # replace !room with not_room
        pi_list = re.findall(r'!(?!\()\s*([^\]]+)', formula_upd)
        for pi in pi_list:
            formula_upd = formula_upd.replace('!'+pi, 'not_'+pi)
        return formula_upd





