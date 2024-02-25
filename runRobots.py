#!/usr/bin/python3

"""
Given agents and their respective high level plans, execute the plans. Connect to Optitrack
topics and Anki vectors to send commands.

"""


import sys
import os

sys.path.insert(0, '../../Mission_Switching/scripts')
sys.path.insert(0, '/Users/amy/usr/lib/python3.7/site-packages/')
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                 "/../PythonRobotics/")
import numpy as np
import shapely.geometry as geometry
from shapely.ops import cascaded_union
import matplotlib.pyplot as plt
import time
import csv
import product_automata as pa
import execute_sync as es


from PathPlanning.VisibilityRoadMap import visibility_road_map as vrm


class runRobots:
    def __init__(self, map_arrayfile):
        self.fwdVel = 0; self.angVel = 0;
        self.maxV = 1; self.wheel2Center = 1e-5;
        self.Eps = 0.2;
        self.closeEnough = 0.2;
        self.alpha = 2;
        self.gamma = 1;     
        self.Ds = 1;   
        self.counter = 1;
        self.new_inputs = {}
        self.delt = 0.1;
        self.bloatFactor = 0.4
        self.expand_distance = 0.01

        # self.bindings = [[1, 0, 1], [1, 1, 0]] 
        self.bindings = [[1,1,0], [1,1,1], []]
        self.rooms = {"room1": [0.5, 8], "room2": [1,6], "room3":[1,4], "room4":[1.5, 2.5], "room5":[1,1], "room6":[3.5, 3.5], "room7":[5,5],"room8":[4.5,1.5],"room9":[6.5,1]}
        self.actions = {"scan": 3, "pickup":10, "camera": 5}

        self.map_arrayfile = map_arrayfile
        with open(self.map_arrayfile) as f:
            self.map_array = [[float(x) for x in line[5:].split()] for line in f if line[0] == "w"]
        with open(self.map_arrayfile) as f:
            self.line_array = [[float(x) for x in line[5:].split()] for line in f if line[0] == "l"]

        agent_list, ap_list = pa.generate_agents(3)
        self.agent_list = agent_list[0:4]
        self.all_bindings = [1, 2, 3]

        # self.agent_list = agent_list[0:2]
        # self.all_bindings = [1, 3]

        self.obstacle_list = []

    def problem_setup(self):
        self.obstacle_list = es.polygonalWorld(self.map_array, self.bloatFactor)

        # create roadmap of just obstacles
        obstacles = []
        for obs in self.obstacle_list:
            obs_x = [p[0] for p in obs]
            obs_y = [p[1] for p in obs]
            obstacles.append(vrm.ObstaclePolygon(obs_x, obs_y))

        _, obs_nodes = vrm.VisibilityRoadMap(self.expand_distance, do_plot=False).generate_visibility_nodes(0, 0, 0, 0,
                                                                                            obstacles, obs_nodes=None)

        # initializing agents

        # agent_list, ap_list = pa.generate_agents(3)
        # agent_list = agent_list[0:4]
        # all_bindings = [1,2,3]
        # num_bindings = len(all_bindings)
        # num_robots = len(agent_list)
        # team_assignment = {1: {1, 3}, 2: {2}, 3: {1}, 4: {2}}

        for agent in self.agent_list:
            if agent.id == 1:
                # plan1 = [["room8", "arm_idle", "!scan"], \
                #          ["room6", "arm_idle", "!scan"], \
                #          ["room2", "arm_idle", "!scan"], \
                #          ["room1", "arm_idle", "!scan"], \
                #          ["room1", "arm_idle", "scan"]]
                plan1 = [["room7", "arm_idle", "!scan"], \
                         ["room8", "arm_idle", "!scan"]]

                plan2 = [ ["room6", "arm_idle", "!scan"], \
                            ["room2", "arm_idle", "!scan"], \
                            ["room1", "pickup", "!scan"]]
                plan3 = [["room1", "!pickup", "!scan"]]

                # plan2 = [["room1", "arm_idle", "!scan"], \
                #          ["room1", "pickup", "!scan"]]
                # plan2 = [
                #          ["room1", "pickup", "!scan"]]

                agent.bindings = {1}
                agent.pose = self.rooms["room7"] + [0]    # [x y theta]  *** change to Optitrack input ****

            elif agent.id == 2:
                plan1 = [["room6", "!beep"]]

                plan2 = [["room8", "!beep"], \
                        ["room9", "beep"]]

                plan3 = [["room9", "!beep"]]
                # plan1 = [["room7", "!camera"]]

                # plan2 = [["room8", "!camera"], \
                #          ["room9", "!camera"], \
                #          ["room9", "camera"]]

                agent.bindings = {3}
                agent.pose = self.rooms["room6"] + [0]    # [x y theta]  *** change to Optitrack input ****

            elif agent.id == 3:
                plan1 = [["room3", "arm_idle"], \
                        ["room3", "arm_idle"]]

                plan2 = [["room2", "arm_idle"], \
                            ["room1", "pickup"]]

                plan3 = [["room1", "!pickup"]]

                agent.bindings = {1}
                agent.pose = self.rooms["room3"] + [0]    # [x y theta]  *** change to Optitrack input ****

            elif agent.id == 4:
                plan1 = [["room9", "!camera", "!scan", "!beep"], \
                        ["room8", "!camera", "!scan", "!beep"], \
                        ["room6", "!camera", "!scan", "!beep"], \
                        ["room2", "!camera", "!scan", "!beep"], \
                        ["room1", "!camera", "scan", "!beep"]]

                plan2 = [["room2", "!camera", "!scan", "!beep"], \
                        ["room6", "!camera", "!scan", "!beep"], \
                        ["room8", "!camera", "!scan", "!beep"], \
                        ["room9", "!camera", "!scan", "beep"]]

                plan3 = [["room9", "!camera", "!scan", "!beep"]]
                # plan1 = [["room9", "!camera", "  !beep"]]
                # # plan2 = [["room9", "!camera", "!beep"],\
                # #          ["room9", "camera", "!beep"]]
                # plan2 = [
                #          ["room9", "camera", "!beep"]]

                agent.bindings = {2,3}
                agent.pose = self.rooms["room9"] + [0]    # [x y theta]  *** change to Optitrack input ****



            agent.plan = [plan1, plan2]


            # convert agent plans into coordinates
            # ag.plan_coords = {}
            # for path_idx in range(len(ag.plan)):
            agent.plan_coords = es.findRoomCoords(agent.plan, self.rooms, self.actions)


            # ag.gotopt = 1
            # ag.wait_gotopt = 1
            # ag.sync_gotopt = 1
            # ag.counter = 1
            agent.action_timer = -1

            agent.truthPose = []
            # [~,all_robots(idx).plan_coords{1}] = findPath(roadmap,all_robots(idx).current_wp,all_robots(idx).next_wp);

            agent.current_wp = agent.pose + [0]*len(self.actions)
            agent.next_wp = agent.plan_coords[0][0]

            # agent.inputs = [0, 0]    # cmdV cmdW 
            agent.inputs_nom = [0, 0]
            agent.wp_idx = 0;

            agent.all_wp = {}
            agent.wait_wp = {}
            agent.sync_wp = {}
            for path_idx in range(len(agent.plan)):
                if path_idx > 0:
                    current_pose = agent.plan_coords[path_idx-1][-1] 
                    # reset the action (THIS MAY CHANGE DEPENDING ON HOW I DEFINE THINGS)
                    current_pose[2:] = [0]*len(self.actions)
                else:
                    current_pose = agent.pose + [0]*len(self.actions)
                agent.all_wp[path_idx] = es.generateWP(agent.plan_coords, self.actions, path_idx, current_pose, 0, obstacles, self.expand_distance, obs_nodes)


                # wait transition
                current_pose = agent.all_wp[path_idx][-1]
                agent.wait_wp[path_idx] =  es.generateWP(agent.plan_coords, self.actions, path_idx, current_pose, 1, obstacles, self.expand_distance, obs_nodes)

                # behavior after all robots are ready
                current_pose = agent.wait_wp[path_idx][-1]
                agent.sync_wp[path_idx] = es.generateWP(agent.plan_coords, self.actions, path_idx, current_pose, 2, obstacles, self.expand_distance, obs_nodes)

    def execute_plan(self, show_plot=False, save_data=False):
        path_length = len(self.agent_list[0].plan)
        num_agents = len(self.agent_list)
        num_actions = len(self.actions)



        for ag in self.agent_list:
            print(ag.id, '---------------------------------')
            print( ag.all_wp)
            print(ag.wait_wp)
            print(ag.sync_wp)

        # if not robot_class:
        # fwdVel = 0; angVel = 0;
        # maxV = 1; wheel2Center = 1e-5;
        # Eps = 0.2;
        # closeEnough = 0.2;
        # alpha = 2;
        # gamma = 1;     
        # Ds = 1;   
        # counter = 1;
        # new_inputs = {}
        # delt = 0.1;
        # else:
        #     fwdVel = robot_class.fwdVel; angVel = robot_class.angVel;
        #     maxV = robot_class.maxV; wheel2Center = robot_class.wheel2Center;
        #     Eps = robot_class.Eps
        #     closeEnough = robot_class.closeEnough
        #     alpha = robot_class.alpha;
        #     gamma = robot_class.gamma;     
        #     Ds = robot_class.Ds;   
        #     counter = robot_class.counter;
        #     new_inputs = robot_class.new_inputs
        #     delt = robot_class.delt


        # control loop
        if show_plot:
            fig = plt.figure()

        # figure, ax = plt.subplots(figsize=(10, 8))
        t0 = time.time()
        for path_idx in range(path_length):
            required_bindings = es.find_indices(self.bindings[path_idx], 1)
            Rbar = []
            team_done = [1]*num_agents
            ready_vec = [1]*num_agents
            waiting = 0

            # find robot ids that need to sync in this step of path
            for agent_idx in range(num_agents):
                agent = self.agent_list[agent_idx]
                agent.done = 0
                agent.counter = 0
                agent.sync_gotopt = 0
                agent.wait_gotopt = 0
                agent.gotopt = 0
                agent.counter = 0
                if set(agent.bindings).intersection(required_bindings):
                    Rbar.append(agent.id)
                    ready_vec[agent_idx] = 0
                    team_done[agent_idx] = 0

                agent.current_wp = agent.pose[1:2] + [0]*num_actions
                agent.next_wp = agent.plan_coords[path_idx][0]

            team_done_sync = team_done

            # while loop to get up to everyone being ready
            while not all(i == 1 for i in team_done) and not all(i == 1 for i in team_done_sync):
                u_hat = {}
                new_inputs = {}
                for agent in self.agent_list:
                    # execute behavior up to last step
                    if agent.done == 0 and waiting == 0:
                        path2do = agent.all_wp[path_idx]
                        agent = es.robotController2(agent, path2do, waiting, self.Eps, self.closeEnough, self.alpha, self.maxV, self.wheel2Center)
                        u_hat[agent] = agent.inputs_nom
                        if agent.done:
                            ready_vec[agent.id - 1] = 1

                    else:
                        # if no need to sync
                        if agent.id not in Rbar or [agent.id] == Rbar:
                            path2do = agent.all_wp[path_idx]
                            agent = es.robotController2(agent, path2do, 2, self.Eps, self.closeEnough, self.alpha, self.maxV, self.wheel2Center)
                            u_hat[agent] = agent.inputs_nom

                        else:

                            if agent.done:
                                ready_vec[agent.id-1] = 1
                            
                            #wait for all other robots to be ready
                            #wait
                            if sum(ready_vec) != len(ready_vec):     # if not all elements in ready_vec = 1
                                u_hat[agent] = [0,0]
                            else:
                                # execute last step in behavior
                                waiting = 2
                                path2do = agent.sync_wp[path_idx]
                                agent = es.robotController2(agent, path2do, waiting, self.Eps, self.closeEnough, self.alpha, self.maxV, self.wheel2Center)
                                u_hat[agent] = agent.inputs_nom
                                if agent.done:
                                    team_done_sync[agent.id-1] = 1

                
                self.counter = self.counter + 1

                new_inputs = u_hat.copy()           # command to send to Optitrack


                for agent in self.agent_list:
                    # agent.inputs = new_inputs[agent]

                    d = new_inputs[agent][0]*self.delt; phi = new_inputs[agent][1]*self.delt;
                    agent.pose = es.integrateOdom(agent.pose, d, phi)    # CHANGE to feedback from Optitrack
                    agent.truthPose.append([time.time()] + agent.pose)
                    
                    if agent.action_timer >= 0: 
                        b = [agent.id, agent.action_timer] 
                        print(b) 
            
            
                time.sleep(self.delt)

                # ------ PLOTTING ------
                
                if show_plot:
                    plt.clf()

                    # plt.clf();
                    # plt.set(gca,'position',[0, 0, 1, 1])
                    # plt.set(gca,'Color','white')
                    plt.grid(visible=None, zorder=0)
                    plt.xlim([-1, 10])
                    plt.ylim([-1, 10])

                    # plot walls
                    x1,y1,x2,y2 = list(), list(), list(), list()
                    for row in self.map_array:
                        x1.append(row[0])
                        y1.append(row[1])
                        x2.append(row[2])
                        y2.append(row[3])

                    for i in range(len(x1)):
                        plt.plot([x1[i], x2[i]], [y1[i], y2[i]], 'k', zorder=10);

                    # plot rooms
                    x1,y1,x2,y2 = list(), list(), list(), list()
                    for row in self.line_array:
                        x1.append(row[0])
                        y1.append(row[1])
                        x2.append(row[2])
                        y2.append(row[3])
                    for i in range(len(x1)):
                        plt.plot([x1[i], x2[i]], [y1[i], y2[i]], 'k:', zorder=10);

                    # plot robots
                    plt.scatter(self.agent_list[0].pose[0], self.agent_list[0].pose[1], s=200,marker='d', color=[0.8500, 0.3250, 0.0980], zorder=20)
                    plt.scatter(self.agent_list[1].pose[0], self.agent_list[1].pose[1], s=200,marker='o',color="blue", zorder=20)
                    plt.scatter(self.agent_list[2].pose[0], self.agent_list[2].pose[1], s=200,marker='s', color=[0, 0.5, 0], zorder=20)
                    plt.scatter(self.agent_list[3].pose[0], self.agent_list[3].pose[1], s=200,marker='^', color='magenta', zorder=20)

                    
                    plt.pause(0.01)
                    # time.sleep(self.delt)

        
        if show_plot:
            plt.show()
            plt.close('all') 

        # save truthPose data
        if save_data:
            for agent in self.agent_list:

                with open('truthPose_robot'+str(agent.id)+'.csv', 'w', encoding='UTF8', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerows(agent.truthPose)


if __name__ == '__main__':

    r = runRobots('map_9rooms.txt')

    r.problem_setup()
    r.execute_plan(show_plot=True, save_data=False)




