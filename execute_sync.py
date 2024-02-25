#!/usr/bin/env python

import sys
import os
import numpy as np
import shapely.geometry as geometry
from shapely.ops import cascaded_union
import matplotlib.pyplot as plt
import time
import csv
import product_automata as pa

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                 "/../PythonRobotics/")

from PathPlanning.VisibilityRoadMap import visibility_road_map as vrm



def polygonalWorld(map_array, bloatFactor):  
    map_coords = [x[:] for x in map_array] # deep copy

    originalMap = map_coords.copy()
    map_array_x = [coord[0] for coord in map_coords] + [coord[2] for coord in map_coords]
    map_array_y = [coord[1] for coord in map_coords] + [coord[1] for coord in map_coords]
    minX, maxX = min(map_array_x), max(map_array_x)
    minY, maxY = min(map_array_y), max(map_array_y)

    obstacleVerts = []
    bloatPoly = []
    # construct polygons from given map_array

    bloatFac = bloatFactor
    for i in range(len(map_coords)):
        modifiedMap = originalMap.copy()
        # extend the wall at both end points
        x1 = map_coords[i][0]
        x2 = map_coords[i][2]
        y1 = map_coords[i][1]
        y2 = map_coords[i][3]
        unitVec = [(x1-x2), (y1-y2)]
        unitVec = unitVec/np.linalg.norm(unitVec)
        # modifiedMap(i,:) = []
        modifiedMap.pop(i)
        coords1 = [coord[0:2] for coord in modifiedMap]
        coords2 = [coord[2:4] for coord in modifiedMap]

        res1 = [x1, y1] in coords1
        res2 = [x1, y1] in coords2
        res3 = [x2, y2] in coords1
        res4 = [x2, y2] in coords2

        resA = res1 or res2
        resB = res3 or res4
        onBoundary1 =  x1==minX or x1==maxX or y1==minY or y1==maxY
        onBoundary2 =  x2==minX or x2==maxX or y2==minY or y2==maxY
        

        
        if (x1==x2) or (y1==y2):
            map_coords[i][2:4] = [map_coords[i][2] - bloatFac*unitVec[0], map_coords[i][3] - bloatFac*unitVec[1]]
            map_coords[i][0:2] = [map_coords[i][0] + bloatFac*unitVec[0], map_coords[i][1] + bloatFac*unitVec[1]] 
        else:
            if not resA and not onBoundary1: 
                map_coords[i][0:2] = map_coords[i][0:2] + bloatVec
            if not resB and not onBoundary2:
                map_coords[i][2:4] = map_coords[i][2:4] - bloatVec
           
        # find the 4 vertices at end points of the walls
        normVec = [unitVec[1], -unitVec[0]] # normal vector to the line
        vert1 = [map_coords[i][0] + bloatFac*normVec[0], map_coords[i][1] + bloatFac*normVec[1]]
        vert2 = [map_coords[i][2] + bloatFac*normVec[0], map_coords[i][3] + bloatFac*normVec[1]]
        vert3 = [map_coords[i][2] - bloatFac*normVec[0], map_coords[i][3] - bloatFac*normVec[1]]
        vert4 = [map_coords[i][0] - bloatFac*normVec[0], map_coords[i][1] - bloatFac*normVec[1]]
         
        bloatPoly.append([vert1, vert2, vert3, vert4])  


    # given the vertices for each bloatpoly create polygons
    # using Polygon
    pgons = []
    pgons_coords = []

    for i in range(len(bloatPoly)):
        vert1 = bloatPoly[i][0][:]
        vert2 = bloatPoly[i][1][:]
        vert3 = bloatPoly[i][2][:]
        vert4 = bloatPoly[i][3][:]
        pgons.append(geometry.Polygon([vert1, vert2, vert3, vert4]))

        poly = geometry.Polygon([vert1, vert2, vert3, vert4])
        pgons_coords.append(list(poly.exterior.coords))
    return pgons_coords

def create_roadmap(q_start, q_goal, obstacle_pgons, expand_distance, roadmap_obs, show_animation = False):
    ''' create visibility roadmap

    obstacle_list: list of obstacles with a list of coordinates in cw direction
    expand_distance: how much to bloat obstacle by

    outputs path as a list of x coords and a list of y coords
    '''

    rx, ry = vrm.VisibilityRoadMap(expand_distance, do_plot=False).planning(q_start[0], q_start[1], q_goal[0], q_goal[1], obstacle_pgons, roadmap_obs)

    waypoints = [(rx[i], ry[i]) for i in range(len(rx))]

    if show_animation: 
        plt.axis("equal")
        # start and goalpoints
        plt.plot(q_start[0], q_start[1], "or", label='start')
        plt.plot(q_goal[0], q_goal[1], "og", label='goal')
        for ob in obstacles:
            ob.plot()

        plt.plot(rx, ry, "--c", label='path')
        plt.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
        plt.tight_layout()
        plt.pause(30)
    return waypoints


def get_actions(agent_plan, actions):
    # each column represents a different action. actions = {str: time}
    plan_actions = []

    for step in agent_plan:
        plan_row = [0]*len(actions)
        # "room8"    "arm_idle"    "!scan"
        step_actions = step[1:]
        for cap in step_actions:
            if cap == "scan":
                idx = 0 
                plan_row[idx] = actions[cap]
            elif cap == "camera": 
                idx = 1
                plan_row[idx] = actions[cap]
            elif cap == "pickup":
                idx = 2
                plan_row[idx] = actions[cap]


            

        plan_actions.append(plan_row)
    return plan_actions
           
def findRoomCoords(agent_plan, rooms, actions):
    # rooms = {str: [x,y]}
    plan_coords = {}
    for path_idx in range(len(agent_plan)):
        room_coords = [rooms[step[0]] for step in agent_plan[path_idx]]

        plan_actions = get_actions(agent_plan[path_idx], actions)


        plan_coords[path_idx] = [ room_coords[i] + plan_actions[i] for i in range(len(room_coords))] 

    return plan_coords

def generateWP(agent_plan, actions, path_idx, current_pose, ready, obstacle_list, expand_distance, obs_nodes):
#     % generate waypoints between room coordinates. also include the total
#     % time necessary for each waypoint if actions are required
#     % path n x 3: [x, y, time]
# %     plan_actions =  get_actions(robot, actions, path_idx);
# %     time_actions = sum(plan_actions, 2);
    
    num_actions = len(actions)

    last = []
    if ready == 0:
        if len(agent_plan[path_idx]) == 1:
            plan = [current_pose] + agent_plan[path_idx] # time_actions];

        else:
            plan = [current_pose] + agent_plan[path_idx][:-1] # time_actions(1:end-1)];
        
    elif ready == 1:    # waiting state
        if len(agent_plan[path_idx]) == 1:
            plan = agent_plan[path_idx]  # time_actions];
        else:
            plan = [agent_plan[path_idx][-2]] # time_actions(end-1)];
    else:                # sync state
        plan = [current_pose] + [agent_plan[path_idx][-1]] # time_actions(end)];
    
    path = [plan[0]]
    # print('path', plan)
    for i in range(len(plan)-1):
        p = create_roadmap(plan[i][0:2], plan[i+1][0:2], obstacle_list, expand_distance, obs_nodes)

        if len(p) > 1:
            p = p[1:]   # avoid double counting


        # add action times into path (THIS MAY CHANGE DEPENDING ON HOW I DEFINE THINGS)
        if ready == 0 or ready == 1:
            p = [list(coord) + [0]*num_actions for coord in p]
        else:
            p1 = [list(coord) + [0]*num_actions for coord in p[:-1]]
            p1.append(list(p[-1]) + plan[i+1][2:])
            p = p1
        path += p

    return path

def visitWaypoints(wp, pose, timer, gotopt, Eps, closeEnough, alpha):
    # % Calculate the desired [v, w] controls to drive the robot along a
    # % series of waypoints. WITH ACTIONS 9/26/22
    # % 
    # %   INPUTS
    # %       wp              nx2 matrix of waypoints, where each row is the (x,y)
    # %                       coordinate of a waypoint
    # %       pose            robot's current pose [x y theta]  (1-by-3) 
    # %       gotopt          index of the waypoint being reached
    # %       Eps             epsilon, a value that is arbitrarily chosen
    # %       closeEnough     acceptable amount of deviation from the waypoint
    # %       alpha           scaling coefficient to convert position into Vx and Vy 
    # % 
    # %   OUTPUTS
    # %       v               forward velocity (m/s)
    # %       w               angular velocity (rad/s)
    # %       gotopt_next     next index of waypoint that should be used


    gotopt_next = gotopt;
    wp_x = wp[gotopt][0]
    wp_y = wp[gotopt][1]
    wp_act = sum(wp[gotopt][2:])

    xpos = pose[0]
    ypos = pose[1]
    theta = pose[2] 
    timer_next = -1

    dev = np.sqrt((wp_x - xpos)**2 + (wp_y - ypos)**2)   # deviation from waypoint

    # check if can move onto next waypoint (if current waypoint is reached)
    if dev <= closeEnough and gotopt < len(wp)-1:
        # perform the action at that waypoint TODO
        if wp_act > 0:
            if timer < 0:    # first timestep
                timer_next = 0
            else:
                timer_next = timer
            if timer >= wp_act: # if done with action
                gotopt_next = gotopt+1
                wp_x = wp[gotopt_next][0]
                wp_y = wp[gotopt_next][1]
                timer_next = -1;
            else:
                wp_x = xpos; wp_y = ypos; # set v,w = 0
        else:
            gotopt_next = gotopt+1
            wp_x = wp[gotopt_next][0]
            wp_y = wp[gotopt_next][1]
        
    Vx = alpha*(wp_x - xpos)
    Vy = alpha*(wp_y - ypos)


    v, w = feedbackLin(Vx, Vy, theta, Eps)


    if dev <= closeEnough and gotopt == len(wp)-1:   
        v = 0
        w = 0
        if wp_act > 0:
            if timer >= wp_act: # if done with action
                gotopt_next = gotopt+1
                timer_next = -1
            else:
                timer_next = 0
        else:
            gotopt_next = gotopt+1
            timer_next = -1
    return v,w, gotopt_next, timer_next


def limitCmds(fwdVel, angVel, maxV, wheel2Center):
    wheelVel1 = fwdVel + wheel2Center * angVel # v = R * omega
    wheelVel2 = fwdVel - wheel2Center * angVel

    wheelVels = [wheelVel1, wheelVel2]
    wheelVelmax = max(abs(wheelVel1), abs(wheelVel2))
    # check if saturated
    if wheelVelmax > maxV:
        wheelVels = [x * maxV / wheelVelmax for x in wheelVels]
    # parse out forward and ang velocities
    cmdV = sum(wheelVels) / 2
    cmdW = (wheelVels[0] - wheelVels[1]) / (2 * wheel2Center)

    return cmdV, cmdW

def feedbackLin(vx,vy,theta, epsilon):

    Eps_matrix = np.array([[1, 0], [0, 1/epsilon]])

    R_BI = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
    # print(np.matmul(Eps_matrix, R_BI))
    # print(np.array([[vx], [vy]]))
    vec =  np.matmul(np.matmul(Eps_matrix, R_BI), np.array([[vx], [vy]]))
    vr = vec[0]
    vl = vec[1]

    return float(vr), float(vl)



def find_indices(list_to_check, item_to_find):
    # 1-indexing
    return [idx+1 for idx, value in enumerate(list_to_check) if value == item_to_find]


def robotController2(agent, waypoints, waiting, Eps, closeEnough, alpha, maxV, wheel2Center):
# % robot with roadmap controller. robot is a struct with the following
# % fields:
# %               id: #
# %            plan: [N×3 string]
# %     path_length: N
# %            pose: [3×1 double]
# %     plan_coords: [N×2 double]
# %      current_wp: [1x2 double]
# %         next_wp: [1x2 double]
# %          gotopt: #
# %         counter: #
# %       truthPose: [M×3 double]
# %            path: [Px2 double]


    if waiting == 0:
        gotopt = agent.gotopt;
    # elif waiting == 1:
    #     gotopt = agent.wait_gotopt
    else:
        gotopt = agent.sync_gotopt
    
    pose = agent.pose;
    truthPose = agent.truthPose;
    current_wp = agent.current_wp;
    next_wp = agent.next_wp;
    counter = agent.counter;
    done = 0;
    plan_coords = agent.plan_coords;
    wp_idx = agent.wp_idx;
    timer = agent.action_timer;
        
    
    if gotopt >= len(waypoints) or len(waypoints) == 0 and (agent.action_timer < 0 ):  # robot is done with entire path
        cmdV = 0; cmdW = 0;
        agent.action_timer = -1
        done = 1
    else:

        # waypoint controller
        if gotopt >= len(waypoints) and (agent.action_timer < 0 ):  # need to calculate next path for visitWaypoints
            agent.action_timer = -1;
            done = 1;
            cmdV = 0; cmdW = 0;
        else:
            cmdV,cmdW, gotopt, timer_next = visitWaypoints(waypoints, pose, timer, gotopt, Eps, closeEnough, alpha);
            cmdV,cmdW = limitCmds(cmdV,cmdW,maxV,wheel2Center);
            # print('commands', cmdV, cmdW)
            if timer_next < 0:
                agent.action_timer = -1;
            
            else:
                delT = truthPose[-1][0]-truthPose[-2][0]
                if timer < 0:      # initialize
                    agent.action_timer = delT;
                else:
                    agent.action_timer = agent.action_timer + delT
            

        if gotopt >= len(waypoints)  and (agent.action_timer < 0 or agent.action_timer >= sum(waypoints[-1][2:])):  # need to calculate next waypoints for visitWaypoints
            done = 1;
            agent.action_timer= -1;
            counter = counter + 1;
            if counter >= len(waypoints):
                cmdV = 0; cmdW = 0;

            agent.wp_idx = agent.wp_idx + 1;
    
    agent.pose = pose;
    agent.truthPose = truthPose;
    agent.current_wp = current_wp;
    agent.next_wp = next_wp;
    agent.counter = counter;
    agent.inputs_nom = [cmdV, cmdW];
    agent.done = done;
    
    if waiting == 0:
        agent.gotopt = gotopt;
    # elif waiting == 1:
    #     agent.wait_gotopt = gotopt;
    else:
        agent.sync_gotopt = gotopt;


    return agent


def integrateOdom(config, d, phi):

    # % Given a known initial configuration (x, y, theta) of the robot within a global frame, 
    # % the distance traveled d and the angle the robot turned phi, compute the 
    # % new configuration of the robot.
    # % The method implemented is from Probabilistic Robotics, p. 101
    # %
    # %   INPUTS
    # %       d               Distance returned by DistanceSensorRoomba   1xN [m]
    # %       phi             Angle returned by AngleSensorRoomba         1xN [rad]
    # %       config_init     initial [x y theta]                         3x1
    # % 
    # %   OUTPUTS
    # %       config_all      the new configuration                       3x1
    # %       w               angular velocity (rad/s)



    config_all = config.copy()

    x = config_all[0];
    y = config_all[1];
    theta = config_all[2];


    delD = d; delTheta = phi;
    
    if delTheta == 0: # moving in a straight line
            delMat = [ d * np.cos(theta), d * np.sin(theta), delTheta] 
    else:
        vw_ratio = delD/delTheta;

        delMat = [ -vw_ratio * np.sin(theta) + vw_ratio * np.sin(theta + phi),
                    vw_ratio * np.cos(theta) - vw_ratio * np.cos(theta + phi),
                   delTheta];
    config_all = [float(config_all[i] + delMat[i]) for i in range(len(config_all))]

    # config_all(1:3,i) = config;
    # config_all = config_all(:,2:end);
    
    return config_all

if __name__ == '__main__':
    # generate environment
    map_arrayfile = 'map_9rooms.txt'
    with open(map_arrayfile) as f:
        map_array = [[float(x) for x in line[5:].split()] for line in f if line[0] == "w"]
    with open(map_arrayfile) as f:
        line_array = [[float(x) for x in line[5:].split()] for line in f if line[0] == "l"]
    


    rooms = {"room1": [0.5, 8], "room2": [1,6], "room3":[1,4], "room4":[1.5, 2.5], "room5":[1,1], "room6":[3.5, 3.5], "room7":[5,5],"room8":[4.5,1.5],"room9":[6.5,1]}
    actions = {"scan": 3, "pickup":10, "camera": 5}
    num_actions = len(actions)

    bindings = [[1, 0, 1], [1, 1, 0]]    # [1,3], [1,2] currently excluding self trans

    bloatFactor = 0.3
    expand_distance = 0.01

















