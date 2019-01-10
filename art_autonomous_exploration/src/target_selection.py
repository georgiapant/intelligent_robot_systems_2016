#!/usr/bin/env python

import rospy
import random
import math
import time
import numpy as np
from timeit import default_timer as timer
from utilities import RvizHandler
from utilities import OgmOperations
from utilities import Print
from brushfires import Brushfires
from topology import Topology
import scipy
from path_planning import PathPlanning


# Class for selecting the next best target
class TargetSelection:
    
    # Constructor
    def __init__(self, selection_method):
        self.goals_position = []
        self.goals_value = []
        self.omega = 0.0
        self.radius = 0
        self.method = selection_method

        self.brush = Brushfires()
        self.topo = Topology()
        self.path_planning = PathPlanning()


    def selectTarget(self, init_ogm, coverage, robot_pose, origin, \
        resolution, force_random = False):
        
        target = [-1, -1]

        ######################### NOTE: QUESTION  ##############################
        # Implement a smart way to select the next target. You have the 
        # following tools: ogm_limits, Brushfire field, OGM skeleton,
        # topological nodes.

        # Find only the useful boundaries of OGM. Only there calculations
        # have meaning
        ogm_limits = OgmOperations.findUsefulBoundaries(init_ogm, origin, resolution)

        # Blur the OGM to erase discontinuities due to laser rays
        ogm = OgmOperations.blurUnoccupiedOgm(init_ogm, ogm_limits)
        # print(ogm)
        # Calculate Brushfire field
        tinit = time.time()
        brush = self.brush.obstaclesBrushfireCffi(ogm, ogm_limits)
        Print.art_print("Brush time: " + str(time.time() - tinit), Print.ORANGE)

        # Calculate skeletonization
        tinit = time.time()
        skeleton = self.topo.skeletonizationCffi(ogm, \
                   origin, resolution, ogm_limits)
        Print.art_print("Skeletonization time: " + str(time.time() - tinit), Print.ORANGE)

        # Find topological graph
        tinit = time.time()
        nodes = self.topo.topologicalNodes(ogm, skeleton, coverage, origin, \
                resolution, brush, ogm_limits)
        Print.art_print("Topo nodes time: " + str(time.time() - tinit), Print.ORANGE)
        
        # Visualization of topological nodes
        vis_nodes = []
        for n in nodes:
            vis_nodes.append([
                n[0] * resolution + origin['x'],
                n[1] * resolution + origin['y']
            ])
        RvizHandler.printMarker(\
            vis_nodes,\
            1, # Type: Arrow
            0, # Action: Add
            "map", # Frame
            "art_topological_nodes", # Namespace
            [0.3, 0.4, 0.7, 0.5], # Color RGBA
            0.1 # Scale
        )

    
        # Random point
        if self.method == 'random' or force_random == True:
          target = self.selectRandomTarget(ogm, coverage, brush, ogm_limits)
        #closest point
        elif self.method == 'closest_node':
          target = self.selectClosestTarget(robot_pose, vis_nodes, nodes)
        elif self. method == 'cost_based':
          target = self.selectCostBasedTarget(robot_pose, vis_nodes, brush)
        

        return target

    def selectCostBasedTarget(self,robot_pose, vis_nodes, brush):


        dist_cost = []
        topol_cost = []
        turn_cost = []


        dist_cost_norm = []
        topol_cost_norm = []
        turn_cost_norm = []

        cost = 0
        index_final = 0 

        [rx, ry]=[robot_pose['x'], robot_pose['y']]
        
        for n in vis_nodes:

            #distance cost
            dist_cost.append(pow(pow(n[1]-ry,2) + pow(n[0]-rx,2),0.5)) #distance from node 

            #topological cost
            for i in range(-1,1):
                for j in range(-1,1):
                    if i==0 and j==0:
                        continue

                    cost +=  brush[int(n[0])+i][int(n[1])+j]

            topol_cost.append((1/8) * cost)
  
              #rotational cost
            angle = round(math.degrees(math.atan2(n[1] - ry + 0.00001, n[0]-rx + 0.00001)))
            turn_cost.append(angle%360)


        # normalisation
        
        for cost in dist_cost:
            normalised_cost = 1 - ((cost - min(dist_cost))/(max(dist_cost)/min(dist_cost)))
            dist_cost_norm.append(normalised_cost)

        for cost in topol_cost:
            normalised_cost = 1 - ((cost - min(topol_cost))/(max(topol_cost)-min(topol_cost)))
            topol_cost_norm.append(normalised_cost)

        for cost in turn_cost:
            normalised_cost = 1 - ((cost - min(turn_cost))/(max(turn_cost)-min(turn_cost)))
            turn_cost_norm.append(normalised_cost)    
    
        final_cost = []
        for i in range(len(topol_cost_norm)):
            cost = 4*dist_cost_norm[i] +2* turn_cost_norm[i] +topol_cost_norm[i]
            final_cost.append(cost)



        #find target with the 
        index_final = final_cost.index(max(final_cost))
        target = vis_nodes[index_final] 
        return target


    def selectClosestTarget(self, robot_pose, vis_nodes, nodes):
        [rx, ry]=[robot_pose['x'], robot_pose['y']]

        dist=[]
        for n in vis_nodes:
          dist.append(pow(pow(n[1]-ry,2) + pow(n[0]-rx,2),0.5)) #distance from node

        index_min = dist.index(min(dist))

        next_target=[nodes[index_min][0],nodes[index_min][1]]

        return next_target

    ######################################################################## 
    def selectRandomTarget(self, ogm, coverage, brushogm, ogm_limits):
      # The next target in pixels
        tinit = time.time()
        next_target = [0, 0] 
        found = False
        while not found:
          x_rand = random.randint(0, ogm.shape[0] - 1)
          y_rand = random.randint(0, ogm.shape[1] - 1)
          if ogm[x_rand][y_rand] < 50 and coverage[x_rand][y_rand] < 50 and \
              brushogm[x_rand][y_rand] > 5:
            next_target = [x_rand, y_rand]
            found = True
        Print.art_print("Select random target time: " + str(time.time() - tinit), \
            Print.ORANGE)
        return next_target

