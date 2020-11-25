#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" Inverse Kinematic solver

Solves full-pose inverse kinematics with different methods including ga

Based on ikpy ( https://github.com/Phylliade/ikpy ) for the BFGS optimisation ( added SLSQP and de)

Genetic Algoritm inspired by the paper 

S. Starke, N. Hendrich, D. Krupke and J. Zhang, 
"Evolutionary multi-objective inverse kinematics on highly articulated and humanoid robots," 
2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 
Vancouver, BC, 2017, pp. 6959-6966, doi: 10.1109/IROS.2017.8206620.


"""

#logging
import logging
import sys
logger = logging.getLogger("gaikpy_logger")
module=sys.modules[__name__]
logger.info("Logging started on  " + str(module))


module=sys.modules[__name__]
logger.info("Logging started on  " + str(module))

import scipy.optimize
import numpy as np

import math3d as m3d
import math
import random
import numpy as np

import transforms3d
import math


g_orientation_weight=None


def inverse_kinematic_optimization(chain, target_frame, starting_nodes_angles, regularization_parameter=None,
                                   max_iter=None, second_chain=None, second_target_frame=None,
                                   include_orientation=False, method="L-BFGS-B", orientation_weight=None):
    

    



    """
    Computes the inverse kinematic on the specified target_ with an optimization method
    Based on ikpy, extended with different optimization methods
    
    Parameters
    ----------
    chain: gaikpy.chain.Chain
        The chain used for the Inverse kinematics.
    target_frame: numpy.array
        The desired target.
    starting_nodes_angles: numpy.array
        The initial pose of your chain.
    regularization_parameter: float
        The coefficient of the regularization.
    max_iter: int
        Maximum number of iterations for the optimisation algorithm.
    second_chain: bool
        if true optimization also for second chain
    second_target_frame: numpy.array
        second taget
    include_orientation: bool
        Only postion or a full pose, including orientation, default Fals
    method: str
        optimization method, see chain.inverse_kinematic
    """
      # Compute squared distance to target
    def optimize_position(x):
        """optimizer function 

        Parameters
        ----------
        x : array
            array of joint values

        Returns
        -------
        float
            squared distance between position and goal position
        """
        # y = np.append(starting_nodes_angles[:chain.first_active_joint], x)

        y = chain.active_to_full(x)    
        squared_distance = np.linalg.norm(
            chain.forward_kinematics(y)[:3, -1] - target_position)
        return squared_distance

    def optimize_two_chains(x):
            # y = np.append(starting_nodes_angles[:chain.first_active_joint], x)
            y1 = chain.active_to_full(x)
            squared_distance_1 = np.linalg.norm(
                chain.forward_kinematics(y1)[:3, -1] - target_position)
            y2 = second_chain.active_to_full(x, initial_second_position)
            # squared_distance_2 = np.linalg.norm(second_chain.forward_kinematics(y2)[:3, -1] - second_target)
            squared_distance_2 = np.linalg.norm(second_chain.forward_kinematics(y2)[
                                                :3, -1] - second_target) * 2
            return squared_distance_1 + squared_distance_2

    def optimize_with_regularization(x):
        """optimizer function with regularization parameter

        Parameters
        ----------
        x :  array
            array of joint values

        Returns
        -------
        float
            squared distance between position and goal position
        """
        regularization = np.linalg.norm(
            x - starting_nodes_angles[chain.first_active_joint:])
        return optimize_position(x) + regularization_parameter * regularization

    def optimize_with_orientation(x):
        """optimizer function with orientation

        Parameters
        ----------
        x :  array
            array of joint values

        Returns
        -------
        float
            added and weighted value of distnace and orientation distance
        """
        y = chain.active_to_full(x)

        #Calculate te forward kinematics
        fwk = chain.forward_kinematics(y)

        #Calculate the euclidean distance    
        euclidean_distance = np.linalg.norm(fwk[:3, -1] - target_position)

        # Calculate orientation distance using quaternions 
        recentQuad = m3d.quaternion.UnitQuaternion(
            m3d.orientation.Orientation(fwk[:3, :3]))
        orientation_distance = targetQuad.dist_squared(recentQuad)

        #combine the euclidean distance and the orientation using a weight
        return (euclidean_distance * (1 - orientation_weight) + orientation_distance * orientation_weight)

    # Get the position part of the pose
    target_position = target_frame[:3, 3]

    if orientation_weight == None:
        orientation_weight = 0.1

    # Get the second target frame if there is one
    if second_target_frame is not None:
        second_target = second_target_frame[:3, 3]

    if len(starting_nodes_angles) != chain.number_of_active_joints():
        raise ValueError("Wrong number of starting nodes angles!")

    if starting_nodes_angles is None:
        starting_nodes_angles = [0] * chain.number_of_active_joints()
    
    # If a regularization is selected
    if regularization_parameter is not None:
        chosen_optimizer=optimize_with_regularization

    if second_chain is not None:

        initial_second_position = [0] * len(second_chain.joints)

        chosen_optimizer=optimize_two_chains

    else:
        # If orientation should be included
        if include_orientation:

            # Calculate UnitQuaternion for target 
            targetQuad = m3d.quaternion.UnitQuaternion(m3d.orientation.Orientation(target_frame[:3, :3]))

            #set the optimizer function
            chosen_optimizer=optimize_with_orientation

        else:
            chosen_optimizer=optimize_position

    # Compute bounds
    real_bounds = [link.bounds for link in chain.joints]
    real_bounds = chain.active_from_full(real_bounds)
    logger.debug ("real bounds " + str(len(real_bounds)))
    
    # Manage iterations maximum
    options = {}
    if max_iter is not None:
        options["maxiter"] = max_iter
    
    if (method == "SLSQP"):
    
        res = scipy.optimize.minimize(chosen_optimizer, starting_nodes_angles, method='SLSQP',
                                      bounds=real_bounds, options=options)
    
    # used differential evolution algoritm from scipy, but the results are behind the ga used as method "ga"
    elif (method == "de"):
        atol=0
        tol=0.001
        max_iter=5000
        res = scipy.optimize.differential_evolution(chosen_optimizer, bounds=real_bounds, atol=atol,tol=tol,maxiter=max_iter)
    else:
        logger.debug ("bounds: " +str(real_bounds) + " starting: " + str(starting_nodes_angles))
        res = scipy.optimize.minimize(chosen_optimizer, starting_nodes_angles, method='L-BFGS-B',
                                      bounds=real_bounds, options=options )

        
    return (chain.active_to_full(res.x))


def inverse_kinematic_optimization_multi(chain, j_targets, starting_nodes_angles,
                                         max_iter=None, second_target_frame=None,
                                         method="L-BFGS-B",orientation_weight=None):
    """
    Computes the inverse kinematic for multi joint targets
    
    Parameters
    ----------
    chain: gaikpy.chain.Chain
        The chain used for the Inverse kinematics.
    j_targets: list of targets with (joint number (int), target(numpy.array))
        j_targets must be in the format [(number_of_first_joint_to_target,target_of_the_first_joint [:3, 3]-format),
                                         (number_of_second_joint_to_target,target_of_the_second_joint [:3, 3]-format),
                                           (number_of_third_joint_to_target,target_of_the_third_joint [:3, 3]-format)]
    starting_nodes_angles: numpy.array
        The initial pose of your chain.
    max_iter: int
        Maximum number of iterations for the optimisation algorithm.
    second_target_frame: numpy.array
        second taget
    method: str
        optimization method, see chain.inverse_kinematic
    orientation_weight: float
        orientation weight

    """
    
    #Get the target position
    target_position = target_frame[:3, 3]

    def multi_optimizer(x):

        y1 = chain.active_to_full(x)
        # calculate matrices of the joints
        a_matrices = chain.forward_kinematics(y1, full_kinematics=True)

        # Calculate the difference to all given joints to targets
        # j_targets must be in the format [(number_of_first_joint_to_target,target_of_the_first_joint [:3, 3]-format),
        #                                 (number_of_second_joint_to_target,target_of_the_second_joint [:3, 3]-format),
        #                                   (number_of_third_joint_to_target,target_of_the_third_joint [:3, 3]-format)]
        squared_distances = []
        for j_target in j_targets:
            joint_number, target = j_target
            squared_distances.append(np.linalg.norm(
                a_matrices[joint_number][:3, -1] - target))
            logger.debug ("squared distance: " + str(squared_distances))

        # Sum up all distances
        squared_distance_sum = 0
        for squared_distance in squared_distances:
            squared_distance_sum += squared_distance

        # squared_distance_2 = np.linalg.norm(second_chain.forward_kinematics(y2)[:3, -1] - second_target)
        #squared_distance_2 = np.linalg.norm(second_chain.forward_kinematics(y2)[:3, -1] - second_target) * 2
        # return squared_distance_1 + squared_distance_2
        return squared_distance_sum

    # Compute bounds
    real_bounds = [link.bounds for link in chain.joints]
    # real_bounds = real_bounds[chain.first_active_joint:]
    real_bounds = chain.active_from_full(real_bounds)

    options = {}
    
    # Manage iterations maximum
    if max_iter is not None:
        options["maxiter"] = max_iter
    

    # use "SLSQP" or as standard "BFGS" 
    if (method == "SLSQP"):
        
        res = scipy.optimize.minimize(multi_optimizer, chain.active_from_full(starting_nodes_angles), method='SLSQP',
                                      bounds=real_bounds, options=options)
       
    else:
        res = scipy.optimize.minimize(multi_optimizer, chain.active_from_full(starting_nodes_angles), method='L-BFGS-B',
                                      bounds=real_bounds, options=options)

    return (chain.active_to_full(res.x))

from gaikpy.ga import ga_world
def inverse_kinematic_ga(chain, target_frame, starting_nodes_angles, multiproc_call=False, orientation_weight=None, max_iter=None,
                          include_orientation=False, *args, **kwargs):

    """
    Computes the inverse kinematic on the specified target using a genetic algorithm 

    
    Parameters
    ----------
    chain: gaikpy.chain.Chain
        The chain used for the Inverse kinematics
    target_frame: numpy.array
        The desired target.
    starting_nodes_angles: numpy.array
        The initial pose of your chain.
    multiproc_call: bool
        True for use of multiprocessing 
    orientation_weight: float 
        orientattaion weight against target_
    max_iter: int
        Maximum number of iterations for the local optimisation algorithm.
    """

    #Functions to optimize
    def fwki_with_orientation(x):
        """optimizer function with orientation

        Parameters
        ----------
        x :  array
            array of joint values

        Returns
        -------
        float
            added and weighted value of distnace and orientation distance
        """

        global g_orientation_weight

        orientation_weight = g_orientation_weight

        y = chain.active_to_full(x)

        fwk = chain.forward_kinematics(y,caching=True)
        distance = math.sqrt(np.linalg.norm(fwk[:3, -1] - target_position))

        # Calculate orientation distance using quaternions
        recentQuad = m3d.quaternion.UnitQuaternion(
            m3d.orientation.Orientation(fwk[:3, :3]))        
        orientation_distance = targetQuad.ang_dist(recentQuad)

        # Set the weight for orientation
        # if the parameter is -1 randomize the orientation weight
        if orientation_weight==None:
            orientation_weight = 0.9
        elif orientation_weight==-1:
            orientation_weight = random.random()
        
        #orientation_weight = 0.5
        #return ((distance * (1 - orientation_weight) + orientation_distance * orientation_weight, distance, orientation_distance))
        return (distance * (1 - orientation_weight) + orientation_distance * orientation_weight)

    def fwki_only_position(x):
        # Calculate target_ distance
        y = chain.active_to_full(x)
        fwk = chain.forward_kinematics(y)
        squared_distance = np.linalg.norm(fwk[:3, -1] - target_position)
        return (squared_distance)


    #For full pose, use SLSQP as optimizer of the local minimum
    if include_orientation:
        ga_optimiser="SLSQP"
        #ga_optimizer="L-BFGS-B"
    else:
        ga_optimiser="L-BFGS-B"

    global g_orientation_weight
    g_orientation_weight=orientation_weight

    if starting_nodes_angles is None:
        starting_nodes_angles = chain.inverse_kinematics(target_frame, method="L-BFGS-B")

    if starting_nodes_angles is None:
        raise ValueError("starting_nodes_angles must be specified")

    # Get the position part
    target_position = target_frame[:3, -1]

    targetQuad = m3d.quaternion.UnitQuaternion(
        m3d.orientation.Orientation(target_frame[:3, :3]))

    # be in the limits of the joint bounds
    bound_arr = []
    for t in chain.joints:
        if t.bounds != (None, None):
            bound_arr.append(t.bounds)

    def eval_func(chromosome):
        """ evaluation function used in the genetic algorithm

        Parameters
        ----------
        chromosome : array
            array of joints values

        Returns
        -------
        float
            fitness value
        """

        # if gene not in bounds, fit the value into the bounds
        x = []
        for t, value in enumerate(chromosome):
            down, up = bound_arr[t]
            if value < down:
                value = down
            if value > up:
                value = up
            x.append(value)

        if (include_orientation):
            return(fwki_with_orientation(x))
        else:
            opt = fwki_only_position(x)
            return float(opt)


    def accuracy_reached(chromosome):
        """ Calculates if the defined accuracy of a chromosome, which is here a set of joint values, is reached

        Parameters
        ----------
        chromosome : array
            array of joint values

        Returns
        -------
        boolean
            If defined accuracy is reached or not
        """
        import gaikpy.utils as utils          
        
        y = chain.active_to_full(chromosome)
        #y =chromosome
        fwk = chain.forward_kinematics(y,caching=True)
        
        if dist_acc != None:
            min_distance = math.sqrt(np.linalg.norm(fwk[:3, -1] - target_position))

        if or_acc != None:
            
            #min_or_distance = utils.angle_diff(fwk,target_frame)
            min_or_distance = utils.angle_diff_tait(fwk,target_frame)
            logger.debug (" fwk:   " + str(fwk) + " or_dist" + str(min_or_distance) )

        if dist_acc==None and or_acc==None:
            acc_reached=False
        elif dist_acc==None:
            acc_reached = min_or_distance < or_acc    
        elif or_acc==None:
            acc_reached = min_distance < dist_acc
        else:
            logger.debug ("dist acc:" + str(min_distance))
            acc_reached = min_distance < dist_acc and min_or_distance < or_acc
        return (acc_reached)

    def local_optimiser(joint_values):
        """ local optimiser function for a joint set using scipy function

        Parameters
        ----------
        joint_values : array
            joint value set
        """
        logger.debug (" local opt in: " + str(joint_values))
        optimised_joint_values = chain.inverse_kinematics(target_frame, initial_position=chain.active_to_full(joint_values), 
        method=ga_optimiser, include_orientation=include_orientation, max_iter=max_iter*100)
        logger.debug (" local opt out: " + str(chain.active_from_full(optimised_joint_values)))
        return(chain.active_from_full(optimised_joint_values))
                        

    if include_orientation:
        #Default taken from hyperopt runs
        #See Paper Kerzel et. al., Neuro-Genetic Visuomotor Architecture for Robotic Grasping
        popSize = kwargs.get('popSize', 8)
        mutRate = kwargs.get('mutRate', 0.36)
        
        numElites = kwargs.get('numElites', 3)

        #reachFitness = kwargs.get('reachFitness',0.0001)
        max_iter = kwargs.get('max_iter', 500)
        dist_acc = kwargs.get('dist_acc', .001)
        or_acc = kwargs.get('or_acc', .01)

        if dist_acc is not None or or_acc is not None:
            numGenerations = kwargs.get('numGenerations', 10000)
        else:
            numGenerations = kwargs.get('numGenerations', 20)
    else:
        #Default taken from hyperopt runs
        #See Paper Kerzel et. al., Neuro-Genetic Visuomotor Architecture for Robotic Grasping
        popSize = kwargs.get('popSize', 8)
        mutRate = kwargs.get('mutRate', 0.36)
        numGenerations = kwargs.get('numGenerations', 20)
        numElites = kwargs.get('numElites', 3)

        #reachFitness = kwargs.get('reachFitness',0.0001)
        max_iter = kwargs.get('max_iter', 50)
        dist_acc = kwargs.get('dist_acc', .001)
        #or_acc = kwargs.get('or_acc', .01)
        or_acc=None

    chrom_length = chain.number_of_active_joints()

    # Build bounds array from URDF data
    bound_arr = []
    for t in chain.joints:
        
        if t.bounds != (None, None):
            bound_arr.append(t.bounds)

    gaw = ga_world(chrom_length,bound_arr,eval_func=eval_func,local_optimiser=local_optimiser,accuracy_reached=accuracy_reached)
    fitness,joi = gaw.run()
    joi = chain.active_to_full(joi)
    return(fitness,joi) 

 


