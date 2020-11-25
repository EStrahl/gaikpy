#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
   chain
   Uses a chain class from ikpy library and extends it to be capable to use the ga algorithm
"""

from ikpy import chain
from gaikpy import inverse_kinematics as ik
import numpy as np
from ikpy import link as link_lib
import time

#set maximal number of CPUs for multiprocessing
max_cpu=128

# Initialise and start the logging
import logging
import sys
log_formatter = logging.Formatter("%(asctime)s [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s")
root_logger = logging.getLogger("gaikpy_logger")
file_handler = logging.FileHandler('gaikpy.log')
file_handler.setFormatter(log_formatter)
root_logger.addHandler(file_handler)
console_handler = logging.StreamHandler()
console_handler.setFormatter(log_formatter)
root_logger.addHandler(console_handler)
root_logger.setLevel(logging.INFO)
logger=root_logger


class Chain(chain.Chain):
    """The base Chain class, based on ikpy ( https://github.com/Phylliade/ikpy ) chain

    ikpy uses not the URDF terminology but names the URDF joints as "links"
    gaikpy uses the chain class of ikpy but stays with the URDF terminology and names the URDF joints as joints


    Parameters
    ----------

    joints: list[ikpy.link.Link]
        List of the links of the chain
    active_joints_mask: list
        A list of boolean indicating that whether or not the corresponding link is active
    name: str
        The name of the Chain
    
    """

    # implements alias name "joints" for "links" 
    @property
    def joints(self):
        return self.links

    @joints.setter
    def joints(self, value):
        self.links = value


    def __init__(self, joints, active_links_mask=None, name="chain", **kwargs):
        logger.info("--Init chain -- " + name  )
        
        super().__init__(joints, active_links_mask, name, **kwargs)
        self.cache_count=0
        self.local_transformation_cache = {}
        self.initial_position=[0] * len(self.links)
        
    def forward_kinematics(self, joints, full_kinematics=False,caching=False):
        """Returns the transformation matrix of the forward kinematics

        Based on ikpy forward ki, added caching to accelerate computation for ga inverse ki 

        Parameters
        ----------
        joints: list
            The list of the positions of all joints (full joint set).
        full_kinematics: bool
            Return the transformation matrices of each joint
        caching
            True if caching of the translation parameters should be used (useful for algorithms lieke genetic ones)

        Returns
        -------
        frame_matrix:
            The transformation matrix
        """
        
        frame_matrix = np.eye(4)

        if full_kinematics:
            frame_matrixes = []

        if len(self.links) != len(joints):
            raise ValueError(
                "Your joints vector length is {} but you have {} links".format(len(joints), len(self.links)))

        for index, (link, joint_angle) in enumerate(zip(self.links, joints)):
            
            #If exists, get the local transformation matrix out of the cache
            parameters={}
            parameters["theta"]=joint_angle
            if caching:
                local_cache=None
                cache_index=(link.name,str(round(joint_angle, 9)))
                try:
                    local_cache=self.local_transformation_cache[cache_index]
                    logger.debug ("Cache use: " +str(cache_index))
                    self.cache_count+=1
                except:
                    #local_cache=np.asarray(link.get_transformation_matrix(joint_angle))
                    local_cache=np.asarray(link.get_link_frame_matrix(parameters))
                    self.local_transformation_cache[cache_index]=local_cache
                    logger.debug (" local cache: " + str(len(self.local_transformation_cache)))
            else:
                #local_cache=np.asarray(link.get_transformation_matrix(joint_angle))
                local_cache=np.asarray(link.get_link_frame_matrix(parameters))

            frame_matrix = np.dot(frame_matrix, local_cache)
            
            #frame_matrix = np.matmul(frame_matrix, link.get_transformation_matrix(joint_angle))
            if full_kinematics:
                # rotation_axe = np.dot(frame_matrix, link.rotation)
                frame_matrixes.append(frame_matrix)

        # Return the matrix, or matrixes
        logger.debug ("Cache used: " +str(self.cache_count))
        if full_kinematics:
            return frame_matrixes
        else:
            return frame_matrix

    def inverse_kinematics(self, target, initial_position=None, second_chain=None, second_target_frame=None,
                           method="L-BFGS-B", orientation_weight=None, multiproc=False, **kwargs):

        """Computes the inverse kinematic on the specified target

        Based on ikpy, extended with other optimization methods and genetic algorith

        Parameters
        ----------
        target: numpy.array
            The frame target of the inverse kinematic, in meters. It must be 4x4 transformation matrix

        initial_position: numpy.array
            Optional : the initial position of each joint of the chain. Defaults to 0 for each joint

        second_chain: bool 
            Optional : True if a second chain should be optimized. Default False

        second_target_frame : numpy.array
            Optional : Second targets goal. Default None.

        method : str
            Optional : Method of calculation to use ("L-BFGS-B","SLSQP","ga"(genetic algorithm),"multi"(multi target optimization)
        
        orientation_weight : float
            Optional : Weight for orientation accuracy against postion. Default None (automatic)

        Returns
        -------
        list:
            The list of the positions of each joint according to the target. Note : Inactive joints are in the list.
        """
        
        logger.debug( "Initial position: " + str(initial_position))

        # Checks on input
        if method!="multi":
            target = np.array(target)
            if target.shape != (4, 4):
                raise ValueError("Your target must be a 4x4 transformation matrix")

        if initial_position is not None:
            if (len(self.initial_position)!=len(initial_position)):
                raise ValueError("Wrong length of initial position! From self: " + str(len(self.initial_position) ) +
                      " Given: " + str(len(initial_position) ))
            self.initial_position=initial_position
            

        # optimization methods
        if (method == "L-BFGS-B") or (method == "SLSQP") or (method == "de"):


            return ik.inverse_kinematic_optimization(self, target, 
                                       starting_nodes_angles=self.get_active_initial_position(),
                                                     second_chain=second_chain, second_target_frame=second_target_frame,
                                                     method=method, orientation_weight=orientation_weight, **kwargs)

        # multijoint optimization
        if (method == "multi"):

                return ik.inverse_kinematic_optimization_multi(self, target, starting_nodes_angles=self.initial_position,
                                                         method=method, orientation_weight=orientation_weight, **kwargs)

        # genetic algoritm methods
        if (method == "ga_simple"):
            
            if multiproc:
                def f(procnum,return_dict,found_something):
                    #return_dict[procnum]=((1,2,procnum))
                    #return_dict[procnum]=(1,procnum)
                    #return_dict[procnum] = ik.inverse_kinematic_ga(self, target, multiproc_call=True, starting_nodes_angles=initial_position,
                    #                       second_chain=second_chain, second_target_frame=second_target_frame,
                    #                       method=method, orientation_weight=orientation_weight, **kwargs)
                    
                    result = ik.inverse_kinematic_ga(self, target, multiproc_call=True, 
                                    starting_nodes_angles=self.get_active_initial_position(),
                                           second_chain=second_chain, second_target_frame=second_target_frame,
                                           method=method, orientation_weight=orientation_weight, **kwargs)
                    found_something.set()
                    return_dict[procnum]=result

                from multiprocessing import Process,cpu_count,Manager,Event
                manager = Manager()
                return_dict = manager.dict()
                jobs = []
                found_something = Event()

                #if cpu_count()>max_cpu:
                #    cpu_num=12
                #else:
                #    cpu_num=cpu_count()-1
                
                if max_cpu > cpu_count()-2:
                    cpu_num=cpu_count()-2
                else:
                    cpu_num=max_cpu
                if cpu_num<1:
                    cpu_num=1

                cpu_num=int(cpu_num)
                for i in range(cpu_num):
                    p = Process(target=f, args=(i,return_dict,found_something))
                    jobs.append(p)
                    #p.daemon=True
                    p.start()
                #for proc in jobs:
                #    proc.join()
                
                success=False
                while not success:
                    logger.debug (" Nothing found so far.")
                    time.sleep(.1)
                    
                    if found_something.is_set():
                        logger.debug ("Found something. Lets kill the jobs.")
                        time.sleep(.1)
                        for proc in jobs:
                            proc.terminate()
                        success=True

                
                #Get the best one from all calculations
                logger.debug ("Dictionary of all (multithreded) calculations " + str(return_dict))
                val,joints=return_dict[min(return_dict, key=return_dict.get)]
                
            else: 
                dummy,joints=ik.inverse_kinematic_ga(self, target, 
                           starting_nodes_angles=self.get_active_initial_position(),
                                           second_chain=second_chain, second_target_frame=second_target_frame,
                                           method=method, orientation_weight=orientation_weight, **kwargs)
            #joints = self.active_to_full(joints)
            logger.debug("Best joint set : " + str(joints))
            return (joints)
    
    def active_to_full(self, active_joints):
        """Converts an active joint set ( joints that can move) into the full joint set 

        Parameters
        ----------
        active_joints : array
            array of values of the active joints

        Returns
        -------
        array
            array of the full joint set

        Raises
        ------
        ValueError
            If the given array is too big too small or otherwise in wrong format
        """
        if (self.number_of_active_joints())!=len(active_joints):
            raise ValueError("Wrong input in active_to_full! given join no  = " + str(len(active_joints)) 
                                + "  - number of all active links " + str(self.number_of_active_joints()))
        logger.debug ("initial joint position: " + str(self.initial_position))
        full_joints = np.array(self.initial_position, copy=True, dtype=np.float)
        logger.debug ("full joints 1: " + str(full_joints))
        np.place(full_joints, self.active_links_mask, active_joints)
        logger.debug ("full_joints 2: "  + str(full_joints))
        return full_joints
    
    def active_from_full(self, joints):
        """Converts a full joint into an active joint set 

        Parameters
        ----------
        full_joints : array
            array of values of the full joint set

        Returns
        -------
        array
            array of the active

        Raises
        ------
        ValueError
            If the given array is too big too small or otherwise in wrong format
        """
        if (len(joints)!=self.number_of_all_joints()):
            raise ValueError("Wrong input in active_from_full_function!  joints len  = " + str(len(joints)) 
                                + "  - number of all links " + str(self.number_of_all_joints()))
        active_links=np.compress(self.active_links_mask, joints, axis=0)
        if (len(active_links)!=self.number_of_active_joints()):
            raise ValueError("Wrong active link length!")

        return(active_links) 
    
    def get_active_initial_position(self,position=None):
        """Returns the initial positions of the active joint set or the positions itself if given

        Parameters
        ----------
        position : 
            array of values of the active joints or None

        Returns
        -------
        array
            initials positions of the active joint set or the given position array

        Raises
        ------
        ValueError
            If the given array is too big too small or otherwise in wrong format
        """
        if position is None:
            active_position=np.compress(self.active_links_mask, self.initial_position, axis=0)
        else:
            #active_position=np.compress(self.active_links_mask,position, axis=0)
            if (len(position)!=self.number_of_active_joints()):
                raise ValueError("Wrong active link length!")
            active_position=position
        return(active_position) 
    
    def number_of_all_joints(self):
        """Returns the number of all joints
        """
        return(len(self.active_links_mask))

    def number_of_active_joints(self):
        """Returns the number of the active joints
        """
        return(np.count_nonzero(self.active_links_mask))

    def get_all_joint_names(self):
        """Returns an array with the joint names
        """
        link_names = [link.name for link in self.links]
        return (link_names)

    def get_all_active_joint_names(self):
        link_names = [link.name for link in self.active_from_full(self.links)]
        return (link_names)

    def get_dict_active_joints(self,joint_values=None):
        names=self.get_all_active_joint_names()
        if joint_values is None:
            joint_values=len(names)*[0.0]
        ret=dict(zip(names,joint_values))
        return(ret)
    
    @classmethod
    def from_urdf_file(cls,urdf_file, base_elements=None,active_joints_mask=None, name='chain',symbolic=True):
        """[summary]

        Parameters
        ----------
        urdf_file : string
            filename with path to URDF file
        base_elements : array, optional
            arry with joint names of the chain, by default None
        active_joints_mask : array, optional
            boolean array of active (True) and inctive (False) joints, by default None
        name : str, optional
            name of the chain, by default 'chain'
        symbolic : bool, optional
            should caclculation be done symbolic with sympy, by default True
        """
        chain_from_urdf = super().from_urdf_file(urdf_file, base_elements, active_links_mask=active_joints_mask, 
                                    name=name, symbolic=symbolic)
        return(chain_from_urdf)


