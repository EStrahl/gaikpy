#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" Robot: Class for visualisation of the URDF robot model and the movements with it 
"""

from gaikpy import chain
from urdfpy import URDF
import pyrender  
import trimesh
import numpy as np

import logging
import sys
logger = logging.getLogger("gaikpy_logger")
module=sys.modules[__name__]
logger.info("Logging started on  " + str(module))

class robot(object):

    """ A robot object based on a URDF file for visualisation and data generation

    
        Parameters
        ----------

        path_to_urdf: str
            Urdf filename
        active_links_mask: list of str
            List of the active joints
        use_collision: bool, optional
            Use or not collision
        cfg: list of joint parameters, optional
            Initial joint positions, default: zero positions
        visualisation: bool, optional
            Switch visualisation on, 
        off_screen: bool, optional
            For off screen visualisation (writing as file), defualt: False
        ego_view: bool, optional
            Use predefind ego view or observer view, default: False
        """ 
    
    def __init__(self, path_to_urdf, active_chain_definition,use_collision=False,cfg=None,
                    visualisation=True,off_screen=False,ego_view=True):
        
        self.off_screen=off_screen
        self.r_model = URDF.load(path_to_urdf)
        self.use_collision=use_collision
        self.cfg=cfg
        self.viewer=None
        self.active_chain_definition=active_chain_definition
        self.scene=None
        self.rend=None
        self.r_node_map= {}
        self.visualisation=visualisation
        if visualisation:
            logger.debug ("robot visualisation scene created")
            self.create_scene(off_screen=off_screen,ego_view=ego_view)
        else:
            self.scene=None
        self.target_node=None
    
    def __del__(self):
        '''
        destructor
        '''
        if self.viewer is not None:
            if self.viewer.is_active:
                self.viewer.close_external()
        
        
    def get_robot_trimesh(self):
        if self.use_collision:
            fk = self.r_model.collision_trimesh_fk(self.cfg)
        else:
            fk = self.r_model.visual_trimesh_fk(self.cfg)
        return (fk)

    def create_scene(self,off_screen=False,ego_view=True,pose=None):
        """ Creates a scene with the robot and a camera

        Parameters
        ----------
        off_screen : bool, optional
            False for showing and displaying the robot, by default False
        ego_view : bool, optional
            True for ego view, false for observer camera, by default True
        """
        if pose is None:
            if ego_view:
                pose=[[ 0,  0.5,  -0.35,  0.2],
                            [ -1,  0,  0, 0.0],
                            [ 0,  0.35,  0.5,  1.5],
                            [ 0,  0,  0,  1]]
            else:
                pose=[[ 0,  0,  1,  1.5],
                        [ 1,  0,  0, 0.0],
                        [ 0,  1,  0,  1.2],
                        [ 0,  0,  0,  1]]

        node_map = {}
        self.scene = pyrender.Scene()
        fk=self.get_robot_trimesh()
        for tm in self.get_robot_trimesh():
            pose = fk[tm]
            mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)
            node = self.scene.add(mesh, pose=pose)
            self.r_node_map[tm] = node
        if off_screen:
            if ego_view:
                camera = pyrender.IntrinsicsCamera(400, 400, 1920/2, 1080/2)
                light = pyrender.DirectionalLight(color=[1,1,1], intensity=2e3)
                '''
                self.scene.add(camera, pose=[[ 0,  0.5,  -0.87,  0.2],
                            [ -1,  0,  0, 0.0],
                            [ 0,  0.87,  0.5,  1.5],
                            [ 0,  0,  0,  1]])'''
                self.scene.add(camera, pose=pose)

                self.scene.add(light, pose=  np.eye(4))
            else:
                camera = pyrender.IntrinsicsCamera(800, 800, 1920/2, 1080/2)
                light = pyrender.DirectionalLight(color=[1,1,1], intensity=2e3)
                self.scene.add(camera, pose=pose)

                self.scene.add(light, pose=  np.eye(4))

            self.rend = pyrender.OffscreenRenderer(1920, 1080)
            logger.debug ("renderer initialised: " + str(self.rend))
        else:
            self.viewer=pyrender.Viewer(self.scene, use_raymond_lighting=True, run_in_thread=True)

    def take_shot(self,filename):
        """Takes a shot, if not in off_screen mode, otherwise ignore it

        Parameters
        ----------
        filename : string
            filename for the picture to take
        """
        if self.visualisation:
            #if not in off_screen mode, just ignore it
            if self.off_screen:
                from PIL import Image
                color, _ = self.rend.render(self.scene)
                im = Image.fromarray(color)
                im.save(filename)


    def add_axis_object(self,tms_pose=None):
        """Add a marker with an axis to the scene (for example as target for a movement)

        Parameters
        ----------
        tms_pose : numpy.matrix, optional
            pose, how and where to place marker, by default None
        """
        tms = trimesh.creation.axis(origin_size=.02)

        if tms_pose is None:
            tms_pose = np.array([
                [1.0, 0.0,  0.0, 0.0],
                [0.0, 0.0, -1.0, 0.0],
                [0.0, 1.0,  0.0, 0.0],
                [0.0, 0.0,  0.0, 1.0],
            ]) 
        m = pyrender.Mesh.from_trimesh(tms,smooth=False)
        nb=self.scene.add(m,pose=tms_pose)
        return(nb)

    def update_target(self,sfwr):
        """ Update pose of an existing target

        Parameters
        ----------
        sfwr : numpy.matrix
            new pose
        """
        if self.visualisation:
            if self.target_node is None:
                self.target_node=self.add_axis_object()

            if not self.off_screen:
                self.viewer.render_lock.acquire()
            self.scene.set_pose(self.target_node, pose=sfwr)
            logger.debug ("target pose to : " +str(self.target_node))
            if not self.off_screen:
                self.viewer.render_lock.release()
    
    def update_robot_pose(self,joint_values):
        """Update full robot pose 

        Parameters
        ----------
        joint_values : array
            joint set of all joints
        """
        if self.visualisation:
            self.cfg=self.get_cfg_from_joint_values(joint_values)
            logger.debug ("set new pose configuration cfg: " +str(self.cfg))
            
            fk=self.get_robot_trimesh()

            if not self.off_screen:
                self.viewer.render_lock.acquire()
            
            #set robot pose 
            logger.debug("set robot pose to " + str(fk))       
            for mesh in fk:
                pose = fk[mesh]
                self.r_node_map[mesh].matrix = pose
            
            if not self.off_screen:
                self.viewer.render_lock.release()

    def get_cfg_from_joint_values(self,joint_values=None):
        """Get the cfg data from the joints positions

        Parameters
        ----------
        joint_values : array, optional
            set of joint values, by default None
        """
        names=self.active_chain_definition
        if joint_values is None:
            joint_values=len(names)*[0.0]
        
        ret=dict(zip(names,joint_values))
        
        return(ret)



if __name__ == "__main__":

    # Small example
    # Show the target for 3s 
    # and then move the hand to this position
    # using evaluation data
    from aeikpy import chain_definitions
    import time
    import pickle

    nico_path = "./resources/urdf/nico/complete_modified_bounds.urdf"
    nico_right_chain=chain_definitions.nico_right_chain_active

    nico=robot(nico_path,nico_right_chain)

    #get forward kinematics of the robot and print first link
    fk = nico.r_model.link_fk()
    print("Robot forward kinematics: " + str(fk[nico.r_model.links[1]]))

    with open('./resources/data/nico/nico_right_20_new.p', 'rb') as f:
            sample_set = pickle.load(f)

    
    for sample in sample_set:
    
        (joi,sfwr)=sample
        
        
        print ("Original target pose: \n" + str(joi))
        #display_robot_pose(nb,joi)
        
        nico.update_target(sfwr)

        time.sleep(3)
        
        nico.update_robot_pose(joi)

        time.sleep(3)